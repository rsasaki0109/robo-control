//! Unitree G1 (12-DoF legs) walking on RNE with robo-control's LIPM walkers.
//!
//! Every 10 ms robo-control's [`LipmWalker`] reads the measured CoM and returns
//! the desired ZMP and footholds (the same walker drives the point-mass
//! scenario in robo-control). Every 2 ms the ZMP, clamped to the support foot,
//! gives the desired CoM acceleration `w^2 (c - p)`; RNE's `rne_wbc` (with
//! `enforce_contact_cones`) or this crate's [`ForceQpWbc`] turns it, an
//! upright-pelvis task and the swing-leg inverse kinematics into joint torques;
//! and `rne_dynamics::contact_step` advances the floating-base model with hard
//! contacts at the four sole spheres of each foot.

use nalgebra::{Rotation3, UnitQuaternion, Vector3};
use rne_dynamics::{
    center_of_mass, com_jacobian, contact_step, ArticulatedModel, ContactPoint, ContactStepConfig, JointPdControl,
};
use rne_ecs::{Entity, World};
use rne_math::Vec3;
use rne_robot::{FloatingBase, Robot};
use rne_urdf_import::{
    fuse_urdf_fixed_children, parse_urdf_document, spawn_urdf_document_with_config, UrdfSpawnConfig,
};
use robo_control::scenarios::humanoid::{LipmParams, LipmWalker, Strategy, PUSH, START_S};

const G1_URDF: &str = include_str!("../assets/g1_12dof.rne.urdf");
/// Sole contact spheres of each ankle-roll link (from the URDF), radius 5 mm.
pub const SOLE_POINTS_M: [[f64; 3]; 4] = [
    [-0.05, 0.025, -0.03],
    [-0.05, -0.025, -0.03],
    [0.12, 0.03, -0.03],
    [0.12, -0.03, -0.03],
];
const SOLE_RADIUS_M: f64 = 0.005;
/// Ankle origin height above the ground when the sole is flat.
pub const ANKLE_HEIGHT_M: f64 = 0.035;
/// Foot centre ahead of the ankle origin.
pub const FOOT_CENTER_X_M: f64 = 0.035;
const LEG_JOINTS: [&str; 6] = ["hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll"];

/// The articulated G1 legs model.
#[derive(Debug, Clone)]
pub struct G1 {
    /// floating-base model, Z up
    pub model: ArticulatedModel,
    /// actuated index of each leg joint: `[left, right]`, order of [`LEG_JOINTS`]
    pub leg_dofs: [[usize; 6]; 2],
    /// ankle-roll link entity and kinematic index `[left, right]`
    pub feet: [(Entity, usize); 2],
    /// links drawn in the GIF: pelvis, then per leg hip, knee, ankle
    pub draw_links: Vec<usize>,
    /// effort limit per actuated joint
    pub effort_nm: Vec<f64>,
    /// total mass
    pub mass_kg: f64,
}

impl G1 {
    /// Loads the vendored 12-DoF G1.
    pub fn load() -> Self {
        let document = parse_urdf_document(G1_URDF).expect("parse G1 URDF");
        let mut world = World::new();
        let config = UrdfSpawnConfig {
            attach_colliders: false,
            attach_mesh_colliders: false,
            self_collisions: false,
            use_declared_inertial_masses: true,
            ..UrdfSpawnConfig::default()
        };
        let spawned = spawn_urdf_document_with_config(&mut world, &document, config).expect("spawn G1");
        fuse_urdf_fixed_children(&mut world, &document.robot, &spawned);
        let base_link = world.get::<Robot>(spawned.robot).expect("robot").base_link;
        world.entity_mut(base_link).insert(FloatingBase);
        let model = ArticulatedModel::from_robot_with_gravity(&world, spawned.robot, Vec3::new(0.0, 0.0, -9.81))
            .expect("model");
        assert_eq!(model.base_dof(), 6);
        let dof = |name: &str| -> usize { model.kinematic().dof_index_of_joint(spawned.joints[name]).expect("dof") };
        let leg_dofs = [0, 1].map(|s| {
            let side = if s == 0 { "left" } else { "right" };
            LEG_JOINTS.map(|j| dof(&format!("{side}_{j}_joint")))
        });
        let link = |name: &str| -> (Entity, usize) {
            let e = model.kinematic().link_entity_by_name(name).expect(name);
            (e, model.kinematic().link_index(e).expect("index"))
        };
        let feet = [link("left_ankle_roll_link"), link("right_ankle_roll_link")];
        let mut draw_links = vec![link("pelvis").1];
        for side in ["left", "right"] {
            for l in ["hip_pitch_link", "knee_link", "ankle_roll_link"] {
                draw_links.push(link(&format!("{side}_{l}")).1);
            }
        }
        let mut effort_nm = vec![0.0; model.nv() - 6];
        for (name, joint) in &spawned.joints {
            if let Some(d) = model.kinematic().dof_index_of_joint(*joint) {
                let lim = document
                    .robot
                    .joints
                    .iter()
                    .find(|j| &j.name == name)
                    .and_then(|j| j.limit.as_ref())
                    .map_or(50.0, |l| l.max_effort_nm);
                effort_nm[d] = lim;
            }
        }
        let mass_kg = (0..model.link_count())
            .filter_map(|i| model.link_inertia(i))
            .map(|i| i.mass_kg)
            .sum();
        Self {
            model,
            leg_dofs,
            feet,
            draw_links,
            effort_nm,
            mass_kg,
        }
    }

    /// Base rotation (body to world) of configuration `q`.
    pub fn base_rotation(q: &[f64]) -> Rotation3<f64> {
        Rotation3::from_euler_angles(q[3], q[4], q[5])
    }

    /// Ankle (position, rotation) of foot `side` (0 left, 1 right).
    pub fn ankle(&self, q: &[f64], side: usize) -> (Vector3<f64>, Rotation3<f64>) {
        let fk = self.model.kinematic().forward_kinematics(q).expect("fk");
        let t = fk.transforms()[self.feet[side].1];
        let r = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            t.rotation.w,
            t.rotation.x,
            t.rotation.y,
            t.rotation.z,
        ))
        .to_rotation_matrix();
        (Vector3::new(t.translation.x, t.translation.y, t.translation.z), r)
    }

    /// Damped-least-squares IK of leg `side` so its ankle reaches `target` with a
    /// flat sole facing +x. Only that leg's joints in `q` change.
    pub fn leg_ik(&self, q: &mut [f64], side: usize, target: Vector3<f64>, iters: usize) {
        let dofs = self.leg_dofs[side];
        let err = |q: &[f64]| -> nalgebra::SVector<f64, 6> {
            let (p, r) = self.ankle(q, side);
            let e = (r.inverse()).scaled_axis(); // to identity
            let ep = target - p;
            nalgebra::SVector::<f64, 6>::from_column_slice(&[ep.x, ep.y, ep.z, e.x * 0.5, e.y * 0.5, e.z * 0.5])
        };
        for _ in 0..iters {
            let e0 = err(q);
            if e0.norm() < 1e-6 {
                break;
            }
            let mut j = nalgebra::SMatrix::<f64, 6, 6>::zeros();
            for (c, &d) in dofs.iter().enumerate() {
                let h = 1e-6;
                q[6 + d] += h;
                let e1 = err(q);
                q[6 + d] -= h;
                j.set_column(c, &((e0 - e1) / h));
            }
            let jt = j.transpose();
            let dq = jt
                * (j * jt + nalgebra::SMatrix::<f64, 6, 6>::identity() * 1e-6)
                    .try_inverse()
                    .unwrap()
                * e0;
            for (c, &d) in dofs.iter().enumerate() {
                q[6 + d] += dq[c].clamp(-0.3, 0.3);
            }
        }
    }

    /// Standing configuration with the pelvis at `pelvis_z` and ankles at `(0, ±y)`.
    pub fn stand(&self, pelvis_z: f64, ankle_y: f64) -> Vec<f64> {
        let mut q = vec![0.0; self.model.nv()];
        q[2] = pelvis_z;
        for side in 0..2 {
            q[6 + self.leg_dofs[side][3]] = 0.6;
            q[6 + self.leg_dofs[side][0]] = -0.3;
            q[6 + self.leg_dofs[side][4]] = -0.3;
        }
        self.leg_ik(&mut q, 0, Vector3::new(0.0, ankle_y, ANKLE_HEIGHT_M), 100);
        self.leg_ik(&mut q, 1, Vector3::new(0.0, -ankle_y, ANKLE_HEIGHT_M), 100);
        q
    }
}

/// One contact point for [`ForceQpWbc`].
#[derive(Debug, Clone, Copy)]
pub struct SolePoint {
    /// link entity
    pub link: Entity,
    /// kinematic link index
    pub index: usize,
    /// point in the link frame
    pub local: Vec3,
}

/// Whole-body control that keeps the contact forces physical: a box QP first
/// distributes the centroidal wrench over the sole points with non-negative
/// normal forces and bounded friction (so the centre of pressure stays inside
/// the sole), then a weighted least-squares solve finds joint accelerations
/// consistent with those forces, and inverse dynamics gives the torques.
///
/// Written before `rne_wbc` could constrain contact forces inside its solve
/// (`enforce_contact_cones`, added in RNE #553); kept as an independent
/// cross-check. Without that option `rne_wbc` projects the forces onto the
/// friction cones only after solving, so its torques can assume a sole edge
/// pulling on the ground, which tips a flat-footed biped.
#[derive(Debug, Clone)]
pub struct ForceQpWbc {
    /// total mass
    pub mass_kg: f64,
    /// approximate centroidal inertia diagonal
    pub inertia_kgm2: [f64; 3],
    /// friction coefficient
    pub mu: f64,
}

/// Output of [`ForceQpWbc::solve`].
#[derive(Debug, Clone)]
pub struct WbcOut {
    /// actuated torques
    pub tau: Vec<f64>,
    /// contact forces (world)
    pub forces: Vec<Vector3<f64>>,
}

impl ForceQpWbc {
    /// Solves for torques. `alpha_body` is the desired base angular acceleration
    /// in the base frame; `qdd_joint` the desired actuated accelerations (posture),
    /// weighted per joint by `w_joint`.
    #[allow(clippy::too_many_arguments)]
    pub fn solve(
        &self,
        model: &ArticulatedModel,
        q: &[f64],
        qd: &[f64],
        contacts: &[SolePoint],
        com: Vector3<f64>,
        acc_com: Vector3<f64>,
        alpha_body: Vector3<f64>,
        qdd_joint: &[f64],
        w_joint: &[f64],
    ) -> WbcOut {
        use nalgebra::{DMatrix, DVector};
        let nv = model.nv();
        let rot = G1::base_rotation(q);
        let fk = model.kinematic().forward_kinematics(q).expect("fk");
        let pts: Vec<Vector3<f64>> = contacts
            .iter()
            .map(|c| {
                let t = fk.transforms()[c.index];
                crate::na(t.translation + t.rotation * c.local)
            })
            .collect();
        let n = contacts.len();
        // ---- stage 1: contact forces
        let ic = nalgebra::Matrix3::from_diagonal(&Vector3::from(self.inertia_kgm2));
        let ldot = rot * (ic * alpha_body);
        let force = (acc_com + Vector3::new(0.0, 0.0, 9.81)) * self.mass_kg;
        let mut a = DMatrix::zeros(6, 3 * n);
        for (j, p) in pts.iter().enumerate() {
            let r = p - com;
            for k in 0..3 {
                a[(k, 3 * j + k)] = 1.0;
            }
            // r x f
            a[(3, 3 * j + 1)] = -r.z;
            a[(3, 3 * j + 2)] = r.y;
            a[(4, 3 * j)] = r.z;
            a[(4, 3 * j + 2)] = -r.x;
            a[(5, 3 * j)] = -r.y;
            a[(5, 3 * j + 1)] = r.x;
        }
        let w = DVector::from_vec(vec![force.x, force.y, force.z, ldot.x, ldot.y, ldot.z]);
        let sw = DMatrix::from_diagonal(&DVector::from_vec(vec![1.0, 1.0, 1.0, 3.0, 3.0, 1.0]));
        let h = a.transpose() * &sw * &a + DMatrix::identity(3 * n, 3 * n) * 1e-4;
        let g = -(a.transpose() * &sw * w);
        let fz_nom = self.mass_kg * 9.81 / n.max(1) as f64;
        let mut lo = DVector::zeros(3 * n);
        let mut hi = DVector::zeros(3 * n);
        for j in 0..n {
            lo[3 * j] = -self.mu * fz_nom;
            hi[3 * j] = self.mu * fz_nom;
            lo[3 * j + 1] = -self.mu * fz_nom;
            hi[3 * j + 1] = self.mu * fz_nom;
            lo[3 * j + 2] = 0.0;
            hi[3 * j + 2] = 4.0 * self.mass_kg * 9.81;
        }
        let f = robo_control::mpc::box_qp_newton_start(&h, &g, &lo, &hi, 300);
        let forces: Vec<Vector3<f64>> = (0..n)
            .map(|j| Vector3::new(f[3 * j], f[3 * j + 1], f[3 * j + 2]))
            .collect();
        // ---- stage 2: accelerations consistent with the forces
        let m = rne_dynamics::mass_matrix(model, q).expect("M");
        let hb = rne_dynamics::non_linear_effects(model, q, qd).expect("h");
        let motions = rne_dynamics::link_motions(model, q, qd).expect("motions");
        let mm = DMatrix::from_fn(nv, nv, |r, c| m.get(r, c));
        let mut jtf = DVector::zeros(nv);
        let mut rows: Vec<(Vec<f64>, f64, f64)> = Vec::new(); // (row, rhs, weight)
        for (j, c) in contacts.iter().enumerate() {
            let jac = rne_dynamics::frame_jacobian(model, q, c.link, c.local).expect("J");
            let mo = &motions[c.index];
            let r = crate::na(mo.world_transform.rotation * c.local);
            let w_ = crate::na(mo.angular_velocity_world_rad_s);
            let bias = crate::na(mo.linear_acceleration_world_m_s2)
                + crate::na(mo.angular_acceleration_world_rad_s2).cross(&r)
                + w_.cross(&w_.cross(&r));
            for k in 0..3 {
                let row: Vec<f64> = (0..nv).map(|cc| jac.get(k, cc)).collect();
                let vel: f64 = row.iter().zip(qd).map(|(a, b)| a * b).sum();
                rows.push((row, -bias[k] - 20.0 * vel, 1.0e3));
                for cc in 0..nv {
                    jtf[cc] += jac.get(k, cc) * forces[j][k];
                }
            }
        }
        // floating-base rows of the equations of motion
        for r in 0..6 {
            rows.push(((0..nv).map(|c| mm[(r, c)]).collect(), jtf[r] - hb[r], 1.0e2));
        }
        for i in 0..nv - 6 {
            let mut row = vec![0.0; nv];
            row[6 + i] = 1.0;
            rows.push((row, qdd_joint[i], w_joint[i]));
        }
        for k in 0..3 {
            let mut row = vec![0.0; nv];
            row[3 + k] = 1.0;
            rows.push((row, alpha_body[k], 1.0));
        }
        let mut ata = DMatrix::identity(nv, nv) * 1e-6;
        let mut atb = DVector::zeros(nv);
        for (row, rhs, wt) in &rows {
            for i in 0..nv {
                if row[i] == 0.0 {
                    continue;
                }
                atb[i] += wt * row[i] * rhs;
                for k in 0..nv {
                    ata[(i, k)] += wt * row[i] * row[k];
                }
            }
        }
        let qdd = ata.cholesky().expect("SPD").solve(&atb);
        let tau_full = &mm * &qdd + DVector::from_vec(hb) - jtf;
        WbcOut {
            tau: tau_full.rows(6, nv - 6).iter().copied().collect(),
            forces,
        }
    }
}

/// Which whole-body controller turns the walker's CoM acceleration into torques.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WbcBackend {
    /// [`ForceQpWbc`] in this crate.
    ForceQp,
    /// RNE's `rne_wbc::WholeBodyController` with `enforce_contact_cones` and
    /// in-solve torque limits.
    Rne,
}

/// Recorded frame.
#[derive(Debug, Clone)]
pub struct G1Frame {
    /// time [s]
    pub t: f64,
    /// pelvis, then per leg (left, right) hip, knee, ankle
    pub points: Vec<[f64; 3]>,
    /// top of the torso (pelvis + 0.45 m along the pelvis up axis)
    pub torso_top: [f64; 3],
    /// CoM
    pub com: [f64; 3],
    /// applied ZMP
    pub zmp: [f64; 2],
    /// DCM
    pub dcm: [f64; 2],
    /// footsteps placed so far (centres)
    pub steps: Vec<[f64; 2]>,
    /// swing target (centre)
    pub next: [f64; 2],
    /// whether a push is active
    pub pushing: bool,
    /// fallen
    pub fallen: bool,
}

/// Result of a G1 run.
#[derive(Debug, Clone)]
pub struct G1Run {
    /// strategy name
    pub name: String,
    /// frames every 20 ms
    pub frames: Vec<G1Frame>,
    /// fall time
    pub fell_at_s: Option<f64>,
    /// metrics
    pub metrics: Vec<(String, f64)>,
}

/// Gait used on the G1: 0.45 s steps of 0.12 m (0.27 m/s), feet 0.236 m apart,
/// a conservative support region inside the sole.
pub fn g1_params(com_height_m: f64, step_width_m: f64) -> LipmParams {
    LipmParams {
        com_height_m,
        step_s: 0.45,
        step_length_m: 0.12,
        step_width_m,
        foot_size_m: (0.16, 0.05),
        max_reach_m: 0.35,
    }
}

/// Runs the G1 with `strategy` for `duration_s`, optionally pushed with the same
/// velocity change as the point-mass scenario ([`PUSH`]), on RNE's whole-body
/// controller ([`WbcBackend::Rne`]).
pub fn run(strategy: Strategy, duration_s: f64, push: bool) -> G1Run {
    run_with(strategy, duration_s, push, WbcBackend::Rne)
}

/// [`run`] with a choice of whole-body controller.
pub fn run_with(strategy: Strategy, duration_s: f64, push: bool, backend: WbcBackend) -> G1Run {
    let g1 = G1::load();
    let model = &g1.model;
    let nv = model.nv();
    let na = nv - 6;
    let ankle_y = 0.118;
    let mut q = g1.stand(0.72, ankle_y);
    let lowest = (0..2).map(|s| g1.ankle(&q, s).0.z).fold(f64::INFINITY, f64::min);
    q[2] += ANKLE_HEIGHT_M - lowest + 0.001;
    let mut qd = vec![0.0; nv];
    let com0 = center_of_mass(model, &q).expect("com");
    let p = g1_params(com0.z, 2.0 * ankle_y);
    let dt = 0.002;
    let w = p.omega();
    let walker_dt = 0.01;
    let walker_every = (walker_dt / dt) as usize;
    let mut walker = LipmWalker::new(
        p,
        strategy,
        walker_dt,
        [FOOT_CENTER_X_M, -ankle_y],
        (duration_s / p.step_s) as usize + 8,
    );
    let wbc = ForceQpWbc {
        mass_kg: g1.mass_kg,
        inertia_kgm2: [1.5, 1.4, 0.4],
        mu: 0.6,
    };
    let rne_controller = rne_wbc::WholeBodyController::new(rne_wbc::WholeBodyConfig {
        torque_limits_nm: Some(g1.effort_nm.clone()),
        enforce_torque_limits: true,
        enforce_contact_cones: true,
        com_weight: 1.0e4,
        posture_weight: 10.0,
        ..rne_wbc::WholeBodyConfig::default()
    });
    let step_cfg = ContactStepConfig {
        step_time_s: dt,
        ..ContactStepConfig::default()
    };
    let sole_local: Vec<Vec3> = SOLE_POINTS_M
        .iter()
        .map(|s| Vec3::new(s[0], s[1], s[2] - SOLE_RADIUS_M))
        .collect();
    let mut warm: Option<Vec<[f64; 3]>> = None;
    let mut q_ref = q.clone();
    let mut frames = Vec::new();
    let mut fell = None;
    let mut ev = Vec::new();
    let mut last_out = None;
    let mut last_step = 0usize;
    let mut last_swing: Option<usize> = None;
    let steps = (duration_s / dt).round() as usize;
    for k in 0..steps {
        let t = k as f64 * dt;
        let com = center_of_mass(model, &q).expect("com");
        let jc = com_jacobian(model, &q).expect("com jacobian");
        let cd: Vec<f64> = (0..3).map(|r| (0..nv).map(|c| jc.get(r, c) * qd[c]).sum()).collect();
        // ---- walker (robo-control) at 100 Hz
        if k % walker_every == 0 || last_out.is_none() {
            let o = walker.update(t, [com.x, com.y], [cd[0], cd[1]]);
            if o.step_index != last_step {
                // tell the walker where the new stance foot really landed
                last_step = o.step_index;
                let (a, r) = g1.ankle(&q, if o.stance_left { 0 } else { 1 });
                let c = a + r * Vector3::new(FOOT_CENTER_X_M, 0.0, 0.0);
                walker.set_stance_position([c.x, c.y]);
                last_out = Some(walker.update(t, [com.x, com.y], [cd[0], cd[1]]));
            } else {
                last_out = Some(o);
            }
        }
        let out = last_out.expect("walker output");
        let (lo, hi) = out.support;
        let zmp = [out.zmp_des[0].clamp(lo[0], hi[0]), out.zmp_des[1].clamp(lo[1], hi[1])];
        let acc = Vector3::new(
            w * w * (com.x - zmp[0]),
            w * w * (com.y - zmp[1]),
            100.0 * (p.com_height_m - com.z) - 20.0 * cd[2],
        );
        // ---- contacts of the sole spheres with the floor
        let fk = model.kinematic().forward_kinematics(&q).expect("fk");
        let contacts: Vec<ContactPoint> = (0..2)
            .flat_map(|s| sole_local.iter().map(move |pt| (s, *pt)))
            .map(|(s, pt)| {
                let tr = fk.transforms()[g1.feet[s].1];
                let pw = tr.translation + tr.rotation * pt;
                ContactPoint {
                    link: g1.feet[s].0,
                    point_local_m: pt,
                    normal_world: Vec3::new(0.0, 0.0, 1.0),
                    gap_m: pw.z,
                    friction_coefficient: 0.8,
                }
            })
            .collect();
        // support: the walker's stance foot once it touches; until then the
        // previous support foot keeps carrying the robot
        let touching = [0, 1].map(|s| contacts[4 * s..4 * s + 4].iter().any(|c| c.gap_m < 0.003));
        let mut stand = if out.double_support {
            [true, true]
        } else if out.stance_left {
            [true, false]
        } else {
            [false, true]
        };
        if !out.double_support {
            let st = if out.stance_left { 0 } else { 1 };
            if !touching[st] && touching[1 - st] && out.swing.2 < 0.3 {
                stand = [st == 1, st == 0];
            }
        }
        // a late foot stays in swing (and keeps descending) until it touches
        let swing_side = if stand[0] && stand[1] {
            None
        } else if stand[0] {
            Some(1)
        } else {
            Some(0)
        };
        // ---- swing leg: inverse kinematics along a lifted cosine path
        if swing_side != last_swing {
            q_ref = q.clone();
            last_swing = swing_side;
        }
        let mut desired = q[6..].to_vec();
        if let Some(s) = swing_side {
            let (l0, tg, u) = out.swing;
            let u = (u / 0.85).min(1.0);
            let sm = 0.5 - 0.5 * (std::f64::consts::PI * u).cos();
            let x = l0[0] + (tg[0] - l0[0]) * sm - FOOT_CENTER_X_M;
            let y = l0[1] + (tg[1] - l0[1]) * sm;
            let z = ANKLE_HEIGHT_M - 0.004 * u + 0.06 * (std::f64::consts::PI * u).sin();
            q_ref[..6].copy_from_slice(&q[..6]);
            g1.leg_ik(&mut q_ref, s, Vector3::new(x, y, z), 6);
            for &d in &g1.leg_dofs[s] {
                desired[d] = q_ref[6 + d];
            }
        }
        // ---- whole-body control
        let rot = G1::base_rotation(&q);
        let e_body = rot.inverse() * rot.inverse().scaled_axis();
        let alpha = e_body * 200.0 - Vector3::new(qd[3], qd[4], qd[5]) * 28.0;
        let mut tau = vec![0.0; nv];
        if fell.is_none() {
            let soles: Vec<SolePoint> = (0..2)
                .filter(|&s| stand[s])
                .flat_map(|s| (0..4).map(move |i| (s, i)))
                .filter(|&(s, i)| contacts[4 * s + i].gap_m < 0.003)
                .map(|(s, i)| SolePoint {
                    link: g1.feet[s].0,
                    index: g1.feet[s].1,
                    local: sole_local[i],
                })
                .collect();
            let qdd_j: Vec<f64> = (0..na)
                .map(|i| 400.0 * (desired[i] - q[6 + i]) - 40.0 * qd[6 + i])
                .collect();
            let mut w_j = vec![1e-3; na];
            if let Some(s) = swing_side {
                for &d in &g1.leg_dofs[s] {
                    w_j[d] = 10.0;
                }
            }
            match backend {
                WbcBackend::ForceQp => {
                    let sol = wbc.solve(model, &q, &qd, &soles, crate::na(com), acc, alpha, &qdd_j, &w_j);
                    tau[6..].copy_from_slice(&sol.tau);
                }
                WbcBackend::Rne => {
                    let contacts_w: Vec<rne_wbc::ContactPoint> = soles
                        .iter()
                        .map(|s| rne_wbc::ContactPoint::with_normal(s.link, s.local, 0.6, Vec3::new(0.0, 0.0, 1.0)))
                        .collect();
                    let com_task = rne_wbc::ComTask {
                        desired_position_m: com,
                        desired_velocity_m_s: Vec3::new(cd[0], cd[1], cd[2]),
                        desired_acceleration_m_s2: crate::rv(acc),
                        position_gain_s_inv2: 0.0,
                        velocity_gain_s_inv: 0.0,
                    };
                    let attitude = rne_wbc::BaseAttitudeTask {
                        desired_angular_acceleration_rad_s2: crate::rv(alpha),
                    };
                    let posture = rne_wbc::PostureTask {
                        desired_joint_positions: desired.clone(),
                        desired_joint_velocities: None,
                        desired_joint_accelerations: None,
                        position_gain_s_inv2: 400.0,
                        velocity_gain_s_inv: 40.0,
                    };
                    if let Ok(sol) = rne_controller.solve(
                        model,
                        &q,
                        &qd,
                        &contacts_w,
                        Some(&com_task),
                        Some(&attitude),
                        Some(&posture),
                    ) {
                        tau[6..].copy_from_slice(&sol.joint_torque_nm);
                    }
                }
            }
        }
        let pushing = push && (PUSH.0..PUSH.0 + 0.1).contains(&t);
        if pushing {
            let fb = rot.inverse() * Vector3::new(g1.mass_kg * PUSH.1[0] / 0.1, g1.mass_kg * PUSH.1[1] / 0.1, 0.0);
            tau[0] += fb.x;
            tau[1] += fb.y;
            tau[2] += fb.z;
        }
        // ---- RNE hard-contact step (effort limits enforced)
        let pd = JointPdControl {
            position_gains: vec![0.0; na],
            velocity_gains: vec![0.0; na],
            target_positions: q[6..].to_vec(),
            target_velocities: vec![0.0; na],
            effort_limits: g1.effort_nm.clone(),
            torque_bounds: Vec::new(),
        };
        if fell.is_none() {
            let st = contact_step(model, &q, &qd, &tau, &contacts, Some(&pd), &step_cfg, warm.as_deref())
                .expect("contact step");
            warm = Some(
                st.contacts
                    .iter()
                    .map(|c| [c.impulse_world_n_s.x, c.impulse_world_n_s.y, c.impulse_world_n_s.z])
                    .collect(),
            );
            q = st.q;
            qd = st.qd;
            let (roll, pitch, _) = G1::base_rotation(&q).euler_angles();
            if q[2] < 0.45 || roll.abs() > 0.7 || pitch.abs() > 0.7 {
                fell = Some(t);
            }
        }
        if t > START_S + 1.0 {
            ev.push(cd[0] - p.step_length_m / p.step_s);
        }
        if k % 10 == 0 {
            let fk = model.kinematic().forward_kinematics(&q).expect("fk");
            let up = G1::base_rotation(&q) * Vector3::new(0.0, 0.0, 0.45);
            frames.push(G1Frame {
                t,
                torso_top: [q[0] + up.x, q[1] + up.y, q[2] + up.z],
                points: g1
                    .draw_links
                    .iter()
                    .map(|&i| {
                        let v = fk.transforms()[i].translation;
                        [v.x, v.y, v.z]
                    })
                    .collect(),
                com: [com.x, com.y, com.z],
                zmp,
                dcm: [com.x + cd[0] / w, com.y + cd[1] / w],
                steps: walker.steps.clone(),
                next: out.swing.1,
                pushing,
                fallen: fell.is_some(),
            });
        }
    }
    let last = frames.last().unwrap();
    G1Run {
        name: strategy.name().into(),
        fell_at_s: fell,
        metrics: vec![
            ("fell".into(), if fell.is_some() { 1.0 } else { 0.0 }),
            ("fall_time_s".into(), fell.unwrap_or(f64::NAN)),
            ("rms_speed_err_mps".into(), robo_control::metrics::rms(&ev)),
            ("distance_m".into(), last.com[0]),
            ("lateral_drift_m".into(), last.com[1]),
        ],
        frames,
    }
}
