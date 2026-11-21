//! Backend-neutral 3D single-rigid-body (SRB) force controllers for legged
//! robots, plus trot scheduling and Raibert foot placement.
//!
//! A controller looks at the body as one rigid body (mass, body inertia) with
//! point feet and returns the ground reaction force each stance foot should
//! push with, in the world frame (z up). Mapping forces to joint torques
//! (`tau = -J^T f`) and swinging the free legs is the robot integration's job;
//! see the `robo-control-rne` crate for the Unitree Go2 on RNE.

use crate::mpc::{box_qp, box_qp_newton_start};
use nalgebra::{DMatrix, DVector, Matrix3, Rotation3, Vector3};

const G: f64 = 9.81;

/// Single-rigid-body parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SrbParams {
    /// total mass [kg]
    pub mass_kg: f64,
    /// body-frame inertia diagonal [kg m^2]
    pub inertia_kgm2: [f64; 3],
    /// friction coefficient used by the controller
    pub mu: f64,
    /// max normal force per foot [N]
    pub fz_max_n: f64,
}

/// Measured body state in the world frame (z up).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SrbState {
    /// centre of mass [m]
    pub position_m: Vector3<f64>,
    /// linear velocity [m/s]
    pub velocity_mps: Vector3<f64>,
    /// body-to-world rotation
    pub rotation: Rotation3<f64>,
    /// angular velocity in the world frame [rad/s]
    pub omega_radps: Vector3<f64>,
}

/// Body command.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SrbCommand {
    /// desired world velocity (x, y) [m/s]
    pub velocity_mps: [f64; 2],
    /// desired CoM height [m]
    pub height_m: f64,
    /// desired heading [rad]
    pub yaw_rad: f64,
    /// desired yaw rate [rad/s]
    pub yaw_rate_radps: f64,
}

/// Contact information for the controller.
#[derive(Debug, Clone)]
pub struct ContactPlan {
    /// current foot positions in the world [m]
    pub feet_m: [Vector3<f64>; 4],
    /// planned touchdown positions of the swing feet [m]
    pub touchdown_m: [Vector3<f64>; 4],
    /// stance flags at `t + k * dt` for the controller's horizon, `stance[0]` is now
    pub stance: Vec<[bool; 4]>,
    /// time until each leg changes phase [s]
    pub phase_left_s: [f64; 4],
}

/// Ground reaction force controller.
pub trait ForceController {
    /// Short name.
    fn name(&self) -> &str;
    /// Horizon length (number of future stance flags wanted in [`ContactPlan::stance`]).
    fn horizon(&self) -> usize {
        1
    }
    /// Horizon step [s].
    fn horizon_dt_s(&self) -> f64 {
        0.0
    }
    /// World-frame foot forces (zero for swing feet).
    fn forces(&mut self, s: &SrbState, cmd: &SrbCommand, plan: &ContactPlan) -> [Vector3<f64>; 4];
}

fn skew(r: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -r.z, r.y, r.z, 0.0, -r.x, -r.y, r.x, 0.0)
}

/// Rotation vector taking the current attitude to the desired one (world frame).
fn attitude_error(s: &SrbState, yaw_rad: f64) -> Vector3<f64> {
    let rd = Rotation3::from_euler_angles(0.0, 0.0, yaw_rad);
    (rd * s.rotation.inverse()).scaled_axis()
}

fn bounds(p: &SrbParams, stance: &[bool; 4], lo: &mut DVector<f64>, hi: &mut DVector<f64>, off: usize) {
    let n = stance.iter().filter(|&&s| s).count().max(1) as f64;
    let ft = p.mu * p.mass_kg * G / n;
    for i in 0..4 {
        if stance[i] {
            for a in 0..2 {
                lo[off + 3 * i + a] = -ft;
                hi[off + 3 * i + a] = ft;
            }
            lo[off + 3 * i + 2] = 2.0;
            hi[off + 3 * i + 2] = p.fz_max_n;
        }
    }
}

/// Projects a force into the friction cone (keeps the normal part, scales the tangential part).
pub fn project_to_cone(f: Vector3<f64>, mu: f64) -> Vector3<f64> {
    let fz = f.z.max(0.0);
    let t = (f.x * f.x + f.y * f.y).sqrt();
    if t <= mu * fz || t == 0.0 {
        Vector3::new(f.x, f.y, fz)
    } else {
        let k = mu * fz / t;
        Vector3::new(f.x * k, f.y * k, fz)
    }
}

/// PD on the body (velocity, height, attitude) giving a desired wrench, split
/// over the stance feet by a box-constrained least-squares QP.
#[derive(Debug, Clone)]
pub struct PdForceQp {
    /// parameters
    pub p: SrbParams,
    /// horizontal velocity gain [1/s]
    pub kv: f64,
    /// height stiffness and damping
    pub kz: (f64, f64),
    /// attitude stiffness and damping
    pub katt: (f64, f64),
}

impl PdForceQp {
    /// Default gains.
    pub fn new(p: SrbParams) -> Self {
        Self {
            p,
            kv: 4.0,
            kz: (120.0, 18.0),
            katt: (150.0, 18.0),
        }
    }
}

impl ForceController for PdForceQp {
    fn name(&self) -> &str {
        "PD + force QP"
    }
    fn forces(&mut self, s: &SrbState, cmd: &SrbCommand, plan: &ContactPlan) -> [Vector3<f64>; 4] {
        let p = &self.p;
        let acc = Vector3::new(
            self.kv * (cmd.velocity_mps[0] - s.velocity_mps.x),
            self.kv * (cmd.velocity_mps[1] - s.velocity_mps.y),
            self.kz.0 * (cmd.height_m - s.position_m.z) - self.kz.1 * s.velocity_mps.z,
        );
        let wd = Vector3::new(0.0, 0.0, cmd.yaw_rate_radps);
        let alpha = attitude_error(s, cmd.yaw_rad) * self.katt.0 + (wd - s.omega_radps) * self.katt.1;
        let ib = Matrix3::from_diagonal(&Vector3::from(p.inertia_kgm2));
        let iw = s.rotation.matrix() * ib * s.rotation.matrix().transpose();
        let force = (acc + Vector3::new(0.0, 0.0, G)) * p.mass_kg;
        let torque = iw * alpha;
        let mut a = DMatrix::zeros(6, 12);
        for i in 0..4 {
            let r = plan.feet_m[i] - s.position_m;
            a.view_mut((0, 3 * i), (3, 3)).copy_from(&Matrix3::identity());
            a.view_mut((3, 3 * i), (3, 3)).copy_from(&skew(&r));
        }
        let w = DVector::from_vec(vec![force.x, force.y, force.z, torque.x, torque.y, torque.z]);
        let sw = DMatrix::from_diagonal(&DVector::from_vec(vec![1.0, 1.0, 1.0, 20.0, 20.0, 10.0]));
        let h = a.transpose() * &sw * &a + DMatrix::identity(12, 12) * 1e-3;
        let g = -(a.transpose() * &sw * w);
        let (mut lo, mut hi) = (DVector::zeros(12), DVector::zeros(12));
        bounds(p, &plan.stance[0], &mut lo, &mut hi, 0);
        let f = box_qp(&h, &g, &lo, &hi, DVector::zeros(12), 300);
        std::array::from_fn(|i| project_to_cone(Vector3::new(f[3 * i], f[3 * i + 1], f[3 * i + 2]), p.mu))
    }
}

/// Convex MPC (Di Carlo et al. 2018): SRB dynamics linearised about the
/// commanded yaw, contact schedule and footholds known over the horizon,
/// condensed into a box-constrained QP over all foot forces.
#[derive(Debug, Clone)]
pub struct ConvexMpc3d {
    /// parameters
    pub p: SrbParams,
    /// horizon [steps]
    pub n: usize,
    /// step [s]
    pub dt_s: f64,
    /// weights on `[roll, pitch, yaw, x, y, z, wx, wy, wz, vx, vy, vz]`
    pub q: [f64; 12],
    /// force weight
    pub r: f64,
}

impl ConvexMpc3d {
    /// Default: 10 steps of 30 ms.
    pub fn new(p: SrbParams) -> Self {
        Self {
            p,
            n: 10,
            dt_s: 0.03,
            q: [25.0, 25.0, 10.0, 1.0, 1.0, 60.0, 0.5, 0.5, 0.5, 4.0, 4.0, 1.0],
            r: 2e-5,
        }
    }
}

impl ForceController for ConvexMpc3d {
    fn name(&self) -> &str {
        "convex MPC"
    }
    fn horizon(&self) -> usize {
        self.n
    }
    fn horizon_dt_s(&self) -> f64 {
        self.dt_s
    }
    fn forces(&mut self, s: &SrbState, cmd: &SrbCommand, plan: &ContactPlan) -> [Vector3<f64>; 4] {
        let (n, dt, p) = (self.n, self.dt_s, self.p);
        const NX: usize = 13; // [theta(3), p(3), w(3), v(3), g]
        let nu = 12;
        let (roll, pitch, yaw) = s.rotation.euler_angles();
        let x0 = DVector::from_vec(vec![
            roll,
            pitch,
            yaw,
            s.position_m.x,
            s.position_m.y,
            s.position_m.z,
            s.omega_radps.x,
            s.omega_radps.y,
            s.omega_radps.z,
            s.velocity_mps.x,
            s.velocity_mps.y,
            s.velocity_mps.z,
            -G,
        ]);
        // reference
        let xref = |k: usize| -> DVector<f64> {
            let t = k as f64 * dt;
            let yaw_k = cmd.yaw_rad + cmd.yaw_rate_radps * t;
            let yaw_k = yaw + crate::model::wrap(yaw_k - yaw);
            DVector::from_vec(vec![
                0.0,
                0.0,
                yaw_k,
                s.position_m.x + cmd.velocity_mps[0] * t,
                s.position_m.y + cmd.velocity_mps[1] * t,
                cmd.height_m,
                0.0,
                0.0,
                cmd.yaw_rate_radps,
                cmd.velocity_mps[0],
                cmd.velocity_mps[1],
                0.0,
                -G,
            ])
        };
        let rz = Rotation3::from_euler_angles(0.0, 0.0, yaw);
        let ib = Matrix3::from_diagonal(&Vector3::from(p.inertia_kgm2));
        let iw_inv = (rz.matrix() * ib * rz.matrix().transpose()).try_inverse().unwrap();
        let mut a = DMatrix::<f64>::identity(NX, NX);
        a.view_mut((0, 6), (3, 3)).copy_from(&(rz.matrix().transpose() * dt));
        a.view_mut((3, 9), (3, 3)).copy_from(&(Matrix3::identity() * dt));
        a[(11, 12)] = dt;
        let mut sx = DMatrix::zeros(NX * n, NX);
        let mut su = DMatrix::zeros(NX * n, nu * n);
        let mut phi = DMatrix::<f64>::identity(NX, NX);
        let mut gam = DMatrix::<f64>::zeros(NX, nu * n);
        let mut lo = DVector::zeros(nu * n);
        let mut hi = DVector::zeros(nu * n);
        for k in 0..n {
            let st = plan
                .stance
                .get(k)
                .copied()
                .unwrap_or(plan.stance[plan.stance.len() - 1]);
            let ck = xref(k);
            let com = Vector3::new(ck[3], ck[4], ck[5]);
            let mut b = DMatrix::zeros(NX, nu);
            for i in 0..4 {
                if !st[i] {
                    continue;
                }
                let foot = if plan.stance[0][i] && plan.phase_left_s[i] > k as f64 * dt {
                    plan.feet_m[i]
                } else {
                    plan.touchdown_m[i]
                };
                let r = foot - com;
                b.view_mut((6, 3 * i), (3, 3)).copy_from(&(iw_inv * skew(&r) * dt));
                b.view_mut((9, 3 * i), (3, 3))
                    .copy_from(&(Matrix3::identity() * (dt / p.mass_kg)));
            }
            bounds(&p, &st, &mut lo, &mut hi, nu * k);
            phi = &a * phi;
            gam = &a * gam;
            gam.view_mut((0, nu * k), (NX, nu)).copy_from(&b);
            sx.view_mut((NX * k, 0), (NX, NX)).copy_from(&phi);
            su.view_mut((NX * k, 0), (NX, nu * n)).copy_from(&gam);
        }
        let mut qd = DVector::zeros(NX * n);
        let mut xr = DVector::zeros(NX * n);
        for k in 0..n {
            for i in 0..12 {
                qd[NX * k + i] = self.q[i];
            }
            xr.rows_mut(NX * k, NX).copy_from(&xref(k + 1));
        }
        let qbar = DMatrix::from_diagonal(&qd);
        let sut_q = su.transpose() * &qbar;
        let h = &sut_q * &su + DMatrix::identity(nu * n, nu * n) * self.r;
        let g = &sut_q * (&sx * &x0 - &xr);
        let z = box_qp_newton_start(&h, &g, &lo, &hi, 250);
        std::array::from_fn(|i| project_to_cone(Vector3::new(z[3 * i], z[3 * i + 1], z[3 * i + 2]), p.mu))
    }
}

/// Diagonal-pair trot schedule: legs ordered FL, FR, RL, RR.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Trot {
    /// stride period [s]
    pub period_s: f64,
}

impl Trot {
    /// Stance flags at time `t` (FL+RR stand in the first half-period).
    pub fn stance(&self, t: f64) -> [bool; 4] {
        let a = (t / (self.period_s / 2.0)).floor() as i64 % 2 == 0;
        [a, !a, !a, a]
    }
    /// Time left in the current half-period.
    pub fn phase_left_s(&self, t: f64) -> f64 {
        let h = self.period_s / 2.0;
        h - t.rem_euclid(h)
    }
    /// Swing progress in `[0, 1)` for a leg in swing at time `t`.
    pub fn swing_progress(&self, t: f64) -> f64 {
        1.0 - self.phase_left_s(t) / (self.period_s / 2.0)
    }
}

/// Raibert touchdown point for a hip that will be at `hip_at_touchdown`:
/// half a stance of travel ahead, plus a velocity-error correction.
pub fn raibert_foothold(
    hip_at_touchdown: Vector3<f64>,
    v: Vector3<f64>,
    v_des: [f64; 2],
    stance_s: f64,
    k_s: f64,
    ground_z: f64,
) -> Vector3<f64> {
    Vector3::new(
        hip_at_touchdown.x + v.x * stance_s / 2.0 + k_s * (v.x - v_des[0]),
        hip_at_touchdown.y + v.y * stance_s / 2.0 + k_s * (v.y - v_des[1]),
        ground_z,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn standing() -> (SrbParams, SrbState, SrbCommand, ContactPlan) {
        let p = SrbParams {
            mass_kg: 15.0,
            inertia_kgm2: [0.1, 0.25, 0.3],
            mu: 0.6,
            fz_max_n: 200.0,
        };
        let s = SrbState {
            position_m: Vector3::new(0.0, 0.0, 0.3),
            velocity_mps: Vector3::zeros(),
            rotation: Rotation3::identity(),
            omega_radps: Vector3::zeros(),
        };
        let c = SrbCommand {
            velocity_mps: [0.0, 0.0],
            height_m: 0.3,
            yaw_rad: 0.0,
            yaw_rate_radps: 0.0,
        };
        let feet = [
            Vector3::new(0.19, 0.12, 0.0),
            Vector3::new(0.19, -0.12, 0.0),
            Vector3::new(-0.19, 0.12, 0.0),
            Vector3::new(-0.19, -0.12, 0.0),
        ];
        let plan = ContactPlan {
            feet_m: feet,
            touchdown_m: feet,
            stance: vec![[true; 4]; 10],
            phase_left_s: [1.0; 4],
        };
        (p, s, c, plan)
    }

    #[test]
    fn both_controllers_carry_the_weight_when_standing() {
        let (p, s, c, plan) = standing();
        for f in [
            PdForceQp::new(p).forces(&s, &c, &plan),
            ConvexMpc3d::new(p).forces(&s, &c, &plan),
        ] {
            let total: Vector3<f64> = f.iter().sum();
            assert!((total.z - p.mass_kg * G).abs() < 0.05 * p.mass_kg * G, "{total:?}");
            assert!(total.x.abs() < 1.0 && total.y.abs() < 1.0);
        }
    }

    #[test]
    fn swing_feet_get_no_force() {
        let (p, s, c, mut plan) = standing();
        plan.stance = vec![[true, false, false, true]; 10];
        for f in [
            PdForceQp::new(p).forces(&s, &c, &plan),
            ConvexMpc3d::new(p).forces(&s, &c, &plan),
        ] {
            assert!(f[1].norm() < 1e-9 && f[2].norm() < 1e-9);
        }
    }
}
