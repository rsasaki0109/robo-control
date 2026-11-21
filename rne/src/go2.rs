//! Unitree Go2 trotting on RNE with robo-control force controllers.

use crate::{na, rv};
use nalgebra::{Rotation3, UnitQuaternion, Vector3};
use rne_locomotion::{JointCommand, Quadruped, QuadrupedTerrainSim, TerrainSimConfig};
use rne_math::Vec3;
use rne_physics::FractalTerrain;
use robo_control::legged3d::{raibert_foothold, ContactPlan, ForceController, SrbCommand, SrbParams, SrbState, Trot};
use std::sync::Arc;

/// Settings of a Go2 run.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Go2Config {
    /// duration [s]
    pub duration_s: f64,
    /// trot period [s]
    pub period_s: f64,
    /// standing height of the CoM [m]
    pub height_m: f64,
    /// swing apex [m]
    pub swing_height_m: f64,
    /// controller period [s]
    pub control_dt_s: f64,
    /// enable the pushes
    pub pushes: bool,
    /// peak height of seeded fractal terrain [m] (0 = flat floor)
    pub terrain_amplitude_m: f64,
    /// height quantisation of the terrain samples [m] (terraced terrain; 0 = smooth)
    pub terrain_step_m: f64,
}

impl Default for Go2Config {
    fn default() -> Self {
        Self {
            duration_s: 8.0,
            period_s: 0.36,
            height_m: 0.30,
            swing_height_m: 0.08,
            control_dt_s: 0.01,
            pushes: true,
            terrain_amplitude_m: 0.0,
            terrain_step_m: 0.0,
        }
    }
}

/// Speed command: stand 1 s, 0.5 m/s, then 1.0 m/s from 4.5 s.
pub fn speed_command(t: f64) -> [f64; 2] {
    let vx = if t < 1.0 {
        0.0
    } else if t < 4.5 {
        0.5 * ((t - 1.0) / 0.5).min(1.0)
    } else {
        0.5 + 0.5 * ((t - 4.5) / 0.5).min(1.0)
    };
    [vx, 0.0]
}

/// Pushes on the base (locomotion frame): backward at 3 s, sideways at 6 s.
pub fn push_force(t: f64) -> Vec3 {
    if (3.0..3.15).contains(&t) {
        Vec3::new(-120.0, 0.0, 0.0)
    } else if (6.0..6.15).contains(&t) {
        Vec3::new(0.0, 100.0, 0.0)
    } else {
        Vec3::ZERO
    }
}

/// Recorded frame (locomotion frame, z up).
#[derive(Debug, Clone)]
pub struct Go2Frame {
    /// time [s]
    pub t: f64,
    /// base position
    pub base_m: [f64; 3],
    /// base roll, pitch, yaw [rad]
    pub rpy_rad: [f64; 3],
    /// base velocity
    pub velocity_mps: [f64; 3],
    /// commanded forward speed
    pub v_cmd_mps: f64,
    /// per leg: hip, knee, foot positions
    pub legs: [[[f64; 3]; 3]; 4],
    /// commanded stance forces (world)
    pub forces_n: [[f64; 3]; 4],
    /// stance flags
    pub stance: [bool; 4],
    /// push on the base
    pub push_n: [f64; 3],
    /// ground height along x around the robot, at the base's y: `(x, z)`
    pub ground: Vec<[f64; 2]>,
}

/// Result of one run.
#[derive(Debug, Clone)]
pub struct Go2Run {
    /// controller name
    pub name: String,
    /// frames every 10 ms
    pub frames: Vec<Go2Frame>,
    /// fall time
    pub fell_at_s: Option<f64>,
    /// named metrics
    pub metrics: Vec<(String, f64)>,
}

/// Single-rigid-body parameters of the Go2 (mass from the model).
pub fn go2_srb(quadruped: &Quadruped) -> SrbParams {
    SrbParams {
        mass_kg: quadruped.mass_kg(),
        inertia_kgm2: [0.12, 0.30, 0.32],
        mu: 0.6,
        fz_max_n: 220.0,
    }
}

fn rotation(q: rne_math::Quat) -> Rotation3<f64> {
    UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(q.w, q.x, q.y, q.z)).to_rotation_matrix()
}

/// Runs the Go2 under `controller`.
pub fn run(controller: &mut dyn ForceController, cfg: &Go2Config) -> Go2Run {
    let quadruped = Arc::new(Quadruped::go2().expect("Go2 model"));
    let floor = if cfg.terrain_amplitude_m > 0.0 {
        FractalTerrain {
            size_x_m: 16.0,
            size_z_m: 16.0,
            columns: 161,
            rows: 161,
            frequency_per_m: 1.5,
            amplitude_m: cfg.terrain_amplitude_m,
            octaves: 3,
            step_m: cfg.terrain_step_m,
            ..FractalTerrain::default()
        }
        .height_field(7)
        .expect("terrain")
    } else {
        FractalTerrain {
            size_x_m: 60.0,
            size_z_m: 60.0,
            columns: 5,
            rows: 5,
            amplitude_m: 0.0,
            ..FractalTerrain::default()
        }
        .height_field(0)
        .expect("flat floor")
    };
    let terrain = floor.clone();
    let ground = move |x: f64, y: f64| -> f64 {
        let h = rne_locomotion::go2_learning::terrain_height_m(&terrain, x, y);
        if h.is_finite() {
            h
        } else {
            0.0
        }
    };
    let sim_cfg = TerrainSimConfig {
        step_time_s: 0.002,
        position_gain_nm_rad: 40.0,
        velocity_gain_nm_s_rad: 1.5,
        friction_coefficient: 0.8,
        ..TerrainSimConfig::default()
    };
    let mut sim = QuadrupedTerrainSim::new(quadruped.clone(), floor, sim_cfg).expect("sim");
    let gait = Trot { period_s: cfg.period_s };
    let legs = *quadruped.legs();
    let model = quadruped.model();
    let names = [
        ["FL_hip", "FL_calf"],
        ["FR_hip", "FR_calf"],
        ["RL_hip", "RL_calf"],
        ["RR_hip", "RR_calf"],
    ];
    let link_index = |n: &str| {
        let e = model.kinematic().link_entity_by_name(n).expect("link");
        model.kinematic().link_index(e).expect("index")
    };
    let joint_links: Vec<[usize; 2]> = names.iter().map(|[h, k]| [link_index(h), link_index(k)]).collect();
    let foot_r = quadruped.foot_radius_m();
    let dt = 0.002;
    let steps = (cfg.duration_s / dt).round() as usize;
    let ctrl_every = (cfg.control_dt_s / dt).round().max(1.0) as usize;
    let mut liftoff = [Vector3::zeros(); 4];
    let mut touchdown = [Vector3::zeros(); 4];
    let mut prev_stance = [true; 4];
    let mut forces = [Vector3::zeros(); 4];
    let mut frames = Vec::new();
    let mut fell = None;
    let (mut ev, mut eh, mut eatt) = (Vec::new(), Vec::new(), Vec::new());
    let com_off = na(quadruped.stand_com_m());
    for k in 0..steps {
        let t = k as f64 * dt;
        let walking = t >= 1.0;
        let gt = t - 1.0;
        let stance = if walking { gait.stance(gt) } else { [true; 4] };
        let bs = sim.base_state();
        let r = rotation(bs.rotation);
        let base = na(bs.position_m);
        let com = base + r * com_off;
        let state = SrbState {
            position_m: com,
            velocity_mps: na(bs.linear_velocity_m_s),
            rotation: r,
            omega_radps: na(bs.angular_velocity_rad_s),
        };
        let feet_c: Vec<Vector3<f64>> = quadruped
            .foot_centers_m(sim.q())
            .expect("feet")
            .into_iter()
            .map(na)
            .collect();
        let feet: [Vector3<f64>; 4] = std::array::from_fn(|i| feet_c[i] - Vector3::new(0.0, 0.0, foot_r));
        let v_cmd = speed_command(t);
        let cmd = SrbCommand {
            velocity_mps: v_cmd,
            height_m: cfg.height_m,
            yaw_rad: 0.0,
            yaw_rate_radps: 0.0,
        };
        // swing planning
        let left = if walking { gait.phase_left_s(gt) } else { 1.0 };
        for i in 0..4 {
            if !stance[i] && prev_stance[i] {
                liftoff[i] = feet[i];
            }
            if !stance[i] {
                let hip_now = base + r * na(legs[i].hip_offset_m) + Vector3::new(0.0, legs[i].thigh_offset_y_m, 0.0);
                let hip_td = hip_now + state.velocity_mps * left;
                let mut td = raibert_foothold(hip_td, state.velocity_mps, v_cmd, cfg.period_s / 2.0, 0.06, 0.0);
                td.z = ground(td.x, td.y);
                touchdown[i] = td;
            }
        }
        prev_stance = stance;
        if fell.is_none() && k % ctrl_every == 0 {
            let nh = controller.horizon();
            let hdt = controller.horizon_dt_s();
            let plan = ContactPlan {
                feet_m: feet,
                touchdown_m: std::array::from_fn(|i| if stance[i] { feet[i] } else { touchdown[i] }),
                stance: (0..nh.max(1))
                    .map(|j| {
                        if !walking {
                            if t + j as f64 * hdt < 1.0 {
                                [true; 4]
                            } else {
                                gait.stance(t + j as f64 * hdt - 1.0)
                            }
                        } else {
                            gait.stance(gt + j as f64 * hdt)
                        }
                    })
                    .collect(),
                phase_left_s: [if walking { left } else { 1.0 - t }; 4],
            };
            forces = controller.forces(&state, &cmd, &plan);
        }
        // joint command
        let jq = sim.joint_positions_rad().to_vec();
        let jqd = sim.joint_velocities_rad_s().to_vec();
        let mut pos = jq.clone();
        let mut vel = jqd.clone();
        let mut ff = vec![0.0; jq.len()];
        let swing_s = if walking { gait.swing_progress(gt) } else { 0.0 };
        for i in 0..4 {
            let jn = legs[i].joints;
            let angles = [jq[jn[0]], jq[jn[1]], jq[jn[2]]];
            if stance[i] {
                let fb = r.inverse() * forces[i];
                let tau = quadruped.leg_force_torques(i, angles, rv(fb));
                for a in 0..3 {
                    ff[jn[a]] = tau[a];
                }
            } else {
                let u = 0.5 - 0.5 * (std::f64::consts::PI * swing_s).cos();
                let mut target = liftoff[i] + (touchdown[i] - liftoff[i]) * u;
                target.z = liftoff[i].z
                    + (touchdown[i].z - liftoff[i].z) * u
                    + foot_r
                    + cfg.swing_height_m * (std::f64::consts::PI * swing_s).sin();
                let foot_base = r.inverse() * (target - base);
                let ang = quadruped.leg_inverse_kinematics(i, rv(foot_base));
                for a in 0..3 {
                    pos[jn[a]] = ang[a];
                    vel[jn[a]] = 0.0;
                }
            }
        }
        let push = if cfg.pushes { push_force(t) } else { Vec3::ZERO };
        if fell.is_none() {
            sim.step(
                &JointCommand {
                    positions_rad: pos,
                    velocities_rad_s: vel,
                    feedforward_nm: ff,
                },
                push,
            )
            .expect("step");
            let (roll, pitch, _) = r.euler_angles();
            if base.z - ground(base.x, base.y) < 0.15 || roll.abs() > 0.8 || pitch.abs() > 0.8 {
                fell = Some(t);
            }
        }
        if t >= 1.5 {
            ev.push(state.velocity_mps.x - v_cmd[0]);
            eh.push(com.z - cmd.height_m);
            let (roll, pitch, _) = r.euler_angles();
            eatt.push(roll.hypot(pitch));
        }
        if k % 5 == 0 {
            let kin = model.kinematic().forward_kinematics(sim.q()).expect("fk");
            let tr = kin.transforms();
            let (roll, pitch, yaw) = r.euler_angles();
            frames.push(Go2Frame {
                t,
                base_m: [base.x, base.y, base.z],
                rpy_rad: [roll, pitch, yaw],
                velocity_mps: [state.velocity_mps.x, state.velocity_mps.y, state.velocity_mps.z],
                v_cmd_mps: v_cmd[0],
                legs: std::array::from_fn(|i| {
                    let h = tr[joint_links[i][0]].translation;
                    let kn = tr[joint_links[i][1]].translation;
                    let f = feet[i];
                    [[h.x, h.y, h.z], [kn.x, kn.y, kn.z], [f.x, f.y, f.z]]
                }),
                forces_n: std::array::from_fn(|i| {
                    if stance[i] {
                        [forces[i].x, forces[i].y, forces[i].z]
                    } else {
                        [0.0; 3]
                    }
                }),
                stance,
                push_n: [push.x, push.y, push.z],
                ground: (0..=40)
                    .map(|j| {
                        let x = base.x - 1.0 + 0.05 * j as f64;
                        [x, ground(x, base.y)]
                    })
                    .collect(),
            });
        }
    }
    let rms = robo_control::metrics::rms;
    let last = frames.last().unwrap();
    Go2Run {
        name: controller.name().to_string(),
        frames: frames.clone(),
        fell_at_s: fell,
        metrics: vec![
            ("fell".into(), if fell.is_some() { 1.0 } else { 0.0 }),
            ("rms_speed_err_mps".into(), rms(&ev)),
            ("rms_height_err_m".into(), rms(&eh)),
            ("rms_tilt_rad".into(), rms(&eatt)),
            ("lateral_drift_m".into(), last.base_m[1]),
            ("distance_m".into(), last.base_m[0]),
        ],
    }
}
