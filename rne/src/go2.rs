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
    /// stairs with this rise per step [m] (0 = none); see [`stairs_height_m`]
    pub stairs_rise_m: f64,
}

/// Stair tread depth [m].
pub const STAIR_TREAD_M: f64 = 0.3;
/// First riser [m].
pub const STAIR_START_M: f64 = 1.5;
/// Number of steps up.
pub const STAIR_STEPS: usize = 5;

/// Stairs along +x: flat until [`STAIR_START_M`], then [`STAIR_STEPS`] steps of
/// `rise_m`, treads [`STAIR_TREAD_M`] deep, then a flat landing.
pub fn stairs_height_m(x: f64, rise_m: f64) -> f64 {
    if x < STAIR_START_M {
        return 0.0;
    }
    let k = (((x - STAIR_START_M) / STAIR_TREAD_M).floor() as usize + 1).min(STAIR_STEPS);
    k as f64 * rise_m
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
            stairs_rise_m: 0.0,
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

/// Measurements of one 2 ms step of a [`Go2Sim`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Go2Tick {
    /// time at the start of the step [s]
    pub t: f64,
    /// CoM [m]
    pub com: Vector3<f64>,
    /// commanded CoM height [m]
    pub height_cmd_m: f64,
    /// base velocity [m/s]
    pub velocity_mps: Vector3<f64>,
    /// roll, pitch [rad]
    pub roll_pitch_rad: [f64; 2],
}

/// One Go2 on RNE driven step by step with a velocity command: the force
/// controller carries the body, swing legs land at Raibert footholds. The robot
/// stands for the first second and then trots.
pub struct Go2Sim<'a> {
    controller: &'a mut dyn ForceController,
    cfg: Go2Config,
    quadruped: Arc<Quadruped>,
    sim: QuadrupedTerrainSim,
    ground: Box<dyn Fn(f64, f64) -> f64>,
    joint_links: Vec<[usize; 2]>,
    k: usize,
    liftoff: [Vector3<f64>; 4],
    touchdown: [Vector3<f64>; 4],
    prev_stance: [bool; 4],
    forces: [Vector3<f64>; 4],
    feet: [Vector3<f64>; 4],
    stance: [bool; 4],
    fell: Option<f64>,
}

impl std::fmt::Debug for Go2Sim<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Go2Sim")
            .field("t", &self.time_s())
            .field("fell", &self.fell)
            .finish()
    }
}

impl<'a> Go2Sim<'a> {
    /// Standing Go2 at `(x, y)` facing +x.
    pub fn new(controller: &'a mut dyn ForceController, cfg: Go2Config, start_xy: [f64; 2]) -> Self {
        let quadruped = Arc::new(Quadruped::go2().expect("Go2 model"));
        let floor = if cfg.stairs_rise_m > 0.0 {
            // 1 cm samples along x so each riser is (nearly) vertical
            let (size, cols, rows) = (16.0, 1601u32, 5u32);
            let heights: Vec<f64> = (0..rows)
                .flat_map(|_| {
                    (0..cols).map(move |c| {
                        stairs_height_m(-size / 2.0 + size * c as f64 / (cols - 1) as f64, cfg.stairs_rise_m)
                    })
                })
                .collect();
            rne_physics::ColliderShape::HeightField {
                nrows: rows,
                ncols: cols,
                heights_m: std::sync::Arc::from(heights),
                scale: Vec3::new(size, 1.0, size),
            }
        } else if cfg.terrain_amplitude_m > 0.0 {
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
        let ground = Box::new(move |x: f64, y: f64| -> f64 {
            let h = rne_locomotion::go2_learning::terrain_height_m(&terrain, x, y);
            if h.is_finite() {
                h
            } else {
                0.0
            }
        });
        let sim_cfg = TerrainSimConfig {
            step_time_s: 0.002,
            position_gain_nm_rad: 40.0,
            velocity_gain_nm_s_rad: 1.5,
            friction_coefficient: 0.8,
            ..TerrainSimConfig::default()
        };
        let mut sim = QuadrupedTerrainSim::new(quadruped.clone(), floor, sim_cfg).expect("sim");
        if start_xy != [0.0, 0.0] {
            sim.reset_standing(start_xy[0], start_xy[1], 0.0).expect("reset");
        }
        let model = quadruped.model();
        let link_index = |n: &str| {
            let e = model.kinematic().link_entity_by_name(n).expect("link");
            model.kinematic().link_index(e).expect("index")
        };
        let joint_links = [
            ["FL_hip", "FL_calf"],
            ["FR_hip", "FR_calf"],
            ["RL_hip", "RL_calf"],
            ["RR_hip", "RR_calf"],
        ]
        .iter()
        .map(|[h, k]| [link_index(h), link_index(k)])
        .collect();
        Self {
            controller,
            cfg,
            quadruped,
            sim,
            ground,
            joint_links,
            k: 0,
            liftoff: [Vector3::zeros(); 4],
            touchdown: [Vector3::zeros(); 4],
            prev_stance: [true; 4],
            forces: [Vector3::zeros(); 4],
            feet: [Vector3::zeros(); 4],
            stance: [true; 4],
            fell: None,
        }
    }

    /// Simulation time [s].
    pub fn time_s(&self) -> f64 {
        self.k as f64 * 0.002
    }

    /// Time of the fall, if any.
    pub fn fell_at_s(&self) -> Option<f64> {
        self.fell
    }

    /// Base position and velocity in the world (z up).
    pub fn base(&self) -> (Vector3<f64>, Vector3<f64>) {
        let bs = self.sim.base_state();
        (na(bs.position_m), na(bs.linear_velocity_m_s))
    }

    /// Base heading (yaw) [rad].
    pub fn yaw(&self) -> f64 {
        rotation(self.sim.base_state().rotation).euler_angles().2
    }

    /// Controller name.
    pub fn controller_name(&self) -> String {
        self.controller.name().to_string()
    }

    /// Advances 2 ms with world velocity command `v_cmd` and a push on the base.
    pub fn step(&mut self, v_cmd: [f64; 2], push: Vec3) -> Go2Tick {
        self.step_heading(v_cmd, 0.0, 0.0, push)
    }

    /// [`Self::step`] with a heading `yaw_rad` and its rate to track.
    pub fn step_heading(&mut self, v_cmd: [f64; 2], yaw_rad: f64, yaw_rate_radps: f64, push: Vec3) -> Go2Tick {
        let cfg = self.cfg;
        let dt = 0.002;
        let ctrl_every = (cfg.control_dt_s / dt).round().max(1.0) as usize;
        let gait = Trot { period_s: cfg.period_s };
        let legs = *self.quadruped.legs();
        let foot_r = self.quadruped.foot_radius_m();
        let com_off = na(self.quadruped.stand_com_m());
        let t = self.time_s();
        let walking = t >= 1.0;
        let gt = t - 1.0;
        let stance = if walking { gait.stance(gt) } else { [true; 4] };
        let bs = self.sim.base_state();
        let r = rotation(bs.rotation);
        let base = na(bs.position_m);
        let com = base + r * com_off;
        let state = SrbState {
            position_m: com,
            velocity_mps: na(bs.linear_velocity_m_s),
            rotation: r,
            omega_radps: na(bs.angular_velocity_rad_s),
        };
        let feet_c: Vec<Vector3<f64>> = self
            .quadruped
            .foot_centers_m(self.sim.q())
            .expect("feet")
            .into_iter()
            .map(na)
            .collect();
        let feet: [Vector3<f64>; 4] = std::array::from_fn(|i| feet_c[i] - Vector3::new(0.0, 0.0, foot_r));
        let ground = &self.ground;
        // ride at the nominal height above the ground under the feet
        let ground_mean = feet.iter().map(|f| ground(f.x, f.y)).sum::<f64>() / 4.0;
        let cmd = SrbCommand {
            velocity_mps: v_cmd,
            height_m: ground_mean + cfg.height_m,
            yaw_rad,
            yaw_rate_radps,
        };
        // swing planning
        let left = if walking { gait.phase_left_s(gt) } else { 1.0 };
        for i in 0..4 {
            if !stance[i] && self.prev_stance[i] {
                self.liftoff[i] = feet[i];
            }
            if !stance[i] {
                let hip_now = base + r * (na(legs[i].hip_offset_m) + Vector3::new(0.0, legs[i].thigh_offset_y_m, 0.0));
                let hip_td = hip_now + state.velocity_mps * left;
                let mut td = raibert_foothold(hip_td, state.velocity_mps, v_cmd, cfg.period_s / 2.0, 0.06, 0.0);
                td.z = ground(td.x, td.y);
                self.touchdown[i] = td;
            }
        }
        self.prev_stance = stance;
        if self.fell.is_none() && self.k.is_multiple_of(ctrl_every) {
            let nh = self.controller.horizon();
            let hdt = self.controller.horizon_dt_s();
            let plan = ContactPlan {
                feet_m: feet,
                touchdown_m: std::array::from_fn(|i| if stance[i] { feet[i] } else { self.touchdown[i] }),
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
            self.forces = self.controller.forces(&state, &cmd, &plan);
        }
        // joint command
        let jq = self.sim.joint_positions_rad().to_vec();
        let jqd = self.sim.joint_velocities_rad_s().to_vec();
        let mut pos = jq.clone();
        let mut vel = jqd.clone();
        let mut ff = vec![0.0; jq.len()];
        let swing_s = if walking { gait.swing_progress(gt) } else { 0.0 };
        for i in 0..4 {
            let jn = legs[i].joints;
            let angles = [jq[jn[0]], jq[jn[1]], jq[jn[2]]];
            if stance[i] {
                let fb = r.inverse() * self.forces[i];
                let tau = self.quadruped.leg_force_torques(i, angles, rv(fb));
                for a in 0..3 {
                    ff[jn[a]] = tau[a];
                }
            } else {
                let u = 0.5 - 0.5 * (std::f64::consts::PI * swing_s).cos();
                let mut target = self.liftoff[i] + (self.touchdown[i] - self.liftoff[i]) * u;
                target.z = self.liftoff[i].z
                    + (self.touchdown[i].z - self.liftoff[i].z) * u
                    + foot_r
                    + cfg.swing_height_m * (std::f64::consts::PI * swing_s).sin();
                let foot_base = r.inverse() * (target - base);
                let ang = self.quadruped.leg_inverse_kinematics(i, rv(foot_base));
                for a in 0..3 {
                    pos[jn[a]] = ang[a];
                    vel[jn[a]] = 0.0;
                }
            }
        }
        if self.fell.is_none() {
            self.sim
                .step(
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
                self.fell = Some(t);
            }
        }
        self.feet = feet;
        self.stance = stance;
        self.k += 1;
        let (roll, pitch, _) = r.euler_angles();
        Go2Tick {
            t,
            com,
            height_cmd_m: cmd.height_m,
            velocity_mps: state.velocity_mps,
            roll_pitch_rad: [roll, pitch],
        }
    }

    /// Drawing frame of the state at the start of the last step.
    pub fn frame(&self, tick: &Go2Tick, v_cmd_x: f64, push: Vec3) -> Go2Frame {
        let model = self.quadruped.model();
        let kin = model.kinematic().forward_kinematics(self.sim.q()).expect("fk");
        let tr = kin.transforms();
        let bs = self.sim.base_state();
        let base = na(bs.position_m);
        let (roll, pitch, yaw) = rotation(bs.rotation).euler_angles();
        let ground = &self.ground;
        Go2Frame {
            t: tick.t,
            base_m: [base.x, base.y, base.z],
            rpy_rad: [roll, pitch, yaw],
            velocity_mps: [tick.velocity_mps.x, tick.velocity_mps.y, tick.velocity_mps.z],
            v_cmd_mps: v_cmd_x,
            legs: std::array::from_fn(|i| {
                let h = tr[self.joint_links[i][0]].translation;
                let kn = tr[self.joint_links[i][1]].translation;
                let f = self.feet[i];
                [[h.x, h.y, h.z], [kn.x, kn.y, kn.z], [f.x, f.y, f.z]]
            }),
            forces_n: std::array::from_fn(|i| {
                if self.stance[i] {
                    [self.forces[i].x, self.forces[i].y, self.forces[i].z]
                } else {
                    [0.0; 3]
                }
            }),
            stance: self.stance,
            push_n: [push.x, push.y, push.z],
            ground: (0..=40)
                .map(|j| {
                    let x = base.x - 1.0 + 0.05 * j as f64;
                    [x, ground(x, base.y)]
                })
                .collect(),
        }
    }
}

/// Runs the Go2 under `controller`.
pub fn run(controller: &mut dyn ForceController, cfg: &Go2Config) -> Go2Run {
    let mut g = Go2Sim::new(controller, *cfg, [0.0, 0.0]);
    let steps = (cfg.duration_s / 0.002).round() as usize;
    let mut frames = Vec::new();
    let (mut ev, mut eh, mut eatt) = (Vec::new(), Vec::new(), Vec::new());
    for k in 0..steps {
        let t = g.time_s();
        let v_cmd = speed_command(t);
        let push = if cfg.pushes { push_force(t) } else { Vec3::ZERO };
        let tick = g.step(v_cmd, push);
        if t >= 1.5 {
            ev.push(tick.velocity_mps.x - v_cmd[0]);
            eh.push(tick.com.z - tick.height_cmd_m);
            eatt.push(tick.roll_pitch_rad[0].hypot(tick.roll_pitch_rad[1]));
        }
        if k % 5 == 0 {
            frames.push(g.frame(&tick, v_cmd[0], push));
        }
    }
    let rms = robo_control::metrics::rms;
    let last = frames.last().unwrap();
    Go2Run {
        name: g.controller_name(),
        frames: frames.clone(),
        fell_at_s: g.fell_at_s(),
        metrics: vec![
            ("fell".into(), if g.fell_at_s().is_some() { 1.0 } else { 0.0 }),
            ("rms_speed_err_mps".into(), rms(&ev)),
            ("rms_height_err_m".into(), rms(&eh)),
            ("rms_tilt_rad".into(), rms(&eatt)),
            ("lateral_drift_m".into(), last.base_m[1]),
            ("distance_m".into(), last.base_m[0]),
        ],
    }
}
