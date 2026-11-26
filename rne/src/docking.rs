//! Planar docking comparison with RNE chassis dynamics and lumped wheel forces.
//! Omni rollers are free in the lateral direction. Swerve tires resist lateral slip.
//! Vertical load is fixed and distributed evenly across the wheels.

use nalgebra::{Rotation2, Vector2};
use rne_dynamics::{forward_dynamics, integrate_configuration, ArticulatedModel};
use rne_ecs::World;
use rne_math::Vec3;
use rne_robot::{FloatingBase, Robot};
use rne_urdf_import::{parse_urdf_document, spawn_urdf_document_with_config, UrdfSpawnConfig};
use robo_control::model::wrap;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

const DT: f64 = 0.002;
const MASS: f64 = 18.0;
const HALF_FOOTPRINT: f64 = 0.33;
const RADIUS: f64 = 0.08;
const LATERAL_START: f64 = 2.0;
const REFERENCE_END: f64 = 6.0;
const GOAL_Y: f64 = 2.0;
const BAY_START: f64 = 1.15;
const BAY_HALF_WIDTH: f64 = 0.40;

const CHASSIS: &str = r#"<robot name="docking_base"><link name="chassis"><inertial>
<mass value="18"/><inertia ixx="0.60" ixy="0" ixz="0" iyy="0.60" iyz="0" izz="1.08"/>
</inertial></link></robot>"#;

/// Wheel layout and steering mechanism.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Drive {
    /// Three equally spaced omni wheels.
    Omni,
    /// Four independent steerable wheels.
    Swerve,
}

/// Recorded wheel pose. Locations and steering axes are relative to the chassis.
#[derive(Debug, Clone, Copy)]
pub struct WheelPose {
    /// Wheel center in body coordinates [m].
    pub xy: [f64; 2],
    /// Rolling direction [rad].
    pub steering: f64,
    /// Integrated wheel rotation [rad].
    pub spin: f64,
}

/// Sampled state for replay.
#[derive(Debug, Clone)]
pub struct Frame {
    /// Simulation time [s].
    pub t: f64,
    /// Chassis center [m].
    pub xy: [f64; 2],
    /// Chassis yaw [rad].
    pub yaw: f64,
    /// Reference point [m].
    pub reference: [f64; 2],
    /// Wheel transforms.
    pub wheels: Vec<WheelPose>,
}

/// Result of the docking benchmark.
#[derive(Debug, Clone)]
pub struct DockingRun {
    /// Samples at 20 Hz.
    pub frames: Vec<Frame>,
    /// Time from start to settled at the goal [s].
    pub arrival_s: Option<f64>,
    /// RMS shortest distance to the L-shaped path, evaluated over the shared six-second reference [m].
    pub rms_path_error_m: f64,
    /// Worst absolute yaw during the run [rad].
    pub max_heading_error_rad: f64,
    /// Minimum signed clearance from the bay sides during entry [m].
    pub min_clearance_m: f64,
    /// RMS slip in the driven directions over the shared six-second reference [m/s].
    pub rms_driven_slip_mps: f64,
    /// RMS lateral slip at swerve wheels over the same interval [m/s].
    pub rms_lateral_slip_mps: Option<f64>,
    /// Time after lateral motion is requested until all swerve modules stay within
    /// five degrees of their requested axes for 0.1 s.
    pub steering_settle_s: Option<f64>,
}

struct Wheel {
    xy: Vector2<f64>,
    steering: f64,
    spin: f64,
    rate: f64,
}

fn chassis() -> ArticulatedModel {
    let mut world = World::new();
    let urdf = parse_urdf_document(CHASSIS).expect("docking chassis URDF");
    let cfg = UrdfSpawnConfig {
        attach_colliders: false,
        use_declared_inertial_masses: true,
        ..UrdfSpawnConfig::default()
    };
    let spawned = spawn_urdf_document_with_config(&mut world, &urdf, cfg).expect("spawn docking chassis");
    let base = world.get::<Robot>(spawned.robot).expect("robot").base_link;
    world.entity_mut(base).insert(FloatingBase);
    ArticulatedModel::from_robot_with_gravity(&world, spawned.robot, Vec3::ZERO).expect("docking dynamics")
}

fn reference(t: f64) -> (Vector2<f64>, Vector2<f64>) {
    if t < LATERAL_START {
        (Vector2::new(-1.0 + 0.5 * t, 0.0), Vector2::new(0.5, 0.0))
    } else if t < REFERENCE_END {
        (Vector2::new(0.0, 0.5 * (t - LATERAL_START)), Vector2::new(0.0, 0.5))
    } else {
        (Vector2::new(0.0, GOAL_Y), Vector2::zeros())
    }
}

fn path_error(p: Vector2<f64>) -> f64 {
    let leg1 = (p - Vector2::new(p.x.clamp(-1.0, 0.0), 0.0)).norm();
    let leg2 = (p - Vector2::new(0.0, p.y.clamp(0.0, GOAL_Y))).norm();
    leg1.min(leg2)
}

fn wheel_velocity(position: Vector2<f64>, velocity: Vector2<f64>, omega: f64) -> Vector2<f64> {
    velocity + omega * Vector2::new(-position.y, position.x)
}

/// Runs the same fixed-yaw trajectory with either wheel layout.
pub fn dock(drive: Drive) -> DockingRun {
    let swerve = drive == Drive::Swerve;
    let layout: Vec<_> = if swerve {
        [(-0.23, -0.23), (-0.23, 0.23), (0.23, -0.23), (0.23, 0.23)]
            .into_iter()
            .map(|(x, y)| (Vector2::new(x, y), 0.0))
            .collect()
    } else {
        (0..3)
            .map(|i| {
                let a = i as f64 * TAU / 3.0;
                (0.24 * Vector2::new(a.cos(), a.sin()), a + FRAC_PI_2)
            })
            .collect()
    };
    let mut wheels: Vec<_> = layout
        .into_iter()
        .map(|(xy, steering)| Wheel {
            xy,
            steering,
            spin: 0.0,
            rate: 0.0,
        })
        .collect();
    let count = wheels.len() as f64;
    // Match total drive torque, chassis inertia and nominal grip across layouts.
    let max_torque = 4.8 / count;
    let motor_gain = 0.68 / count;
    let wheel_inertia = 0.02 / count;
    let grip = 0.75 * MASS * 9.81 / count;
    let model = chassis();
    let mut q = vec![-1.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let mut qd = [0.0; 6];
    let mut frames = Vec::new();
    let mut min_clearance = f64::INFINITY;
    let mut max_heading: f64 = 0.0;
    let (mut error_sq, mut slip_sq, mut lateral_sq, mut samples) = (0.0, 0.0, 0.0, 0);
    let (mut steer_ticks, mut goal_ticks) = (0, 0);
    let (mut steering_settle_s, mut arrival_s) = (None, None);

    for k in 0..5000 {
        let t = k as f64 * DT;
        let (target, feedforward) = reference(t);
        let position = Vector2::new(q[0], q[1]);
        let yaw = q[5];
        let body = Rotation2::new(yaw);
        let velocity = Vector2::new(qd[0], qd[1]);
        let mut command = body.inverse() * (feedforward + 2.8 * (target - position));
        command = command.cap_magnitude(0.8);
        let yaw_command = (-5.0 * yaw - 0.7 * qd[5]).clamp(-1.2, 1.2);
        let mut force = Vector2::zeros();
        let mut moment = 0.0;
        let mut max_steering_error: f64 = 0.0;

        for w in &mut wheels {
            let command = wheel_velocity(w.xy, command, yaw_command);
            if swerve && command.norm() > 1e-4 {
                let mut delta = wrap(command.y.atan2(command.x) - w.steering);
                if delta.abs() > FRAC_PI_2 {
                    delta = wrap(delta + PI);
                }
                max_steering_error = max_steering_error.max(delta.abs());
                w.steering = wrap(w.steering + delta.clamp(-3.5 * DT, 3.5 * DT));
            }
            let axis = Vector2::new(w.steering.cos(), w.steering.sin());
            let side = Vector2::new(-axis.y, axis.x);
            let hub = wheel_velocity(w.xy, velocity, qd[5]);
            let slip = RADIUS * w.rate - axis.dot(&hub);
            let lateral = side.dot(&hub);
            let drive_force = 160.0 / count * slip;
            let lateral_force = if swerve { -180.0 / count * lateral } else { 0.0 };
            let contact = Vector2::new(drive_force, lateral_force).cap_magnitude(grip);
            let reaction = axis * contact.x + side * contact.y;
            force += reaction;
            moment += w.xy.x * reaction.y - w.xy.y * reaction.x;
            let drive = (axis.dot(&command) / RADIUS - w.rate) * motor_gain;
            w.rate += DT * (drive.clamp(-max_torque, max_torque) - RADIUS * contact.x) / wheel_inertia;
            w.spin = wrap(w.spin + DT * w.rate);
            if t < REFERENCE_END {
                slip_sq += slip * slip;
                lateral_sq += lateral * lateral;
            }
        }

        if t < REFERENCE_END {
            error_sq += path_error(position).powi(2);
            samples += 1;
        }
        let (sin, cos) = yaw.sin_cos();
        let extent = HALF_FOOTPRINT * (cos.abs() + sin.abs());
        if position.y + extent >= BAY_START {
            min_clearance = min_clearance.min(BAY_HALF_WIDTH - position.x.abs() - extent);
        }
        max_heading = max_heading.max(yaw.abs());

        if swerve && (LATERAL_START..REFERENCE_END).contains(&t) && steering_settle_s.is_none() {
            steer_ticks = if max_steering_error <= 5_f64.to_radians() {
                steer_ticks + 1
            } else {
                0
            };
            if steer_ticks == 50 {
                steering_settle_s = Some(t - LATERAL_START - 49.0 * DT);
            }
        }
        if t >= REFERENCE_END {
            let at_goal = (position - target).norm() < 0.03
                && velocity.norm() < 0.04
                && yaw.abs() < 3_f64.to_radians()
                && qd[5].abs() < 0.05
                && min_clearance > 0.0;
            goal_ticks = if at_goal { goal_ticks + 1 } else { 0 };
            if goal_ticks == 125 {
                arrival_s = Some(t - 124.0 * DT);
            }
        }

        if k % 25 == 0 {
            frames.push(Frame {
                t,
                xy: position.into(),
                yaw,
                reference: target.into(),
                wheels: wheels
                    .iter()
                    .map(|w| WheelPose {
                        xy: w.xy.into(),
                        steering: w.steering,
                        spin: w.spin,
                    })
                    .collect(),
            });
        }
        if arrival_s.is_some() {
            break;
        }

        let effort = [force.x, force.y, 0.0, 0.0, 0.0, moment];
        let qdd = forward_dynamics(&model, &q, &qd, &effort).expect("advance chassis dynamics");
        for (v, a) in qd.iter_mut().zip(qdd) {
            *v += DT * a;
        }
        q = integrate_configuration(&model, &q, &qd, DT).expect("integrate chassis pose");
    }

    DockingRun {
        frames,
        arrival_s,
        rms_path_error_m: (error_sq / samples as f64).sqrt(),
        max_heading_error_rad: max_heading,
        min_clearance_m: min_clearance,
        rms_driven_slip_mps: (slip_sq / (samples as f64 * count)).sqrt(),
        rms_lateral_slip_mps: swerve.then(|| (lateral_sq / (samples as f64 * count)).sqrt()),
        steering_settle_s,
    }
}
