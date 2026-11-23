//! The warehouse mission with people and carts walking the aisles: the Go2
//! (convex MPC walking on RNE) follows robot-planning's global route and a
//! local planner dodges the movers. Compared: no avoidance (the planned
//! timing only), DWA from robot-planning and
//! MPPI from robot-control (predicts them at constant velocity).
//!
//! The movers are kinematic discs on fixed back-and-forth routes, so every
//! method meets the same crowd; contact with them is measured, not simulated.

use crate::go2::{Go2Config, Go2Sim};
use crate::mission::{plan_warehouse, GO2_RADIUS_M};
use nalgebra::DVector;
use rne_math::Vec3;
use robo_control::legged3d::ForceController;
use robo_control::metrics::distance_to_polyline;
use robo_control::models::Unicycle;
use robo_control::mppi::Mppi;
use robo_control::robo_planning::dwa::{self, DwaConfig, MovingDisc};
use robo_control::robo_planning::geometry::normalize_angle;
use robo_control::robo_planning::trajectory::{sample_traj, TrajPoint};
use robo_control::robo_planning::{Map2D, Pose2, Vec2};

/// A person or cart pacing back and forth between `a` and `b`.
#[derive(Debug, Clone, Copy)]
pub struct Mover {
    /// one end
    pub a: Vec2,
    /// other end
    pub b: Vec2,
    /// speed [m/s]
    pub speed_mps: f64,
    /// time offset [s]
    pub phase_s: f64,
    /// radius [m]
    pub radius_m: f64,
}

impl Mover {
    /// Position at time `t`.
    pub fn pos(&self, t: f64) -> Vec2 {
        let len = self.a.dist(self.b);
        let s = ((t + self.phase_s) * self.speed_mps).rem_euclid(2.0 * len);
        let s = if s > len { 2.0 * len - s } else { s };
        self.a + (self.b - self.a) * (s / len)
    }

    /// Velocity at time `t`.
    pub fn vel(&self, t: f64) -> Vec2 {
        (self.pos(t + 0.01) - self.pos(t - 0.01)) * 50.0
    }
}

/// Default timing of the cart [s].
pub const CART_PHASE_S: f64 = 6.0;

/// The warehouse crowd, the cart shifted by `cart_phase_s`.
pub fn movers(cart_phase_s: f64) -> Vec<Mover> {
    vec![
        // a person coming the other way through the bottom cross-passage
        Mover {
            a: Vec2::new(5.3, 3.2),
            b: Vec2::new(9.9, 3.2),
            speed_mps: 0.5,
            phase_s: 5.5,
            radius_m: 0.3,
        },
        // a cart crossing the top cross-aisle
        Mover {
            a: Vec2::new(11.1, 8.65),
            b: Vec2::new(11.1, 11.5),
            speed_mps: 0.5,
            phase_s: cart_phase_s,
            radius_m: 0.45,
        },
        // a person crossing the open bay
        Mover {
            a: Vec2::new(15.7, 7.6),
            b: Vec2::new(19.3, 7.6),
            speed_mps: 0.6,
            phase_s: 3.45,
            radius_m: 0.3,
        },
    ]
}

/// How the Go2 deals with the movers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Avoidance {
    /// follow the planned timing and ignore the movers
    None,
    /// robot-planning's dynamic window approach among moving discs
    Dwa,
    /// robot-control's MPPI with constant-velocity predictions of the movers
    Mppi,
}

impl Avoidance {
    /// Display name.
    pub fn name(self) -> &'static str {
        match self {
            Avoidance::None => "no avoidance",
            Avoidance::Dwa => "DWA",
            Avoidance::Mppi => "MPPI",
        }
    }
}

/// Frame every 0.1 s.
#[derive(Debug, Clone)]
pub struct CrowdFrame {
    /// time [s]
    pub t: f64,
    /// base (x, y)
    pub base: [f64; 2],
    /// heading [rad]
    pub yaw: f64,
    /// local plan (predicted path), if any
    pub local: Vec<Vec2>,
}

/// Result of one run.
#[derive(Debug, Clone)]
pub struct CrowdRun {
    /// method
    pub method: Avoidance,
    /// map
    pub map: Map2D,
    /// global reference
    pub reference: Vec<TrajPoint>,
    /// movers
    pub movers: Vec<Mover>,
    /// frames
    pub frames: Vec<CrowdFrame>,
    /// metrics
    pub metrics: Vec<(String, f64)>,
}

const T_WALK: f64 = 1.2;
const V_MAX: f64 = 0.7;

/// Nearest path index to `p` within a window ahead of `from`.
fn progress(path: &[Vec2], from: usize, p: Vec2) -> usize {
    let hi = (from + 120).min(path.len());
    (from.saturating_sub(10)..hi)
        .min_by(|&i, &j| path[i].dist(p).total_cmp(&path[j].dist(p)))
        .unwrap_or(from)
}

/// Runs the crowd mission with `method`, the Go2 walking with `controller`.
pub fn warehouse_crowd(controller: &mut dyn ForceController, method: Avoidance) -> CrowdRun {
    warehouse_crowd_with(controller, method, CART_PHASE_S)
}

/// [`warehouse_crowd`] with the cart shifted by `cart_phase_s`.
pub fn warehouse_crowd_with(controller: &mut dyn ForceController, method: Avoidance, cart_phase_s: f64) -> CrowdRun {
    let (map, reference) = plan_warehouse();
    let movers = movers(cart_phase_s);
    let path: Vec<Vec2> = reference.iter().map(|p| Vec2::new(p.p.x, p.p.y)).collect();
    let goal = *path.last().unwrap();
    let start = reference[0].p;
    let t_ref = reference.last().unwrap().t_s;
    let duration = T_WALK + t_ref + 40.0;
    let mut g = Go2Sim::new(
        controller,
        Go2Config {
            pushes: false,
            ..Go2Config::default()
        },
        [start.x, start.y],
    );
    let uni = Unicycle {
        v_max_mps: V_MAX,
        w_max_radps: 1.2,
        ..Unicycle::default()
    };
    let mut mppi = Mppi::new(2, 30, 0.1, 384, 0.3, DVector::from_vec(vec![0.3, 0.8]), 7);
    mppi.set_nominal(vec![DVector::from_vec(vec![0.5, 0.0]); 30]);
    let dwa_cfg = DwaConfig {
        v_max_mps: V_MAX,
        w_max_radps: 1.2,
        a_max_mps2: 0.8,
        alpha_max_radps2: 3.0,
        horizon_s: 2.5,
        v_min_mps: -0.3,
        robot_radius_m: GO2_RADIUS_M + 0.05,
        ..DwaConfig::default()
    };
    let (mut s_idx, mut vw, mut heading) = (0usize, (0.0f64, 0.0f64), start.yaw_rad);
    let mut cmd = ([0.0, 0.0], start.yaw_rad, 0.0);
    let mut local = Vec::new();
    let (mut frames, mut min_gap, mut clear) = (Vec::new(), f64::INFINITY, f64::INFINITY);
    let (mut contacts, mut contact_time, mut in_contact) = (0usize, 0.0, false);
    let mut arrived = None;
    let steps = (duration / 0.002).round() as usize;
    for k in 0..steps {
        let t = k as f64 * 0.002;
        let (b, _) = g.base();
        let pos = Vec2::new(b.x, b.y);
        let yaw = g.yaw();
        if k % 50 == 0 && arrived.is_none() {
            let walking = t >= T_WALK;
            s_idx = progress(&path, s_idx, pos);
            if !walking {
                cmd = ([0.0, 0.0], start.yaw_rad, 0.0);
            } else if method != Avoidance::None && pos.dist(goal) < 0.5 {
                // final approach: plain position control onto the goal
                let mut v = (goal - pos) * 1.2;
                if v.norm() > 0.4 {
                    v = v * (0.4 / v.norm());
                }
                cmd = ([v.x, v.y], heading, 0.0);
                local.clear();
            } else if method == Avoidance::None {
                // the plain mission tracker
                let r = sample_traj(&reference, t - T_WALK);
                let (c, s) = (r.p.yaw_rad.cos(), r.p.yaw_rad.sin());
                let mut v = Vec2::new(r.v_mps * c, r.v_mps * s) + (Vec2::new(r.p.x, r.p.y) - pos) * 1.2;
                if v.norm() > 0.8 {
                    v = v * (0.8 / v.norm());
                }
                heading = r.p.yaw_rad;
                cmd = ([v.x, v.y], r.p.yaw_rad, r.v_mps * r.p.curvature);
                local.clear();
            } else {
                // carrot on the global path ahead of the robot
                let carrot = path[(s_idx + 30).min(path.len() - 1)];
                let pose = Pose2::new(pos.x, pos.y, yaw);
                (vw, local) = match method {
                    Avoidance::Dwa => {
                        let discs: Vec<MovingDisc> = movers
                            .iter()
                            .map(|m| MovingDisc {
                                pos: m.pos(t),
                                vel: m.vel(t),
                                radius_m: m.radius_m,
                            })
                            .collect();
                        match dwa::plan_with_movers(&map, &discs, pose, vw.0, vw.1, carrot, &dwa_cfg) {
                            Some(c) => ((c.v_mps, c.w_radps), c.rollout.iter().map(|p| p.pos()).collect()),
                            None => ((0.0, 0.0), Vec::new()),
                        }
                    }
                    _ => {
                        let x0 = DVector::from_vec(vec![pos.x, pos.y, yaw, vw.0, vw.1]);
                        let window: Vec<Vec2> = path[s_idx.saturating_sub(5)..(s_idx + 90).min(path.len())].to_vec();
                        let target = path[(s_idx + 42).min(path.len() - 1)];
                        let pred: Vec<Vec<(Vec2, f64)>> = (0..=30)
                            .map(|j| {
                                let tj = j as f64 * 0.1;
                                movers.iter().map(|m| (m.pos(t) + m.vel(t) * tj, m.radius_m)).collect()
                            })
                            .collect();
                        let cost = |p: Vec2, j: usize| {
                            let d = distance_to_polyline(p, &window);
                            let mut c = 4.0 * d * d;
                            let st = map.clearance(p) - GO2_RADIUS_M;
                            if st < 0.0 {
                                c += 400.0;
                            }
                            c += 60.0 * (0.25 - st).max(0.0).powi(2);
                            for (q, r) in &pred[j] {
                                let gap = p.dist(*q) - r - GO2_RADIUS_M;
                                // 10 cm for the gap between commanded and walked path
                                if gap < 0.1 {
                                    c += 3000.0;
                                }
                                c += 400.0 * (0.6 - gap).max(0.0).powi(2);
                            }
                            c
                        };
                        let u = mppi.control(
                            &uni,
                            &x0,
                            |j, x, u| {
                                cost(Vec2::new(x[0], x[1]), j) + 0.3 * (V_MAX - u[0]).powi(2) + 0.05 * u[1] * u[1]
                            },
                            |x| 6.0 * Vec2::new(x[0], x[1]).dist(target) + cost(Vec2::new(x[0], x[1]), 30),
                        );
                        let pts = mppi.predicted.iter().map(|x| Vec2::new(x[0], x[1])).collect();
                        ((u[0].max(-0.3), u[1]), pts)
                    }
                };
                // integrate the commanded heading, kept close to the measured one
                heading = normalize_angle(heading + vw.1 * 0.1);
                let lead = normalize_angle(heading - yaw).clamp(-0.35, 0.35);
                heading = normalize_angle(yaw + lead);
                cmd = ([vw.0 * yaw.cos(), vw.0 * yaw.sin()], heading, vw.1);
            }
        }
        if arrived.is_some() && k % 50 == 0 {
            // hold the goal
            let mut v = (goal - pos) * 1.0;
            if v.norm() > 0.3 {
                v = v * (0.3 / v.norm());
            }
            cmd = ([v.x, v.y], heading, 0.0);
        }
        g.step_heading(cmd.0, cmd.1, cmd.2, Vec3::ZERO);
        if t >= T_WALK {
            let gap = movers
                .iter()
                .map(|m| pos.dist(m.pos(t)) - m.radius_m - GO2_RADIUS_M)
                .fold(f64::INFINITY, f64::min);
            min_gap = min_gap.min(gap);
            if gap < 0.0 {
                contact_time += 0.002;
                if !in_contact {
                    contacts += 1;
                }
            }
            in_contact = gap < 0.0;
            clear = clear.min(map.clearance(pos) - GO2_RADIUS_M);
            if arrived.is_none() && pos.dist(goal) < 0.2 {
                arrived = Some(t - T_WALK);
            }
        }
        if k % 50 == 0 {
            frames.push(CrowdFrame {
                t,
                base: [b.x, b.y],
                yaw,
                local: local.clone(),
            });
        }
    }
    CrowdRun {
        method,
        map,
        reference,
        movers,
        frames,
        metrics: vec![
            ("fell".into(), if g.fell_at_s().is_some() { 1.0 } else { 0.0 }),
            ("contacts".into(), contacts as f64),
            ("contact_s".into(), contact_time),
            ("min_gap_m".into(), min_gap),
            ("min_shelf_clearance_m".into(), clear),
            ("arrival_s".into(), arrived.unwrap_or(f64::NAN)),
        ],
    }
}
