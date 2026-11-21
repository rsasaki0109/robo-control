//! Four articulated Go2s on RNE swap corners through the middle, steered by
//! robot-planning's multi-robot methods:
//!
//! * decentralised reciprocal avoidance ([`robo_control::robo_planning::multi_robot::RvoSim`]):
//!   every 0.1 s each robot's measured position and velocity go in, a velocity
//!   command comes out;
//! * prioritized space-time A* ([`robo_control::robo_planning::multi_robot::prioritized_plan`]):
//!   paths are reserved up front on a 0.5 m grid and tracked with velocity
//!   feed-forward plus position feedback.
//!
//! Each Go2 trots on its own RNE simulation with the convex-MPC force
//! controller; the robots do not touch each other physically, so the
//! separation they keep is the measure.

use crate::go2::{go2_srb, Go2Config, Go2Sim};
use rne_math::Vec3;
use robo_control::legged3d::ConvexMpc3d;
use robo_control::robo_planning::multi_robot::{prioritized_plan, Agent, RvoConfig, RvoSim};
use robo_control::robo_planning::{Map2D, Vec2};

/// How the fleet is steered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FleetMethod {
    /// sampled reciprocal velocity obstacles, decentralised
    Rvo,
    /// prioritized space-time A*, centralised
    Prioritized,
}

impl FleetMethod {
    /// Display name.
    pub fn name(self) -> &'static str {
        match self {
            FleetMethod::Rvo => "reciprocal avoidance (RVO)",
            FleetMethod::Prioritized => "prioritized space-time A*",
        }
    }
}

/// Drawing frame (every 20 ms).
#[derive(Debug, Clone)]
pub struct FleetFrame {
    /// time [s]
    pub t: f64,
    /// base (x, y) of each robot
    pub bases: Vec<[f64; 2]>,
}

/// Result of a fleet run.
#[derive(Debug, Clone)]
pub struct FleetRun {
    /// method
    pub name: String,
    /// frames
    pub frames: Vec<FleetFrame>,
    /// goals
    pub goals: Vec<Vec2>,
    /// planned paths (prioritized only)
    pub plans: Vec<Vec<Vec2>>,
    /// metrics
    pub metrics: Vec<(String, f64)>,
}

/// Robot "radius" used for planning and for the separation metric [m].
pub const RADIUS_M: f64 = 0.4;

/// Runs the corner swap with `method` for `duration_s`.
pub fn corner_swap(method: FleetMethod, duration_s: f64) -> FleetRun {
    let starts = [
        Vec2::new(-1.75, -1.75),
        Vec2::new(1.75, 1.75),
        Vec2::new(-1.75, 1.75),
        Vec2::new(1.75, -1.75),
    ];
    let goals: Vec<Vec2> = starts.iter().map(|s| Vec2::new(-s.x, -s.y)).collect();
    let map = Map2D::new(Vec2::new(-3.0, -3.0), Vec2::new(3.0, 3.0));
    let n = starts.len();
    let q = rne_locomotion::Quadruped::go2().expect("Go2");
    let p = go2_srb(&q);
    let mut ctrls: Vec<ConvexMpc3d> = (0..n).map(|_| ConvexMpc3d::new(p)).collect();
    let cfg = Go2Config {
        pushes: false,
        ..Go2Config::default()
    };
    let mut sims: Vec<Go2Sim> = ctrls
        .iter_mut()
        .zip(&starts)
        .map(|(c, s)| Go2Sim::new(c, cfg, [s.x, s.y]))
        .collect();
    let v_max = 0.5;
    // prioritized plans: one grid step every `tau` seconds
    let cell = 0.5;
    let tau = cell / (0.8 * v_max);
    let plans: Vec<Vec<Vec2>> = if method == FleetMethod::Prioritized {
        prioritized_plan(&map, &starts, &goals, RADIUS_M, cell, 2.0 * RADIUS_M + 0.1, 300)
            .expect("prioritized plan")
            .into_iter()
            .map(|tp| tp.points)
            .collect()
    } else {
        vec![]
    };
    let mut rvo = RvoSim {
        agents: starts
            .iter()
            .zip(&goals)
            .map(|(s, g)| Agent {
                pos: *s,
                vel: Vec2::default(),
                goal: *g,
                radius_m: RADIUS_M,
                v_max_mps: v_max,
            })
            .collect(),
        map: map.clone(),
        cfg: RvoConfig {
            dt_s: 0.1,
            horizon_s: 3.0,
            ..RvoConfig::default()
        },
    };
    let ref_at = |path: &[Vec2], t: f64| -> (Vec2, Vec2) {
        let s = (t / tau).max(0.0);
        let k = (s.floor() as usize).min(path.len() - 1);
        let k2 = (k + 1).min(path.len() - 1);
        let u = if k2 > k { s - s.floor() } else { 0.0 };
        (path[k].lerp(path[k2], u), (path[k2] - path[k]) * (1.0 / tau))
    };
    let steps = (duration_s / 0.002).round() as usize;
    let mut cmds = vec![[0.0, 0.0]; n];
    let mut frames = Vec::new();
    let mut min_gap = f64::INFINITY;
    let mut arrived_at = None;
    for k in 0..steps {
        let t = k as f64 * 0.002;
        let walking = t >= 1.2;
        if k % 50 == 0 {
            let measured: Vec<(Vec2, Vec2)> = sims
                .iter()
                .map(|g| {
                    let (b, v) = g.base();
                    (Vec2::new(b.x, b.y), Vec2::new(v.x, v.y))
                })
                .collect();
            for i in 0..n {
                cmds[i] = if !walking {
                    [0.0, 0.0]
                } else {
                    match method {
                        FleetMethod::Rvo => {
                            rvo.agents[i].pos = measured[i].0;
                            rvo.agents[i].vel = measured[i].1;
                            [0.0, 0.0]
                        }
                        FleetMethod::Prioritized => {
                            let (pr, vr) = ref_at(&plans[i], t - 1.2);
                            let c = vr + (pr - measured[i].0) * 1.5;
                            let sp = c.norm().min(v_max);
                            let c = if c.norm() > 1e-9 { c * (sp / c.norm()) } else { c };
                            [c.x, c.y]
                        }
                    }
                };
            }
            if walking && method == FleetMethod::Rvo {
                for (i, v) in rvo.velocities().into_iter().enumerate() {
                    cmds[i] = [v.x, v.y];
                }
            }
            // separation and arrival
            for i in 0..n {
                for j in i + 1..n {
                    min_gap = min_gap.min(measured[i].0.dist(measured[j].0) - 2.0 * RADIUS_M);
                }
            }
            if arrived_at.is_none() && measured.iter().zip(&goals).all(|(m, g)| m.0.dist(*g) < 0.25) {
                arrived_at = Some(t);
            }
        }
        let mut bases = Vec::with_capacity(n);
        for (g, c) in sims.iter_mut().zip(&cmds) {
            g.step(*c, Vec3::ZERO);
            let (b, _) = g.base();
            bases.push([b.x, b.y]);
        }
        if k % 10 == 0 {
            frames.push(FleetFrame { t, bases });
        }
    }
    let fell = sims.iter().filter(|g| g.fell_at_s().is_some()).count();
    let final_err = frames
        .last()
        .unwrap()
        .bases
        .iter()
        .zip(&goals)
        .map(|(b, g)| (b[0] - g.x).hypot(b[1] - g.y))
        .fold(0.0, f64::max);
    FleetRun {
        name: method.name().into(),
        frames,
        goals,
        plans,
        metrics: vec![
            ("min_separation_m".into(), min_gap),
            ("all_arrived_s".into(), arrived_at.unwrap_or(f64::NAN)),
            ("max_final_err_m".into(), final_err),
            ("fallen".into(), fell as f64),
        ],
    }
}
