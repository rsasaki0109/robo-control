//! A full planning + control mission on RNE: the Go2 crosses the warehouse of
//! robot-planning's benchmark scenes, following an A* -> line-of-sight ->
//! spline -> speed-profile reference while facing along it, with a
//! robot-control force controller doing the walking.

use crate::go2::{Go2Config, Go2Sim};
use rne_math::Vec3;
use robo_control::legged3d::ForceController;
use robo_control::metrics::{distance_to_polyline, max_abs, rms};
use robo_control::robo_planning::grid::{self, OccupancyGrid};
use robo_control::robo_planning::trajectory::{
    densify, sample_traj, spline_path, time_parameterize, SpeedLimits, TrajPoint,
};
use robo_control::robo_planning::{scenes, Map2D, Vec2};

/// Go2 footprint radius used for planning and clearance [m].
pub const GO2_RADIUS_M: f64 = 0.38;

/// Mission frame (every 0.1 s).
#[derive(Debug, Clone, Copy)]
pub struct MissionFrame {
    /// time [s]
    pub t: f64,
    /// base (x, y)
    pub base: [f64; 2],
    /// heading [rad]
    pub yaw: f64,
}

/// Result of a mission run.
#[derive(Debug, Clone)]
pub struct MissionRun {
    /// controller name
    pub name: String,
    /// warehouse map
    pub map: Map2D,
    /// reference
    pub reference: Vec<TrajPoint>,
    /// frames
    pub frames: Vec<MissionFrame>,
    /// metrics
    pub metrics: Vec<(String, f64)>,
}

/// Plans the warehouse crossing for the Go2.
pub fn plan_warehouse() -> (Map2D, Vec<TrajPoint>) {
    let sc = scenes::warehouse();
    let g = OccupancyGrid::from_map(&sc.map, 0.1, GO2_RADIUS_M + 0.1);
    let a = grid::astar(&g, sc.start.pos(), sc.goal.pos()).expect("A* path");
    let pruned = grid::prune_line_of_sight(&sc.map, &a.points, GO2_RADIUS_M + 0.15);
    let path = spline_path(&densify(&pruned, 0.8), 0.05);
    let reference = time_parameterize(
        &path,
        &SpeedLimits {
            v_max_mps: 0.7,
            a_max_mps2: 0.4,
            a_lat_max_mps2: 0.4,
        },
    );
    (sc.map, reference)
}

/// Walks the Go2 through the warehouse with `controller`.
pub fn warehouse(controller: &mut dyn ForceController) -> MissionRun {
    let (map, reference) = plan_warehouse();
    let start = reference[0].p;
    let t_ref = reference.last().unwrap().t_s;
    let cfg = Go2Config {
        pushes: false,
        ..Go2Config::default()
    };
    let mut g = Go2Sim::new(controller, cfg, [start.x, start.y]);
    let t_walk = 1.2;
    let duration = t_walk + t_ref + 6.0;
    let steps = (duration / 0.002).round() as usize;
    let line: Vec<Vec2> = reference.iter().map(|p| Vec2::new(p.p.x, p.p.y)).collect();
    let goal = *line.last().unwrap();
    let (mut cte, mut clear, mut frames) = (Vec::new(), f64::INFINITY, Vec::new());
    let mut arrived = None;
    let mut cmd = ([0.0, 0.0], start.yaw_rad, 0.0);
    for k in 0..steps {
        let t = k as f64 * 0.002;
        let (b, _) = g.base();
        let pos = Vec2::new(b.x, b.y);
        if k % 10 == 0 {
            // reference position, velocity and heading at this time
            let r = sample_traj(&reference, (t - t_walk).max(0.0));
            let (c, s) = (r.p.yaw_rad.cos(), r.p.yaw_rad.sin());
            let walking = t >= t_walk;
            let v_ref = if walking {
                Vec2::new(r.v_mps * c, r.v_mps * s)
            } else {
                Vec2::default()
            };
            let fb = if walking {
                (Vec2::new(r.p.x, r.p.y) - pos) * 1.2
            } else {
                Vec2::default()
            };
            let mut v = v_ref + fb;
            if v.norm() > 0.8 {
                v = v * (0.8 / v.norm());
            }
            cmd = (
                [v.x, v.y],
                r.p.yaw_rad,
                if walking { r.v_mps * r.p.curvature } else { 0.0 },
            );
        }
        g.step_heading(cmd.0, cmd.1, cmd.2, Vec3::ZERO);
        if t >= t_walk {
            cte.push(distance_to_polyline(pos, &line));
            clear = clear.min(map.clearance(pos) - GO2_RADIUS_M);
        }
        if arrived.is_none() && t > t_walk && pos.dist(goal) < 0.2 {
            arrived = Some(t - t_walk);
        }
        if k % 50 == 0 {
            frames.push(MissionFrame {
                t,
                base: [b.x, b.y],
                yaw: g.yaw(),
            });
        }
    }
    let last = frames.last().unwrap();
    MissionRun {
        name: g.controller_name(),
        map,
        reference,
        metrics: vec![
            ("fell".into(), if g.fell_at_s().is_some() { 1.0 } else { 0.0 }),
            ("rms_cte_m".into(), rms(&cte)),
            ("max_cte_m".into(), max_abs(&cte)),
            ("min_clearance_m".into(), clear),
            ("arrival_s".into(), arrived.unwrap_or(f64::NAN)),
            ("plan_duration_s".into(), t_ref),
            (
                "goal_err_m".into(),
                (last.base[0] - goal.x).hypot(last.base[1] - goal.y),
            ),
        ],
        frames,
    }
}
