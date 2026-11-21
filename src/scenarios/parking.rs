//! Car parking with reversing: a Hybrid A* plan with cusps (Reeds-Shepp shots)
//! tracked by pure pursuit (with a reverse mode), LQR and LTV-MPC on the
//! bicycle with steering lag.

use super::Run;
use crate::controllers::pid::Pid;
use crate::lqr::TrackingLqr;
use crate::metrics::{distance_to_polyline, max_abs, rms};
use crate::model::{v, wrap, Dynamics};
use crate::models::Bicycle;
use crate::mpc::LtvMpc;
use crate::sim::{simulate, SimConfig};
use nalgebra::{DMatrix, DVector};
use robo_planning::hybrid_astar::{self, HybridAStarConfig};
use robo_planning::trajectory::{sample_traj, time_parameterize, PathPoint, SpeedLimits, TrajPoint};
use robo_planning::{Map2D, Pose2, Vec2};

/// Parking scenario output.
#[derive(Debug, Clone)]
pub struct ParkingResult {
    /// map (kerb, parked cars)
    pub map: Map2D,
    /// planner footprint settings (for collision checks and drawing)
    pub footprint: HybridAStarConfig,
    /// signed-speed reference (negative while reversing)
    pub reference: Vec<TrajPoint>,
    /// goal pose
    pub goal: Pose2,
    /// runs
    pub runs: Vec<Run>,
}

/// The parallel-parking street of the planning gallery.
pub fn street() -> (Map2D, Pose2, Pose2) {
    let map = Map2D::new(Vec2::new(0.0, -1.0), Vec2::new(30.0, 12.0))
        .rect(0.0, -1.0, 30.0, 0.2)
        .rect(3.0, 0.2, 7.4, 2.2)
        .rect(8.0, 0.2, 12.4, 2.2)
        .rect(18.6, 0.2, 23.0, 2.2)
        .rect(23.6, 0.2, 28.0, 2.2)
        .rect(0.0, 10.0, 30.0, 12.0);
    (map, Pose2::new(4.0, 6.0, 0.0), Pose2::new(14.2, 1.2, 0.0))
}

/// Turns a pose sequence with driving directions into a time-stamped reference
/// with signed speed: each direction segment gets its own rest-to-rest speed
/// profile and the car pauses `pause_s` at every cusp.
pub fn signed_reference(poses: &[Pose2], dirs: &[f64], lim: &SpeedLimits, pause_s: f64) -> Vec<TrajPoint> {
    let mut out: Vec<TrajPoint> = Vec::new();
    let mut i = 0;
    let mut t0 = 0.0;
    while i < poses.len() - 1 {
        let dir = dirs[i];
        let mut j = i;
        while j + 1 < poses.len() && dirs[j] == dir {
            j += 1;
        }
        // segment poses i..=j
        let mut path: Vec<PathPoint> = Vec::new();
        for p in &poses[i..=j] {
            if let Some(l) = path.last() {
                if (p.x - l.x).hypot(p.y - l.y) < 1e-6 {
                    continue;
                }
            }
            let s = path
                .last()
                .map_or(0.0, |l: &PathPoint| l.s_m + (p.x - l.x).hypot(p.y - l.y));
            path.push(PathPoint {
                s_m: s,
                x: p.x,
                y: p.y,
                yaw_rad: p.theta_rad,
                curvature: 0.0,
            });
        }
        for k in 1..path.len().saturating_sub(1) {
            let ds = path[k + 1].s_m - path[k - 1].s_m;
            // heading change per *signed* distance: what the bicycle's steering must produce
            path[k].curvature = wrap(path[k + 1].yaw_rad - path[k - 1].yaw_rad) / (dir * ds.max(1e-9));
        }
        if path.len() >= 2 {
            for mut tp in time_parameterize(&path, lim) {
                tp.t_s += t0;
                tp.v_mps *= dir;
                tp.a_mps2 *= dir;
                out.push(tp);
            }
            t0 = out.last().unwrap().t_s + pause_s;
        }
        i = j;
    }
    out
}

fn bicycle_ref(car: &Bicycle, tp: &TrajPoint) -> (DVector<f64>, DVector<f64>) {
    let d = (car.wheelbase_m * tp.p.curvature).atan();
    (v(&[tp.p.x, tp.p.y, tp.p.yaw_rad, tp.v_mps, d]), v(&[tp.a_mps2, d]))
}

/// Plans the parallel park and tracks it with three controllers.
pub fn parallel_park() -> ParkingResult {
    let (map, start, goal) = street();
    let footprint = HybridAStarConfig {
        min_turn_radius_m: 4.0,
        footprint_radius_m: 0.95,
        footprint_front_offset_m: 2.0,
        allow_reverse: true,
        xy_resolution_m: 0.3,
        ..Default::default()
    };
    let plan = hybrid_astar::plan(&map, start, goal, &footprint).expect("parking plan");
    let lim = SpeedLimits {
        v_max_mps: 1.5,
        a_max_mps2: 0.8,
        a_lat_max_mps2: 1.0,
    };
    let reference = signed_reference(&plan.poses, &plan.directions, &lim, 1.0);
    let t_end = reference.last().unwrap().t_s;
    let car = Bicycle::default();
    let cfg = SimConfig {
        dt_s: 0.02,
        duration_s: t_end + 4.0,
        ..Default::default()
    };
    let x0 = v(&[start.x, start.y, start.theta_rad, 0.0, 0.0]);
    let no_d = |_: f64, _: &DVector<f64>| DVector::zeros(5);
    let never = |_: f64, _: &DVector<f64>| false;
    let line: Vec<Vec2> = reference.iter().map(|p| Vec2::new(p.p.x, p.p.y)).collect();
    let metrics = |log: &crate::sim::Log| -> Vec<(String, f64)> {
        let cte: Vec<f64> = log
            .x
            .iter()
            .map(|x| distance_to_polyline(Vec2::new(x[0], x[1]), &line))
            .collect();
        let last = log.x.last().unwrap();
        let collisions = log
            .x
            .iter()
            .filter(|x| !hybrid_astar::car_free(&map, &footprint, Pose2::new(x[0], x[1], x[2])))
            .count();
        vec![
            ("rms_cte_m".into(), rms(&cte)),
            ("max_cte_m".into(), max_abs(&cte)),
            ("final_pos_err_m".into(), (last[0] - goal.x).hypot(last[1] - goal.y)),
            (
                "final_yaw_err_deg".into(),
                wrap(last[2] - goal.theta_rad).abs().to_degrees(),
            ),
            ("contact_steps".into(), collisions as f64),
        ]
    };
    let mut runs = Vec::new();

    // pure pursuit with a reverse mode: look ahead in time within the current
    // direction segment; when reversing, pursue with the car's rear and flip the steering
    let seg_end = |t: f64| -> f64 {
        let i = reference.partition_point(|p| p.t_s <= t).min(reference.len() - 1);
        let dir = reference[i].v_mps.signum();
        reference[i..]
            .iter()
            .take_while(|p| p.v_mps.signum() == dir || p.v_mps == 0.0)
            .last()
            .map_or(t, |p| p.t_s)
    };
    let mut pid = Pid::new(2.0, 0.3, 0.0, 3.0);
    let log = simulate(
        &car,
        x0.clone(),
        &cfg,
        |t, x| {
            let r = sample_traj(&reference, t);
            let dir = if r.v_mps.abs() > 1e-6 {
                r.v_mps.signum()
            } else {
                sample_traj(&reference, t + 0.1).v_mps.signum()
            };
            let tl = (t + 1.0).min(seg_end(t));
            let target = sample_traj(&reference, tl);
            let yaw_eff = if dir < 0.0 { x[2] + std::f64::consts::PI } else { x[2] };
            let (dx, dy) = (target.p.x - x[0], target.p.y - x[1]);
            let ld = dx.hypot(dy).max(0.5);
            let alpha = wrap(dy.atan2(dx) - yaw_eff);
            let kappa = 2.0 * alpha.sin() / ld;
            let delta = dir * (car.wheelbase_m * kappa).atan();
            v(&[pid.update(r.v_mps - x[3], cfg.dt_s) + r.a_mps2, delta])
        },
        no_d,
        never,
    );
    runs.push(Run {
        name: "pure pursuit (reverse mode)".into(),
        metrics: metrics(&log),
        log,
    });

    let lqr = TrackingLqr {
        q: DMatrix::from_diagonal(&v(&[10.0, 10.0, 20.0, 1.0, 0.5])),
        r: DMatrix::from_diagonal(&v(&[1.0, 2.0])),
        horizon: 150,
        dt_s: cfg.dt_s,
    };
    let log = simulate(
        &car,
        x0.clone(),
        &cfg,
        |t, x| {
            let (xr, ur) = bicycle_ref(&car, &sample_traj(&reference, t));
            lqr.control(&car, x, &xr, &ur)
        },
        no_d,
        never,
    );
    runs.push(Run {
        name: "LQR".into(),
        metrics: metrics(&log),
        log,
    });

    let dt_mpc = 0.1;
    let mut mpc = LtvMpc::new(
        DMatrix::from_diagonal(&v(&[10.0, 10.0, 20.0, 1.0, 0.5])),
        DMatrix::from_diagonal(&v(&[0.5, 2.0])),
        DMatrix::from_diagonal(&v(&[40.0, 40.0, 60.0, 2.0, 1.0])),
        20,
        dt_mpc,
    );
    let log = simulate(
        &car,
        x0,
        &cfg,
        |t, x| {
            let refs: Vec<_> = (0..=mpc.horizon)
                .map(|k| bicycle_ref(&car, &sample_traj(&reference, t + k as f64 * dt_mpc)))
                .collect();
            mpc.control(&car, x, &refs)
        },
        no_d,
        never,
    );
    runs.push(Run {
        name: "LTV-MPC".into(),
        metrics: metrics(&log),
        log,
    });
    let _ = car.nx();
    ParkingResult {
        map,
        footprint,
        reference,
        goal,
        runs,
    }
}
