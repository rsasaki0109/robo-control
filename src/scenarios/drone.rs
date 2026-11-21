//! Planar quadrotor flying an RRT* path through a forest with a wind gust.

use super::Run;
use crate::controllers::quadrotor::{flat_reference, CascadedPid};
use crate::lqr::TrackingLqr;
use crate::metrics::{distance_to_polyline, max_abs, rms};
use crate::model::v;
use crate::models::Quadrotor2D;
use crate::mpc::LtvMpc;
use crate::sim::{simulate, SimConfig};
use nalgebra::{DMatrix, DVector};
use robo_planning::sampling::{rrt_star, shortcut, Point2Space, SamplingConfig};
use robo_planning::trajectory::{densify, sample_traj, spline_path, time_parameterize, SpeedLimits, TrajPoint};
use robo_planning::{scenes, Map2D, Vec2};

/// Drone scenario output.
#[derive(Debug, Clone)]
pub struct DroneResult {
    /// map (x horizontal, y = altitude)
    pub map: Map2D,
    /// reference
    pub reference: Vec<TrajPoint>,
    /// runs
    pub runs: Vec<Run>,
    /// gust window [s] and horizontal force [N]
    pub gust: (f64, f64, f64),
}

fn refs_at(q: &Quadrotor2D, r: &[TrajPoint], t: f64) -> (DVector<f64>, DVector<f64>) {
    let p = sample_traj(r, t);
    let (c, s) = (p.p.yaw_rad.cos(), p.p.yaw_rad.sin());
    let an = p.v_mps * p.v_mps * p.p.curvature;
    let acc = [p.a_mps2 * c - an * s, p.a_mps2 * s + an * c];
    flat_reference(q, [p.p.x, p.p.y], [p.v_mps * c, p.v_mps * s], acc)
}

/// RRT* + shortcut + spline reference, tracked by cascaded PID, LQR and LTV-MPC
/// while a sideways gust pushes the drone.
pub fn forest_flight() -> DroneResult {
    let sc = scenes::forest();
    let sp = Point2Space {
        map: &sc.map,
        margin_m: 0.45,
    };
    let r = rrt_star(
        &sp,
        &sc.start.pos(),
        &sc.goal.pos(),
        &SamplingConfig {
            step: 0.5,
            max_iters: 2500,
            goal_tolerance: 0.5,
            seed: 3,
            ..Default::default()
        },
        1.5,
    );
    let pts = shortcut(&sp, &r.path, 300, 5);
    let path = spline_path(&densify(&pts, 0.7), 0.05);
    let reference = time_parameterize(
        &path,
        &SpeedLimits {
            v_max_mps: 3.0,
            a_max_mps2: 2.0,
            a_lat_max_mps2: 3.0,
        },
    );
    let t_end = reference.last().unwrap().t_s;
    let q = Quadrotor2D::default();
    let cfg = SimConfig {
        dt_s: 0.01,
        duration_s: t_end + 2.0,
        ..Default::default()
    };
    let (x0, _) = refs_at(&q, &reference, 0.0);
    let gust = (t_end * 0.35, t_end * 0.35 + 1.0, 4.0);
    let wind = move |t: f64, _: &DVector<f64>| {
        let f = if t >= gust.0 && t < gust.1 { gust.2 } else { 0.0 };
        v(&[0.0, 0.0, 0.0, f / q.mass_kg, -0.5 * f / q.mass_kg, 0.0])
    };
    let line: Vec<Vec2> = reference.iter().map(|p| Vec2::new(p.p.x, p.p.y)).collect();
    let metrics = |log: &crate::sim::Log| -> Vec<(String, f64)> {
        let e: Vec<f64> = log
            .x
            .iter()
            .map(|x| distance_to_polyline(Vec2::new(x[0], x[1]), &line))
            .collect();
        let pos_err: Vec<f64> = log
            .t
            .iter()
            .zip(&log.x)
            .map(|(t, x)| {
                let p = sample_traj(&reference, *t);
                (x[0] - p.p.x).hypot(x[1] - p.p.y)
            })
            .collect();
        let clearance = log
            .x
            .iter()
            .map(|x| sc.map.clearance(Vec2::new(x[0], x[1])))
            .fold(f64::INFINITY, f64::min)
            - 0.25;
        vec![
            ("rms_path_err_m".into(), rms(&e)),
            ("max_path_err_m".into(), max_abs(&e)),
            ("rms_pos_err_m".into(), rms(&pos_err)),
            ("min_clearance_m".into(), clearance),
        ]
    };
    let mut runs = Vec::new();
    let mut pid = CascadedPid::new(q);
    let rf = reference.clone();
    let log = simulate(
        &q,
        x0.clone(),
        &cfg,
        |t, x| {
            let (xr, _) = refs_at(&q, &rf, t);
            let acc = {
                let pp = sample_traj(&rf, t);
                let (c, s) = (pp.p.yaw_rad.cos(), pp.p.yaw_rad.sin());
                let an = pp.v_mps * pp.v_mps * pp.p.curvature;
                [pp.a_mps2 * c - an * s, pp.a_mps2 * s + an * c]
            };
            pid.control(x, [xr[0], xr[1]], [xr[3], xr[4]], acc, cfg.dt_s)
        },
        wind,
        |_, _| false,
    );
    runs.push(Run {
        name: "cascaded PID".into(),
        metrics: metrics(&log),
        log,
    });

    let lqr = TrackingLqr {
        q: DMatrix::from_diagonal(&v(&[20.0, 20.0, 1.0, 2.0, 2.0, 0.1])),
        r: DMatrix::from_diagonal(&v(&[0.05, 0.05])),
        horizon: 200,
        dt_s: cfg.dt_s,
    };
    let rf = reference.clone();
    let log = simulate(
        &q,
        x0.clone(),
        &cfg,
        |t, x| {
            let (xr, ur) = refs_at(&q, &rf, t);
            lqr.control(&q, x, &xr, &ur)
        },
        wind,
        |_, _| false,
    );
    runs.push(Run {
        name: "LQR".into(),
        metrics: metrics(&log),
        log,
    });

    let dt_mpc = 0.05;
    let mut mpc = LtvMpc::new(
        DMatrix::from_diagonal(&v(&[20.0, 20.0, 1.0, 2.0, 2.0, 0.1])),
        DMatrix::from_diagonal(&v(&[0.02, 0.02])),
        DMatrix::from_diagonal(&v(&[60.0, 60.0, 2.0, 5.0, 5.0, 0.2])),
        20,
        dt_mpc,
    );
    mpc.iterations = 200;
    let rf = reference.clone();
    let log = simulate(
        &q,
        x0,
        &cfg,
        |t, x| {
            let refs: Vec<_> = (0..=mpc.horizon)
                .map(|k| refs_at(&q, &rf, t + k as f64 * dt_mpc))
                .collect();
            mpc.control(&q, x, &refs)
        },
        wind,
        |_, _| false,
    );
    runs.push(Run {
        name: "LTV-MPC".into(),
        metrics: metrics(&log),
        log,
    });
    DroneResult {
        map: sc.map,
        reference,
        runs,
        gust,
    }
}
