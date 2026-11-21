//! Wheeled robots: a differential-drive AMR in a warehouse and a car on a fast course.

use super::Run;
use crate::controllers::path_tracking::{Kanayama, PurePursuit, RefPath, Stanley};
use crate::controllers::pid::Pid;
use crate::lqr::TrackingLqr;
use crate::metrics::{distance_to_polyline, max_abs, rms};
use crate::model::v;
use crate::models::{Bicycle, Unicycle};
use crate::mpc::LtvMpc;
use crate::sim::{simulate, SimConfig};
use nalgebra::{DMatrix, DVector};
use robo_planning::trajectory::{densify, sample_traj, spline_path, time_parameterize, SpeedLimits, TrajPoint};
use robo_planning::{grid, scenes, Map2D, Vec2};

/// Scenario output: map, reference and one run per controller.
#[derive(Debug, Clone)]
pub struct WheeledResult {
    /// map
    pub map: Map2D,
    /// reference trajectory
    pub reference: Vec<TrajPoint>,
    /// runs
    pub runs: Vec<Run>,
    /// simulation step [s]
    pub dt_s: f64,
}

fn unicycle_ref(tp: &TrajPoint) -> (DVector<f64>, DVector<f64>) {
    let w = tp.v_mps * tp.p.curvature;
    (v(&[tp.p.x, tp.p.y, tp.p.yaw_rad, tp.v_mps, w]), v(&[tp.v_mps, w]))
}

fn bicycle_ref(car: &Bicycle, tp: &TrajPoint) -> (DVector<f64>, DVector<f64>) {
    let d = (car.wheelbase_m * tp.p.curvature).atan();
    (v(&[tp.p.x, tp.p.y, tp.p.yaw_rad, tp.v_mps, d]), v(&[tp.a_mps2, d]))
}

fn path_metrics(log: &crate::sim::Log, reference: &[TrajPoint], radius: f64, map: &Map2D) -> Vec<(String, f64)> {
    let line: Vec<Vec2> = reference.iter().map(|p| Vec2::new(p.p.x, p.p.y)).collect();
    let cte: Vec<f64> = log
        .x
        .iter()
        .map(|x| distance_to_polyline(Vec2::new(x[0], x[1]), &line))
        .collect();
    let end = reference.last().unwrap();
    let last = log.x.last().unwrap();
    let clearance = log
        .x
        .iter()
        .map(|x| map.clearance(Vec2::new(x[0], x[1])))
        .fold(f64::INFINITY, f64::min)
        - radius;
    vec![
        ("rms_cte_m".into(), rms(&cte)),
        ("max_cte_m".into(), max_abs(&cte)),
        ("goal_err_m".into(), (last[0] - end.p.x).hypot(last[1] - end.p.y)),
        ("time_s".into(), *log.t.last().unwrap()),
        ("min_clearance_m".into(), clearance),
    ]
}

/// Differential-drive AMR: A* -> prune -> spline -> speed profile, tracked by
/// pure pursuit, Kanayama, LQR and LTV-MPC on a unicycle with velocity lag.
pub fn amr_warehouse() -> WheeledResult {
    let sc = scenes::warehouse();
    let radius = 0.35;
    let g = grid::OccupancyGrid::from_map(&sc.map, 0.1, radius + 0.1);
    let a = grid::astar(&g, sc.start.pos(), sc.goal.pos()).expect("path");
    let pruned = grid::prune_line_of_sight(&sc.map, &a.points, radius + 0.15);
    let path = spline_path(&densify(&pruned, 0.8), 0.05);
    let reference = time_parameterize(
        &path,
        &SpeedLimits {
            v_max_mps: 1.4,
            a_max_mps2: 0.8,
            a_lat_max_mps2: 0.8,
        },
    );
    let t_end = reference.last().unwrap().t_s;
    let m = Unicycle::default();
    let cfg = SimConfig {
        dt_s: 0.02,
        duration_s: t_end + 6.0,
        ..Default::default()
    };
    let x0 = v(&[sc.start.x, sc.start.y, reference[1].p.yaw_rad + 0.3, 0.0, 0.0]);
    let goal = sc.goal.pos();
    let stop = |t: f64, x: &DVector<f64>| t > t_end && Vec2::new(x[0], x[1]).dist(goal) < 0.05 && x[3].abs() < 0.02;
    let no_d = |_: f64, _: &DVector<f64>| DVector::zeros(5);
    let mut runs = Vec::new();

    let mut rp = RefPath::new(reference.clone());
    let pp = PurePursuit {
        lookahead_min_m: 0.5,
        lookahead_gain_s: 0.6,
    };
    let log = simulate(
        &m,
        x0.clone(),
        &cfg,
        |_, x| {
            let i = rp.nearest(x[0], x[1]);
            let vel = rp.speed(i, 0.25, 0.4);
            let k = pp.curvature(&rp, i, x[0], x[1], x[2], vel);
            v(&[vel, vel * k])
        },
        no_d,
        stop,
    );
    runs.push(Run {
        name: "pure pursuit".into(),
        metrics: path_metrics(&log, &reference, radius, &sc.map),
        log,
    });

    let kan = Kanayama {
        kx: 1.5,
        ky: 6.0,
        kth: 3.0,
    };
    let r2 = reference.clone();
    let log = simulate(
        &m,
        x0.clone(),
        &cfg,
        |t, x| {
            let (vv, w) = kan.control(x[0], x[1], x[2], &sample_traj(&r2, t));
            v(&[vv, w])
        },
        no_d,
        stop,
    );
    runs.push(Run {
        name: "Kanayama".into(),
        metrics: path_metrics(&log, &reference, radius, &sc.map),
        log,
    });

    let lqr = TrackingLqr {
        q: DMatrix::from_diagonal(&v(&[10.0, 10.0, 4.0, 0.5, 0.2])),
        r: DMatrix::from_diagonal(&v(&[1.0, 0.5])),
        horizon: 60,
        dt_s: cfg.dt_s,
    };
    let r3 = reference.clone();
    let log = simulate(
        &m,
        x0.clone(),
        &cfg,
        |t, x| {
            let (xr, ur) = unicycle_ref(&sample_traj(&r3, t));
            lqr.control(&m, x, &xr, &ur)
        },
        no_d,
        stop,
    );
    runs.push(Run {
        name: "LQR".into(),
        metrics: path_metrics(&log, &reference, radius, &sc.map),
        log,
    });

    let dt_mpc = 0.1;
    let mut mpc = LtvMpc::new(
        DMatrix::from_diagonal(&v(&[10.0, 10.0, 4.0, 0.5, 0.2])),
        DMatrix::from_diagonal(&v(&[0.5, 0.2])),
        DMatrix::from_diagonal(&v(&[30.0, 30.0, 10.0, 1.0, 0.5])),
        15,
        dt_mpc,
    );
    let r4 = reference.clone();
    let log = simulate(
        &m,
        x0,
        &cfg,
        |t, x| {
            let refs: Vec<_> = (0..=mpc.horizon)
                .map(|k| unicycle_ref(&sample_traj(&r4, t + k as f64 * dt_mpc)))
                .collect();
            mpc.control(&m, x, &refs)
        },
        no_d,
        stop,
    );
    runs.push(Run {
        name: "LTV-MPC".into(),
        metrics: path_metrics(&log, &reference, radius, &sc.map),
        log,
    });

    WheeledResult {
        map: sc.map,
        reference,
        runs,
        dt_s: cfg.dt_s,
    }
}

/// Fast car course: a spline circuit with hairpins driven at up to 12 m/s by a
/// bicycle with steering lag; pure pursuit, Stanley, LQR and LTV-MPC.
pub fn car_course() -> WheeledResult {
    let map = Map2D::new(Vec2::new(-5.0, -5.0), Vec2::new(85.0, 50.0))
        .circle(40.0, 22.0, 6.0)
        .rect(15.0, 15.0, 22.0, 30.0)
        .rect(58.0, 15.0, 65.0, 30.0);
    let wps = [
        Vec2::new(0.0, 0.0),
        Vec2::new(30.0, 0.0),
        Vec2::new(45.0, 8.0),
        Vec2::new(55.0, 2.0),
        Vec2::new(75.0, 5.0),
        Vec2::new(78.0, 25.0),
        Vec2::new(70.0, 40.0),
        Vec2::new(52.0, 38.0),
        Vec2::new(40.0, 32.0),
        Vec2::new(28.0, 40.0),
        Vec2::new(8.0, 40.0),
        Vec2::new(4.0, 22.0),
        Vec2::new(10.0, 10.0),
    ];
    let path = spline_path(&densify(&wps, 4.0), 0.25);
    let car = Bicycle::default();
    let reference = time_parameterize(
        &path,
        &SpeedLimits {
            v_max_mps: 12.0,
            a_max_mps2: 1.8,
            a_lat_max_mps2: 4.0,
        },
    );
    let t_end = reference.last().unwrap().t_s;
    let cfg = SimConfig {
        dt_s: 0.02,
        duration_s: t_end + 6.0,
        ..Default::default()
    };
    let x0 = v(&[0.0, 1.0, 0.1, 0.0, 0.0]);
    let goal = Vec2::new(path.last().unwrap().x, path.last().unwrap().y);
    let stop = |t: f64, x: &DVector<f64>| t > t_end && Vec2::new(x[0], x[1]).dist(goal) < 0.3 && x[3].abs() < 0.05;
    let no_d = |_: f64, _: &DVector<f64>| DVector::zeros(5);
    let mut runs = Vec::new();
    let speed_pid = || Pid::new(1.5, 0.2, 0.0, 5.0);

    let mut rp = RefPath::new(reference.clone());
    let pp = PurePursuit {
        lookahead_min_m: 3.0,
        lookahead_gain_s: 0.5,
    };
    let mut pid = speed_pid();
    let log = simulate(
        &car,
        x0.clone(),
        &cfg,
        |_, x| {
            let i = rp.nearest(x[0], x[1]);
            let vel = rp.speed(i, 1.0, 1.5);
            let k = pp.curvature(&rp, i, x[0], x[1], x[2], x[3]);
            v(&[pid.update(vel - x[3], cfg.dt_s), (car.wheelbase_m * k).atan()])
        },
        no_d,
        stop,
    );
    runs.push(Run {
        name: "pure pursuit".into(),
        metrics: path_metrics(&log, &reference, 0.0, &map),
        log,
    });

    let mut rp = RefPath::new(reference.clone());
    let mut rp_rear = RefPath::new(reference.clone());
    let st = Stanley {
        k: 1.5,
        k_soft_mps: 1.0,
    };
    let mut pid = speed_pid();
    let log = simulate(
        &car,
        x0.clone(),
        &cfg,
        |_, x| {
            let (fx, fy) = (x[0] + car.wheelbase_m * x[2].cos(), x[1] + car.wheelbase_m * x[2].sin());
            let delta = st.steer(&mut rp, fx, fy, x[2], x[3]);
            let i = rp_rear.nearest(x[0], x[1]);
            let vel = rp_rear.speed(i, 1.0, 1.5);
            v(&[pid.update(vel - x[3], cfg.dt_s), delta])
        },
        no_d,
        stop,
    );
    runs.push(Run {
        name: "Stanley".into(),
        metrics: path_metrics(&log, &reference, 0.0, &map),
        log,
    });

    let lqr = TrackingLqr {
        q: DMatrix::from_diagonal(&v(&[5.0, 5.0, 20.0, 1.0, 0.5])),
        r: DMatrix::from_diagonal(&v(&[1.0, 2.0])),
        horizon: 150,
        dt_s: cfg.dt_s,
    };
    let r3 = reference.clone();
    let log = simulate(
        &car,
        x0.clone(),
        &cfg,
        |t, x| {
            let (xr, ur) = bicycle_ref(&car, &sample_traj(&r3, t));
            lqr.control(&car, x, &xr, &ur)
        },
        no_d,
        stop,
    );
    runs.push(Run {
        name: "LQR".into(),
        metrics: path_metrics(&log, &reference, 0.0, &map),
        log,
    });

    let dt_mpc = 0.1;
    let mut mpc = LtvMpc::new(
        DMatrix::from_diagonal(&v(&[5.0, 5.0, 20.0, 1.0, 0.5])),
        DMatrix::from_diagonal(&v(&[0.5, 2.0])),
        DMatrix::from_diagonal(&v(&[20.0, 20.0, 60.0, 2.0, 1.0])),
        20,
        dt_mpc,
    );
    let r4 = reference.clone();
    let log = simulate(
        &car,
        x0,
        &cfg,
        |t, x| {
            let refs: Vec<_> = (0..=mpc.horizon)
                .map(|k| bicycle_ref(&car, &sample_traj(&r4, t + k as f64 * dt_mpc)))
                .collect();
            mpc.control(&car, x, &refs)
        },
        no_d,
        stop,
    );
    runs.push(Run {
        name: "LTV-MPC".into(),
        metrics: path_metrics(&log, &reference, 0.0, &map),
        log,
    });

    WheeledResult {
        map,
        reference,
        runs,
        dt_s: cfg.dt_s,
    }
}
