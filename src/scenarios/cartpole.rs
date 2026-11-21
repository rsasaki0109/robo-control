//! Cart-pole (inverted pendulum / balancing robot): swing-up from hanging.

use super::Run;
use crate::controllers::cartpole::EnergySwingUp;
use crate::ilqr::{solve, IlqrCost};
use crate::model::{v, wrap};
use crate::models::CartPole;
use crate::sim::{simulate, SimConfig};
use nalgebra::{DMatrix, DVector};

/// Cart-pole output.
#[derive(Debug, Clone)]
pub struct CartPoleResult {
    /// model
    pub model: CartPole,
    /// runs
    pub runs: Vec<Run>,
    /// push time [s]
    pub push_t_s: f64,
}

/// Energy-shaping + LQR versus iLQR trajectory optimisation with TV-LQR
/// feedback (then LQR), both from hanging, with a push once balanced.
pub fn swing_up() -> CartPoleResult {
    let model = CartPole::default();
    let dt = 0.01;
    let cfg = SimConfig {
        dt_s: dt,
        duration_s: 8.0,
        ..Default::default()
    };
    let x0 = v(&[0.0, 0.0, std::f64::consts::PI, 0.0]);
    let push_t_s = 6.0;
    let push = move |t: f64, _: &DVector<f64>| {
        if (push_t_s..push_t_s + 0.1).contains(&t) {
            v(&[0.0, 0.0, 0.0, 8.0])
        } else {
            DVector::zeros(4)
        }
    };
    let metrics = |log: &crate::sim::Log| -> Vec<(String, f64)> {
        let up = log
            .t
            .iter()
            .zip(&log.x)
            .find(|(_, x)| wrap(x[2]).abs() < 0.1 && x[3].abs() < 0.5)
            .map_or(f64::NAN, |(t, _)| *t);
        let max_cart = log.x.iter().fold(0.0f64, |m, x| m.max(x[0].abs()));
        let last = log.x.last().unwrap();
        vec![
            ("time_to_upright_s".into(), up),
            ("max_cart_travel_m".into(), max_cart),
            ("final_angle_rad".into(), wrap(last[2]).abs()),
            ("effort_n2s".into(), crate::metrics::effort(&log.u, dt)),
        ]
    };
    let mut runs = Vec::new();
    let es = EnergySwingUp::new(model, dt);
    let log = simulate(&model, x0.clone(), &cfg, |_, x| es.control(x), push, |_, _| false);
    runs.push(Run {
        name: "energy + LQR".into(),
        metrics: metrics(&log),
        log,
    });

    // iLQR: 2.5 s horizon
    let n = 250;
    let cost = IlqrCost {
        goal: DVector::zeros(4),
        q: DMatrix::from_diagonal(&v(&[0.5, 0.05, 1.0, 0.05])),
        r: DMatrix::from_element(1, 1, 0.002),
        qf: DMatrix::from_diagonal(&v(&[200.0, 50.0, 1000.0, 100.0])),
    };
    let us0: Vec<DVector<f64>> = (0..n).map(|k| v(&[if k < 30 { 8.0 } else { 0.0 }])).collect();
    let sol = solve(&model, &x0, us0, &cost, dt, 200);
    let log = simulate(
        &model,
        x0,
        &cfg,
        |t, x| {
            let k = (t / dt).round() as usize;
            if k < n && !es.balancing(x) {
                let dx = model_err(x, &sol.xs[k]);
                &sol.us[k] + &sol.gains[k] * dx
            } else {
                es.control(x)
            }
        },
        push,
        |_, _| false,
    );
    runs.push(Run {
        name: "iLQR + TV-LQR".into(),
        metrics: metrics(&log),
        log,
    });
    CartPoleResult { model, runs, push_t_s }
}

fn model_err(x: &DVector<f64>, r: &DVector<f64>) -> DVector<f64> {
    let mut e = x - r;
    e[2] = wrap(e[2]);
    e
}
