//! Fixed-step closed-loop simulation with logging, disturbances and seeded noise.

use crate::model::{step, Dynamics};
use nalgebra::DVector;
use robo_planning::rng::Rng;

/// Simulation settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimConfig {
    /// integration and control period [s]
    pub dt_s: f64,
    /// duration [s]
    pub duration_s: f64,
    /// std-dev of additive state-measurement noise (applied to what the controller sees)
    pub measurement_noise: f64,
    /// RNG seed for noise
    pub seed: u64,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            dt_s: 0.02,
            duration_s: 10.0,
            measurement_noise: 0.0,
            seed: 0,
        }
    }
}

/// Recorded closed-loop run.
#[derive(Debug, Clone, Default)]
pub struct Log {
    /// time stamps [s]
    pub t: Vec<f64>,
    /// true states
    pub x: Vec<DVector<f64>>,
    /// applied (clamped) inputs
    pub u: Vec<DVector<f64>>,
}

impl Log {
    /// Column `i` of the state history.
    pub fn state(&self, i: usize) -> Vec<f64> {
        self.x.iter().map(|x| x[i]).collect()
    }
    /// Column `i` of the input history.
    pub fn input(&self, i: usize) -> Vec<f64> {
        self.u.iter().map(|u| u[i]).collect()
    }
}

/// Runs a closed loop. `controller(t, x_measured)` returns the input, `disturbance(t, x)`
/// is added to the state derivative (as a separate Euler term), and `stop(t, x)` ends
/// the run early when it returns true.
pub fn simulate<M: Dynamics>(
    model: &M,
    x0: DVector<f64>,
    cfg: &SimConfig,
    mut controller: impl FnMut(f64, &DVector<f64>) -> DVector<f64>,
    disturbance: impl Fn(f64, &DVector<f64>) -> DVector<f64>,
    stop: impl Fn(f64, &DVector<f64>) -> bool,
) -> Log {
    let mut rng = Rng::new(cfg.seed);
    let mut x = x0;
    let mut log = Log::default();
    let n = (cfg.duration_s / cfg.dt_s).round() as usize;
    for k in 0..n {
        let t = k as f64 * cfg.dt_s;
        let meas = if cfg.measurement_noise > 0.0 {
            x.map(|v| v + cfg.measurement_noise * rng.normal())
        } else {
            x.clone()
        };
        let u = model.clamp_input(&controller(t, &meas));
        log.t.push(t);
        log.x.push(x.clone());
        log.u.push(u.clone());
        if stop(t, &x) {
            break;
        }
        x = step(model, &x, &u, cfg.dt_s) + disturbance(t, &x) * cfg.dt_s;
    }
    log
}

/// No disturbance.
pub fn no_disturbance(nx: usize) -> impl Fn(f64, &DVector<f64>) -> DVector<f64> {
    move |_, _| DVector::zeros(nx)
}
