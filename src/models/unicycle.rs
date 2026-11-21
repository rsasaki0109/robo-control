use crate::model::{wrap, Dynamics};
use nalgebra::DVector;

/// Differential-drive robot as a unicycle with first-order velocity loops.
///
/// State `[x, y, theta, v, w]`, input `[v_cmd, w_cmd]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Unicycle {
    /// speed loop time constant [s]
    pub tau_v_s: f64,
    /// yaw-rate loop time constant [s]
    pub tau_w_s: f64,
    /// speed limit [m/s]
    pub v_max_mps: f64,
    /// yaw-rate limit [rad/s]
    pub w_max_radps: f64,
}

impl Default for Unicycle {
    fn default() -> Self {
        Self {
            tau_v_s: 0.15,
            tau_w_s: 0.08,
            v_max_mps: 1.5,
            w_max_radps: 2.0,
        }
    }
}

impl Dynamics for Unicycle {
    fn nx(&self) -> usize {
        5
    }
    fn nu(&self) -> usize {
        2
    }
    fn deriv(&self, x: &DVector<f64>, u: &DVector<f64>) -> DVector<f64> {
        DVector::from_vec(vec![
            x[3] * x[2].cos(),
            x[3] * x[2].sin(),
            x[4],
            (u[0] - x[3]) / self.tau_v_s,
            (u[1] - x[4]) / self.tau_w_s,
        ])
    }
    fn u_min(&self) -> DVector<f64> {
        DVector::from_vec(vec![-self.v_max_mps, -self.w_max_radps])
    }
    fn u_max(&self) -> DVector<f64> {
        DVector::from_vec(vec![self.v_max_mps, self.w_max_radps])
    }
    fn state_error(&self, x: &DVector<f64>, r: &DVector<f64>) -> DVector<f64> {
        let mut e = x - r;
        e[2] = wrap(e[2]);
        e
    }
}
