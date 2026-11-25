use crate::model::{wrap, Dynamics};
use nalgebra::DVector;

/// Planar robot with independent first-order body velocity and yaw-rate loops.
///
/// State `[x, y, theta, vx_body, vy_body, omega]`, input `[vx_cmd, vy_cmd, omega_cmd]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Holonomic {
    /// Translational velocity loop time constant [s].
    pub tau_v_s: f64,
    /// Yaw-rate loop time constant [s].
    pub tau_w_s: f64,
    /// Maximum forward speed [m/s].
    pub forward_max_mps: f64,
    /// Maximum reverse speed magnitude [m/s].
    pub reverse_max_mps: f64,
    /// Maximum lateral speed magnitude [m/s].
    pub lateral_max_mps: f64,
    /// Maximum yaw-rate magnitude [rad/s].
    pub w_max_radps: f64,
}

impl Default for Holonomic {
    fn default() -> Self {
        let unicycle = super::Unicycle::default();
        Self {
            tau_v_s: unicycle.tau_v_s,
            tau_w_s: unicycle.tau_w_s,
            forward_max_mps: 0.7,
            reverse_max_mps: 0.3,
            lateral_max_mps: 0.3,
            w_max_radps: 1.2,
        }
    }
}

impl Dynamics for Holonomic {
    fn nx(&self) -> usize {
        6
    }

    fn nu(&self) -> usize {
        3
    }

    fn deriv(&self, x: &DVector<f64>, u: &DVector<f64>) -> DVector<f64> {
        let (s, c) = x[2].sin_cos();
        DVector::from_vec(vec![
            c * x[3] - s * x[4],
            s * x[3] + c * x[4],
            x[5],
            (u[0] - x[3]) / self.tau_v_s,
            (u[1] - x[4]) / self.tau_v_s,
            (u[2] - x[5]) / self.tau_w_s,
        ])
    }

    fn u_min(&self) -> DVector<f64> {
        DVector::from_vec(vec![-self.reverse_max_mps, -self.lateral_max_mps, -self.w_max_radps])
    }

    fn u_max(&self) -> DVector<f64> {
        DVector::from_vec(vec![self.forward_max_mps, self.lateral_max_mps, self.w_max_radps])
    }

    fn state_error(&self, x: &DVector<f64>, r: &DVector<f64>) -> DVector<f64> {
        let mut error = x - r;
        error[2] = wrap(error[2]);
        error
    }
}
