use crate::model::{wrap, Dynamics};
use nalgebra::DVector;

/// Kinematic bicycle (rear-axle reference) with a first-order steering actuator.
///
/// State `[x, y, yaw, v, delta]`, input `[accel, delta_cmd]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bicycle {
    /// wheelbase [m]
    pub wheelbase_m: f64,
    /// steering actuator time constant [s]
    pub tau_steer_s: f64,
    /// steering limit [rad]
    pub steer_max_rad: f64,
    /// max acceleration [m/s^2]
    pub accel_max_mps2: f64,
    /// max braking [m/s^2]
    pub brake_max_mps2: f64,
}

impl Default for Bicycle {
    fn default() -> Self {
        Self {
            wheelbase_m: 2.7,
            tau_steer_s: 0.2,
            steer_max_rad: 0.6,
            accel_max_mps2: 2.0,
            brake_max_mps2: 4.0,
        }
    }
}

impl Dynamics for Bicycle {
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
            x[3] * x[4].tan() / self.wheelbase_m,
            u[0],
            (u[1] - x[4]) / self.tau_steer_s,
        ])
    }
    fn u_min(&self) -> DVector<f64> {
        DVector::from_vec(vec![-self.brake_max_mps2, -self.steer_max_rad])
    }
    fn u_max(&self) -> DVector<f64> {
        DVector::from_vec(vec![self.accel_max_mps2, self.steer_max_rad])
    }
    fn state_error(&self, x: &DVector<f64>, r: &DVector<f64>) -> DVector<f64> {
        let mut e = x - r;
        e[2] = wrap(e[2]);
        e
    }
}
