use crate::model::{wrap, Dynamics};
use nalgebra::DVector;

/// Planar (x-z) quadrotor with two thrust inputs.
///
/// State `[x, z, theta, vx, vz, theta_dot]`, input `[f_left, f_right]` (N, non-negative).
/// Positive `theta` tilts thrust towards `-x`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quadrotor2D {
    /// mass [kg]
    pub mass_kg: f64,
    /// pitch inertia [kg m^2]
    pub inertia_kgm2: f64,
    /// rotor arm [m]
    pub arm_m: f64,
    /// max thrust per rotor [N]
    pub thrust_max_n: f64,
    /// linear drag [N s/m]
    pub drag: f64,
    /// gravity [m/s^2]
    pub gravity_mps2: f64,
}

impl Default for Quadrotor2D {
    fn default() -> Self {
        Self {
            mass_kg: 1.0,
            inertia_kgm2: 0.01,
            arm_m: 0.2,
            thrust_max_n: 12.0,
            drag: 0.1,
            gravity_mps2: 9.81,
        }
    }
}

impl Quadrotor2D {
    /// Per-rotor hover thrust [N].
    pub fn hover_thrust_n(&self) -> f64 {
        self.mass_kg * self.gravity_mps2 / 2.0
    }
}

impl Dynamics for Quadrotor2D {
    fn nx(&self) -> usize {
        6
    }
    fn nu(&self) -> usize {
        2
    }
    fn deriv(&self, x: &DVector<f64>, u: &DVector<f64>) -> DVector<f64> {
        let t = u[0] + u[1];
        DVector::from_vec(vec![
            x[3],
            x[4],
            x[5],
            (-t * x[2].sin() - self.drag * x[3]) / self.mass_kg,
            (t * x[2].cos() - self.drag * x[4]) / self.mass_kg - self.gravity_mps2,
            (u[1] - u[0]) * self.arm_m / self.inertia_kgm2,
        ])
    }
    fn u_min(&self) -> DVector<f64> {
        DVector::zeros(2)
    }
    fn u_max(&self) -> DVector<f64> {
        DVector::from_element(2, self.thrust_max_n)
    }
    fn state_error(&self, x: &DVector<f64>, r: &DVector<f64>) -> DVector<f64> {
        let mut e = x - r;
        e[2] = wrap(e[2]);
        e
    }
}
