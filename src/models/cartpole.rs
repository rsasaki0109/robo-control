use crate::model::{wrap, Dynamics};
use nalgebra::DVector;

/// Cart with a point-mass pendulum. `theta = 0` is upright.
///
/// State `[p, p_dot, theta, theta_dot]`, input `[force]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartPole {
    /// cart mass [kg]
    pub cart_mass_kg: f64,
    /// pendulum mass [kg]
    pub pole_mass_kg: f64,
    /// pendulum length [m]
    pub pole_length_m: f64,
    /// cart viscous friction [N s/m]
    pub cart_friction: f64,
    /// force limit [N]
    pub force_max_n: f64,
    /// gravity [m/s^2]
    pub gravity_mps2: f64,
}

impl Default for CartPole {
    fn default() -> Self {
        Self {
            cart_mass_kg: 1.0,
            pole_mass_kg: 0.3,
            pole_length_m: 0.5,
            cart_friction: 0.1,
            force_max_n: 20.0,
            gravity_mps2: 9.81,
        }
    }
}

impl CartPole {
    /// Pendulum energy relative to the upright rest state (0 when balanced).
    pub fn energy(&self, x: &DVector<f64>) -> f64 {
        let (m, l, g) = (self.pole_mass_kg, self.pole_length_m, self.gravity_mps2);
        0.5 * m * l * l * x[3] * x[3] + m * g * l * (x[2].cos() - 1.0)
    }
    /// Force that produces cart acceleration `pdd` (partial feedback linearisation).
    pub fn force_for_accel(&self, x: &DVector<f64>, pdd: f64) -> f64 {
        let (mc, m, l, g) = (
            self.cart_mass_kg,
            self.pole_mass_kg,
            self.pole_length_m,
            self.gravity_mps2,
        );
        let (s, c) = (x[2].sin(), x[2].cos());
        (mc + m - m * c * c) * pdd + m * g * s * c - m * l * s * x[3] * x[3] + self.cart_friction * x[1]
    }
}

impl Dynamics for CartPole {
    fn nx(&self) -> usize {
        4
    }
    fn nu(&self) -> usize {
        1
    }
    fn deriv(&self, x: &DVector<f64>, u: &DVector<f64>) -> DVector<f64> {
        let (mc, m, l, g) = (
            self.cart_mass_kg,
            self.pole_mass_kg,
            self.pole_length_m,
            self.gravity_mps2,
        );
        let (s, c) = (x[2].sin(), x[2].cos());
        let f = u[0] - self.cart_friction * x[1];
        // [mc+m, m l c; m l c, m l^2] [pdd; thdd] = [f + m l s thd^2; m g l s]
        let (a11, a12, a22) = (mc + m, m * l * c, m * l * l);
        let (b1, b2) = (f + m * l * s * x[3] * x[3], m * g * l * s);
        let det = a11 * a22 - a12 * a12;
        let pdd = (a22 * b1 - a12 * b2) / det;
        let thdd = (-a12 * b1 + a11 * b2) / det;
        DVector::from_vec(vec![x[1], pdd, x[3], thdd])
    }
    fn u_min(&self) -> DVector<f64> {
        DVector::from_element(1, -self.force_max_n)
    }
    fn u_max(&self) -> DVector<f64> {
        DVector::from_element(1, self.force_max_n)
    }
    fn state_error(&self, x: &DVector<f64>, r: &DVector<f64>) -> DVector<f64> {
        let mut e = x - r;
        e[2] = wrap(e[2]);
        e
    }
}
