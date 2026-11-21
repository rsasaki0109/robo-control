//! Cart-pole swing-up: energy shaping with a switch to LQR balance.

use crate::lqr::dlqr;
use crate::model::{linearize, v, wrap, Dynamics};
use crate::models::CartPole;
use nalgebra::{DMatrix, DVector};

/// Energy-shaping swing-up (Åström-Furuta) that hands over to an LQR near upright.
#[derive(Debug, Clone)]
pub struct EnergySwingUp {
    /// plant model used by the controller
    pub model: CartPole,
    /// energy gain
    pub k_energy: f64,
    /// cart centring stiffness
    pub k_pos: f64,
    /// cart centring damping
    pub k_vel: f64,
    /// |theta| below which LQR takes over [rad]
    pub switch_angle_rad: f64,
    /// balance gain
    pub k_lqr: DMatrix<f64>,
}

impl EnergySwingUp {
    /// Builds the controller with an LQR designed at the upright equilibrium.
    pub fn new(model: CartPole, dt_s: f64) -> Self {
        let (a, b) = linearize(&model, &DVector::zeros(4), &DVector::zeros(1), dt_s);
        let q = DMatrix::from_diagonal(&v(&[5.0, 1.0, 50.0, 5.0]));
        let k_lqr = dlqr(&a, &b, &q, &DMatrix::from_element(1, 1, 0.05));
        Self {
            model,
            k_energy: 6.0,
            k_pos: 1.5,
            k_vel: 1.5,
            switch_angle_rad: 0.35,
            k_lqr,
        }
    }
    /// Whether the balance controller is active for this state.
    pub fn balancing(&self, x: &DVector<f64>) -> bool {
        wrap(x[2]).abs() < self.switch_angle_rad && x[3].abs() < 3.0
    }
    /// Control law.
    pub fn control(&self, x: &DVector<f64>) -> DVector<f64> {
        if self.balancing(x) {
            let mut e = x.clone();
            e[2] = wrap(e[2]);
            return self.model.clamp_input(&(-&self.k_lqr * e));
        }
        let e = self.model.energy(x);
        let kick = if x[3].abs() < 1e-3 && e < -1e-6 { 1.0 } else { 0.0 };
        let pdd = self.k_energy * e * x[3] * x[2].cos() - self.k_pos * x[0] - self.k_vel * x[1] + kick;
        self.model
            .clamp_input(&DVector::from_element(1, self.model.force_for_accel(x, pdd)))
    }
}
