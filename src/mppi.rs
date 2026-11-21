//! Model Predictive Path Integral control (MPPI, Williams et al. 2017):
//! sampling-based MPC for any [`Dynamics`] and any (even non-smooth) cost.
//!
//! Each call perturbs the nominal input sequence with Gaussian noise, rolls
//! every sample through the model, and moves the nominal sequence towards the
//! samples weighted by `exp(-cost / lambda)`. No derivatives are needed, so the
//! cost can contain obstacles, contact or saturation terms as they are.

use crate::model::{step, Dynamics};
use nalgebra::DVector;
use robo_planning::rng::Rng;

/// MPPI controller.
#[derive(Debug, Clone)]
pub struct Mppi {
    /// horizon [steps]
    pub horizon: usize,
    /// model step [s]
    pub dt_s: f64,
    /// rollouts per call
    pub samples: usize,
    /// temperature: lower follows the best sample more greedily
    pub lambda: f64,
    /// per-input noise standard deviation
    pub sigma: DVector<f64>,
    rng: Rng,
    nominal: Vec<DVector<f64>>,
    /// best-weighted predicted trajectory of the last call (for plotting)
    pub predicted: Vec<DVector<f64>>,
}

impl Mppi {
    /// New controller with a zero nominal input sequence.
    pub fn new(
        nu: usize,
        horizon: usize,
        dt_s: f64,
        samples: usize,
        lambda: f64,
        sigma: DVector<f64>,
        seed: u64,
    ) -> Self {
        assert_eq!(sigma.len(), nu);
        Self {
            horizon,
            dt_s,
            samples,
            lambda,
            sigma,
            rng: Rng::new(seed),
            nominal: vec![DVector::zeros(nu); horizon],
            predicted: vec![],
        }
    }

    /// Sets the nominal input sequence (e.g. a feed-forward guess).
    pub fn set_nominal(&mut self, us: Vec<DVector<f64>>) {
        assert_eq!(us.len(), self.horizon);
        self.nominal = us;
    }

    /// Computes the input at state `x`. `stage(k, x, u)` is the running cost of
    /// step `k`, `terminal(x)` the cost of the final state.
    pub fn control<M: Dynamics>(
        &mut self,
        m: &M,
        x: &DVector<f64>,
        stage: impl Fn(usize, &DVector<f64>, &DVector<f64>) -> f64,
        terminal: impl Fn(&DVector<f64>) -> f64,
    ) -> DVector<f64> {
        let (n, nu) = (self.horizon, m.nu());
        let mut costs = Vec::with_capacity(self.samples);
        let mut noises: Vec<Vec<DVector<f64>>> = Vec::with_capacity(self.samples);
        for s in 0..self.samples {
            let mut xk = x.clone();
            let mut cost = 0.0;
            let mut eps_seq = Vec::with_capacity(n);
            for k in 0..n {
                // sample 0 keeps the nominal sequence itself
                let noise = if s == 0 {
                    DVector::zeros(nu)
                } else {
                    DVector::from_fn(nu, |i, _| self.sigma[i] * self.rng.normal())
                };
                let u = m.clamp_input(&(&self.nominal[k] + &noise));
                let eps = &u - &self.nominal[k];
                cost += stage(k, &xk, &u);
                xk = step(m, &xk, &u, self.dt_s);
                eps_seq.push(eps);
            }
            cost += terminal(&xk);
            costs.push(if cost.is_finite() { cost } else { f64::MAX / 4.0 });
            noises.push(eps_seq);
        }
        let min = costs.iter().copied().fold(f64::INFINITY, f64::min);
        let weights: Vec<f64> = costs.iter().map(|c| (-(c - min) / self.lambda).exp()).collect();
        let total: f64 = weights.iter().sum();
        for k in 0..n {
            let mut du = DVector::zeros(nu);
            for (w, eps) in weights.iter().zip(&noises) {
                du += &eps[k] * (*w / total);
            }
            self.nominal[k] = m.clamp_input(&(&self.nominal[k] + du));
        }
        // predicted trajectory of the updated nominal sequence
        let mut xk = x.clone();
        self.predicted = vec![xk.clone()];
        for k in 0..n {
            xk = step(m, &xk, &self.nominal[k], self.dt_s);
            self.predicted.push(xk.clone());
        }
        let u0 = self.nominal[0].clone();
        self.nominal.rotate_left(1);
        let last = self.nominal[n - 2].clone();
        self.nominal[n - 1] = last;
        u0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::v;

    #[derive(Debug)]
    struct DoubleIntegrator;
    impl Dynamics for DoubleIntegrator {
        fn nx(&self) -> usize {
            2
        }
        fn nu(&self) -> usize {
            1
        }
        fn deriv(&self, x: &DVector<f64>, u: &DVector<f64>) -> DVector<f64> {
            v(&[x[1], u[0]])
        }
        fn u_min(&self) -> DVector<f64> {
            v(&[-2.0])
        }
        fn u_max(&self) -> DVector<f64> {
            v(&[2.0])
        }
    }

    #[test]
    fn drives_double_integrator_to_origin_deterministically() {
        let run = || {
            let m = DoubleIntegrator;
            let mut c = Mppi::new(1, 20, 0.05, 200, 1.0, v(&[1.0]), 7);
            let mut x = v(&[1.0, 0.0]);
            for _ in 0..200 {
                let u = c.control(
                    &m,
                    &x,
                    |_, x, u| 10.0 * x[0] * x[0] + x[1] * x[1] + 0.01 * u[0] * u[0],
                    |x| 100.0 * x.norm_squared(),
                );
                x = step(&m, &x, &u, 0.05);
            }
            x
        };
        let x = run();
        assert!(x.norm() < 0.1, "{x:?}");
        assert_eq!(x, run());
    }
}
