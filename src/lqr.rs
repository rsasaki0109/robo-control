//! Linear-quadratic regulators: discrete Riccati recursion, infinite-horizon DLQR
//! and a generic "linearise-at-the-reference" tracking LQR for any [`Dynamics`].

use crate::model::{linearize, Dynamics};
use nalgebra::{DMatrix, DVector};

/// Backward Riccati recursion over `horizon` steps for constant `(A, B)`.
/// Returns the first-step feedback gain `K` (u = -K x) and cost-to-go `P`.
pub fn riccati(
    a: &DMatrix<f64>,
    b: &DMatrix<f64>,
    q: &DMatrix<f64>,
    r: &DMatrix<f64>,
    qf: &DMatrix<f64>,
    horizon: usize,
) -> (DMatrix<f64>, DMatrix<f64>) {
    let mut p = qf.clone();
    let mut k = DMatrix::zeros(b.ncols(), a.nrows());
    for _ in 0..horizon {
        let bt_p = b.transpose() * &p;
        let s = r + &bt_p * b;
        k = s
            .clone()
            .try_inverse()
            .unwrap_or_else(|| s.pseudo_inverse(1e-9).unwrap())
            * &bt_p
            * a;
        p = q + a.transpose() * &p * a - a.transpose() * &p * b * &k;
        p = (&p + p.transpose()) * 0.5;
    }
    (k, p)
}

/// Infinite-horizon discrete LQR by iterating the Riccati equation to convergence.
pub fn dlqr(a: &DMatrix<f64>, b: &DMatrix<f64>, q: &DMatrix<f64>, r: &DMatrix<f64>) -> DMatrix<f64> {
    let mut p = q.clone();
    let mut k = DMatrix::zeros(b.ncols(), a.nrows());
    for _ in 0..10_000 {
        let bt_p = b.transpose() * &p;
        let s = r + &bt_p * b;
        k = s.try_inverse().expect("singular R + B'PB") * &bt_p * a;
        let pn = q + a.transpose() * &p * a - a.transpose() * &p * b * &k;
        let diff = (&pn - &p).abs().max();
        p = (&pn + pn.transpose()) * 0.5;
        if diff < 1e-10 {
            break;
        }
    }
    k
}

/// Tracking LQR: at each call it linearises the model at the reference state
/// and input, solves a finite-horizon Riccati recursion and applies
/// `u = u_ref - K (x - x_ref)`. Works for any model; a finite horizon keeps it
/// well defined where the linearisation is not stabilisable (e.g. at zero speed).
#[derive(Debug, Clone)]
pub struct TrackingLqr {
    /// state weight
    pub q: DMatrix<f64>,
    /// input weight
    pub r: DMatrix<f64>,
    /// Riccati horizon [steps]
    pub horizon: usize,
    /// controller period [s]
    pub dt_s: f64,
}

impl TrackingLqr {
    /// Control law.
    pub fn control<M: Dynamics>(
        &self,
        m: &M,
        x: &DVector<f64>,
        x_ref: &DVector<f64>,
        u_ref: &DVector<f64>,
    ) -> DVector<f64> {
        let (a, b) = linearize(m, x_ref, u_ref, self.dt_s);
        let (k, _) = riccati(&a, &b, &self.q, &self.r, &(self.q.clone() * 10.0), self.horizon);
        m.clamp_input(&(u_ref - k * m.state_error(x, x_ref)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dlqr_stabilises_double_integrator() {
        let dt = 0.1;
        let a = DMatrix::from_row_slice(2, 2, &[1.0, dt, 0.0, 1.0]);
        let b = DMatrix::from_row_slice(2, 1, &[0.5 * dt * dt, dt]);
        let k = dlqr(&a, &b, &DMatrix::identity(2, 2), &DMatrix::identity(1, 1));
        let cl = &a - &b * &k;
        let eig = cl.complex_eigenvalues();
        assert!(eig.iter().all(|e| e.norm() < 1.0));
        let (kf, _) = riccati(
            &a,
            &b,
            &DMatrix::identity(2, 2),
            &DMatrix::identity(1, 1),
            &DMatrix::identity(2, 2),
            500,
        );
        assert!((&k - kf).abs().max() < 1e-6);
    }
}
