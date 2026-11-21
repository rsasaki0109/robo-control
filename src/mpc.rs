//! Linear time-varying MPC for any [`Dynamics`].
//!
//! The model is linearised along the reference over the horizon, the
//! input-constrained QP is condensed and solved with FISTA (accelerated
//! projected gradient) warm-started from the previous solution.

use crate::model::{linearize, step_raw, Dynamics};
use nalgebra::{DMatrix, DVector};

/// Solves `min 0.5 z'Hz + g'z  s.t. lo <= z <= hi` with FISTA (accelerated
/// projected gradient), starting from `z0`. `H` must be symmetric positive semidefinite.
pub fn box_qp(
    h: &DMatrix<f64>,
    g: &DVector<f64>,
    lo: &DVector<f64>,
    hi: &DVector<f64>,
    z0: DVector<f64>,
    iterations: usize,
) -> DVector<f64> {
    let project = |z: &mut DVector<f64>| {
        for i in 0..z.len() {
            z[i] = z[i].clamp(lo[i], hi[i]);
        }
    };
    // Lipschitz constant by Gershgorin
    let l = (0..h.nrows())
        .map(|i| h.row(i).iter().map(|v| v.abs()).sum::<f64>())
        .fold(1e-9, f64::max);
    let mut z = z0;
    project(&mut z);
    let mut y = z.clone();
    let mut t = 1.0f64;
    for _ in 0..iterations {
        let mut zn = &y - (h * &y + g) / l;
        project(&mut zn);
        let tn = (1.0 + (1.0 + 4.0 * t * t).sqrt()) / 2.0;
        y = &zn + (&zn - &z) * ((t - 1.0) / tn);
        z = zn;
        t = tn;
    }
    z
}

/// [`box_qp`] started from the projected unconstrained optimum `-H^-1 g`
/// (exact when no bound is active; a far better start than zero when `H` is
/// ill-conditioned, as in force-based MPC).
pub fn box_qp_newton_start(
    h: &DMatrix<f64>,
    g: &DVector<f64>,
    lo: &DVector<f64>,
    hi: &DVector<f64>,
    iterations: usize,
) -> DVector<f64> {
    let z0 = match h.clone().cholesky() {
        Some(c) => -c.solve(g),
        None => DVector::zeros(g.len()),
    };
    box_qp(h, g, lo, hi, z0, iterations)
}

/// LTV-MPC controller.
#[derive(Debug, Clone)]
pub struct LtvMpc {
    /// stage state weight
    pub q: DMatrix<f64>,
    /// stage input-deviation weight
    pub r: DMatrix<f64>,
    /// terminal state weight
    pub qf: DMatrix<f64>,
    /// horizon [steps]
    pub horizon: usize,
    /// model step [s]
    pub dt_s: f64,
    /// FISTA iterations per solve
    pub iterations: usize,
    warm: Option<DVector<f64>>,
    /// last predicted state trajectory (for plotting)
    pub predicted: Vec<DVector<f64>>,
}

impl LtvMpc {
    /// New controller.
    pub fn new(q: DMatrix<f64>, r: DMatrix<f64>, qf: DMatrix<f64>, horizon: usize, dt_s: f64) -> Self {
        Self {
            q,
            r,
            qf,
            horizon,
            dt_s,
            iterations: 150,
            warm: None,
            predicted: vec![],
        }
    }

    /// Computes the input. `refs[k] = (x_ref_k, u_ref_k)` for `k = 0..=horizon`.
    pub fn control<M: Dynamics>(
        &mut self,
        m: &M,
        x: &DVector<f64>,
        refs: &[(DVector<f64>, DVector<f64>)],
    ) -> DVector<f64> {
        let (nx, nu, n) = (m.nx(), m.nu(), self.horizon);
        assert!(refs.len() > n, "need horizon + 1 reference samples");
        // prediction e_k = Sx_k e0 + Su_k dU + Sc_k
        let mut sx = DMatrix::zeros(n * nx, nx);
        let mut su = DMatrix::zeros(n * nx, n * nu);
        let mut sc = DVector::zeros(n * nx);
        let mut phi = DMatrix::<f64>::identity(nx, nx);
        let mut gam = DMatrix::<f64>::zeros(nx, n * nu);
        let mut cvec = DVector::<f64>::zeros(nx);
        for k in 0..n {
            let (xr, ur) = &refs[k];
            let (a, b) = linearize(m, xr, ur, self.dt_s);
            let c = m.state_error(&step_raw(m, xr, ur, self.dt_s), &refs[k + 1].0);
            phi = &a * phi;
            gam = &a * gam;
            gam.view_mut((0, k * nu), (nx, nu)).copy_from(&b);
            cvec = &a * cvec + c;
            sx.view_mut((k * nx, 0), (nx, nx)).copy_from(&phi);
            su.view_mut((k * nx, 0), (nx, n * nu)).copy_from(&gam);
            sc.rows_mut(k * nx, nx).copy_from(&cvec);
        }
        let mut qbar = DMatrix::zeros(n * nx, n * nx);
        let mut rbar = DMatrix::zeros(n * nu, n * nu);
        for k in 0..n {
            let qk = if k == n - 1 { &self.qf } else { &self.q };
            qbar.view_mut((k * nx, k * nx), (nx, nx)).copy_from(qk);
            rbar.view_mut((k * nu, k * nu), (nu, nu)).copy_from(&self.r);
        }
        let e0 = m.state_error(x, &refs[0].0);
        let sut_q = su.transpose() * &qbar;
        let h = &sut_q * &su + rbar;
        let g = &sut_q * (&sx * &e0 + &sc);
        // bounds on du
        let (umin, umax) = (m.u_min(), m.u_max());
        let mut lo = DVector::zeros(n * nu);
        let mut hi = DVector::zeros(n * nu);
        for k in 0..n {
            for i in 0..nu {
                lo[k * nu + i] = umin[i] - refs[k].1[i];
                hi[k * nu + i] = umax[i] - refs[k].1[i];
            }
        }
        let z0 = match &self.warm {
            Some(w) if w.len() == n * nu => {
                let mut s = DVector::zeros(n * nu);
                s.rows_mut(0, (n - 1) * nu).copy_from(&w.rows(nu, (n - 1) * nu));
                s.rows_mut((n - 1) * nu, nu).copy_from(&w.rows((n - 1) * nu, nu));
                s
            }
            _ => DVector::zeros(n * nu),
        };
        let z = box_qp(&h, &g, &lo, &hi, z0, self.iterations);
        // predicted absolute states for visualisation
        let e_pred = &sx * &e0 + &su * &z + &sc;
        self.predicted = (0..n).map(|k| &refs[k + 1].0 + e_pred.rows(k * nx, nx)).collect();
        let u = &refs[0].1 + z.rows(0, nu);
        self.warm = Some(z);
        m.clamp_input(&u)
    }
}
