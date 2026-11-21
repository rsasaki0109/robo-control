//! Iterative LQR trajectory optimisation for any [`Dynamics`].

use crate::model::{linearize, step, Dynamics};
use nalgebra::{DMatrix, DVector};

/// Quadratic cost around a goal state.
#[derive(Debug, Clone)]
pub struct IlqrCost {
    /// goal state
    pub goal: DVector<f64>,
    /// running state weight
    pub q: DMatrix<f64>,
    /// running input weight
    pub r: DMatrix<f64>,
    /// terminal state weight
    pub qf: DMatrix<f64>,
}

/// Optimised trajectory with its time-varying feedback gains.
#[derive(Debug, Clone)]
pub struct IlqrSolution {
    /// states `x_0..x_N`
    pub xs: Vec<DVector<f64>>,
    /// inputs `u_0..u_{N-1}`
    pub us: Vec<DVector<f64>>,
    /// feedback gains: apply `u = us[k] + gains[k] (x - xs[k])`
    pub gains: Vec<DMatrix<f64>>,
    /// final cost
    pub cost: f64,
    /// iterations used
    pub iterations: usize,
}

fn rollout<M: Dynamics>(m: &M, x0: &DVector<f64>, us: &[DVector<f64>], dt: f64) -> Vec<DVector<f64>> {
    let mut xs = vec![x0.clone()];
    for u in us {
        let x = step(m, xs.last().unwrap(), u, dt);
        xs.push(x);
    }
    xs
}

fn total_cost<M: Dynamics>(m: &M, xs: &[DVector<f64>], us: &[DVector<f64>], c: &IlqrCost) -> f64 {
    let mut j = 0.0;
    for (x, u) in xs.iter().zip(us) {
        let e = m.state_error(x, &c.goal);
        j += (e.transpose() * &c.q * &e)[(0, 0)] + (u.transpose() * &c.r * u)[(0, 0)];
    }
    let e = m.state_error(xs.last().unwrap(), &c.goal);
    j + (e.transpose() * &c.qf * &e)[(0, 0)]
}

/// Runs iLQR from `x0` with initial inputs `us0` (inputs are clamped in the rollout).
pub fn solve<M: Dynamics>(
    m: &M,
    x0: &DVector<f64>,
    us0: Vec<DVector<f64>>,
    cost: &IlqrCost,
    dt_s: f64,
    max_iters: usize,
) -> IlqrSolution {
    let n = us0.len();
    let mut us: Vec<DVector<f64>> = us0.iter().map(|u| m.clamp_input(u)).collect();
    let mut xs = rollout(m, x0, &us, dt_s);
    let mut j = total_cost(m, &xs, &us, cost);
    let mut mu = 1e-3;
    let mut gains = vec![DMatrix::zeros(m.nu(), m.nx()); n];
    let mut iters = 0;
    for it in 0..max_iters {
        iters = it + 1;
        let mut ff = vec![DVector::zeros(m.nu()); n];
        let mut kk = vec![DMatrix::zeros(m.nu(), m.nx()); n];
        let e_n = m.state_error(&xs[n], &cost.goal);
        let mut vx = &cost.qf * e_n * 2.0;
        let mut vxx = &cost.qf * 2.0;
        let mut ok = true;
        for k in (0..n).rev() {
            let (a, b) = linearize(m, &xs[k], &us[k], dt_s);
            let e = m.state_error(&xs[k], &cost.goal);
            let qx = &cost.q * e * 2.0 + a.transpose() * &vx;
            let qu = &cost.r * &us[k] * 2.0 + b.transpose() * &vx;
            let qxx = &cost.q * 2.0 + a.transpose() * &vxx * &a;
            let quu = &cost.r * 2.0 + b.transpose() * &vxx * &b + DMatrix::identity(m.nu(), m.nu()) * mu;
            let qux = b.transpose() * &vxx * &a;
            let Some(inv) = quu.clone().try_inverse() else {
                ok = false;
                break;
            };
            ff[k] = -&inv * &qu;
            kk[k] = -&inv * &qux;
            vx = &qx + kk[k].transpose() * &quu * &ff[k] + kk[k].transpose() * &qu + qux.transpose() * &ff[k];
            vxx = &qxx + kk[k].transpose() * &quu * &kk[k] + kk[k].transpose() * &qux + qux.transpose() * &kk[k];
            vxx = (&vxx + vxx.transpose()) * 0.5;
        }
        if !ok {
            mu *= 10.0;
            continue;
        }
        let mut improved = false;
        for alpha in [1.0, 0.5, 0.25, 0.1, 0.05, 0.01] {
            let mut nx = vec![x0.clone()];
            let mut nu = Vec::with_capacity(n);
            for k in 0..n {
                let dx = m.state_error(&nx[k], &xs[k]);
                let u = m.clamp_input(&(&us[k] + &ff[k] * alpha + &kk[k] * dx));
                nx.push(step(m, &nx[k], &u, dt_s));
                nu.push(u);
            }
            let nj = total_cost(m, &nx, &nu, cost);
            if nj < j {
                let rel = (j - nj) / j.max(1e-12);
                xs = nx;
                us = nu;
                j = nj;
                gains = kk.clone();
                improved = true;
                mu = (mu * 0.5).max(1e-6);
                if rel < 1e-6 {
                    return IlqrSolution {
                        xs,
                        us,
                        gains,
                        cost: j,
                        iterations: iters,
                    };
                }
                break;
            }
        }
        if !improved {
            mu *= 10.0;
            if mu > 1e8 {
                break;
            }
        }
    }
    IlqrSolution {
        xs,
        us,
        gains,
        cost: j,
        iterations: iters,
    }
}
