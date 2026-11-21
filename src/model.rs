//! The [`Dynamics`] trait, RK4 integration and numerical linearisation.

use nalgebra::{DMatrix, DVector};

/// Continuous-time controlled dynamics `x_dot = f(x, u)`.
pub trait Dynamics {
    /// State dimension.
    fn nx(&self) -> usize;
    /// Input dimension.
    fn nu(&self) -> usize;
    /// State derivative.
    fn deriv(&self, x: &DVector<f64>, u: &DVector<f64>) -> DVector<f64>;
    /// Input lower bounds (actuator limits).
    fn u_min(&self) -> DVector<f64>;
    /// Input upper bounds (actuator limits).
    fn u_max(&self) -> DVector<f64>;
    /// State error `x - x_ref` (override to wrap angles).
    fn state_error(&self, x: &DVector<f64>, x_ref: &DVector<f64>) -> DVector<f64> {
        x - x_ref
    }
    /// Clamps an input to the actuator limits.
    fn clamp_input(&self, u: &DVector<f64>) -> DVector<f64> {
        let (lo, hi) = (self.u_min(), self.u_max());
        DVector::from_iterator(u.len(), (0..u.len()).map(|i| u[i].clamp(lo[i], hi[i])))
    }
}

/// One RK4 step with zero-order-hold input (input is clamped first).
pub fn step<M: Dynamics + ?Sized>(m: &M, x: &DVector<f64>, u: &DVector<f64>, dt: f64) -> DVector<f64> {
    let u = m.clamp_input(u);
    let k1 = m.deriv(x, &u);
    let k2 = m.deriv(&(x + &k1 * (dt / 2.0)), &u);
    let k3 = m.deriv(&(x + &k2 * (dt / 2.0)), &u);
    let k4 = m.deriv(&(x + &k3 * dt), &u);
    x + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0)
}

/// Unclamped RK4 step (used for linearisation so derivatives exist at the limits).
pub fn step_raw<M: Dynamics + ?Sized>(m: &M, x: &DVector<f64>, u: &DVector<f64>, dt: f64) -> DVector<f64> {
    let k1 = m.deriv(x, u);
    let k2 = m.deriv(&(x + &k1 * (dt / 2.0)), u);
    let k3 = m.deriv(&(x + &k2 * (dt / 2.0)), u);
    let k4 = m.deriv(&(x + &k3 * dt), u);
    x + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0)
}

/// Discrete-time Jacobians `(A, B)` of the RK4 step by central differences.
pub fn linearize<M: Dynamics + ?Sized>(
    m: &M,
    x: &DVector<f64>,
    u: &DVector<f64>,
    dt: f64,
) -> (DMatrix<f64>, DMatrix<f64>) {
    let (nx, nu) = (m.nx(), m.nu());
    let mut a = DMatrix::zeros(nx, nx);
    let mut b = DMatrix::zeros(nx, nu);
    let eps = 1e-5;
    for i in 0..nx {
        let mut xp = x.clone();
        let mut xm = x.clone();
        xp[i] += eps;
        xm[i] -= eps;
        let d = (step_raw(m, &xp, u, dt) - step_raw(m, &xm, u, dt)) / (2.0 * eps);
        a.set_column(i, &d);
    }
    for i in 0..nu {
        let mut up = u.clone();
        let mut um = u.clone();
        up[i] += eps;
        um[i] -= eps;
        let d = (step_raw(m, x, &up, dt) - step_raw(m, x, &um, dt)) / (2.0 * eps);
        b.set_column(i, &d);
    }
    (a, b)
}

/// Convenience constructor for a `DVector` from a slice.
pub fn v(xs: &[f64]) -> DVector<f64> {
    DVector::from_column_slice(xs)
}

/// Wraps an angle to `(-pi, pi]`.
pub fn wrap(a: f64) -> f64 {
    robo_planning::geometry::normalize_angle(a)
}
