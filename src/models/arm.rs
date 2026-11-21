use crate::model::Dynamics;
use nalgebra::{DMatrix, DVector};

/// Planar serial arm of uniform rods with joint friction, gravity along `-y` and
/// an optional point payload at the end effector.
///
/// State `[q_1..q_n, qd_1..qd_n]`, input joint torques `[tau_1..tau_n]`.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanarArmDyn {
    /// link lengths [m]
    pub link_lengths_m: Vec<f64>,
    /// link masses [kg]
    pub link_masses_kg: Vec<f64>,
    /// end-effector payload [kg]
    pub payload_kg: f64,
    /// viscous joint friction [N m s/rad]
    pub joint_friction: f64,
    /// torque limits [N m]
    pub torque_max_nm: Vec<f64>,
    /// gravity [m/s^2]
    pub gravity_mps2: f64,
}

impl PlanarArmDyn {
    /// Three-link arm matching `robo_planning::scenes::arm_cell`.
    pub fn three_link() -> Self {
        Self {
            link_lengths_m: vec![1.0, 0.8, 0.6],
            link_masses_kg: vec![4.0, 3.0, 1.5],
            payload_kg: 0.0,
            joint_friction: 0.5,
            torque_max_nm: vec![120.0, 60.0, 25.0],
            gravity_mps2: 9.81,
        }
    }
    /// Degrees of freedom.
    pub fn dof(&self) -> usize {
        self.link_lengths_m.len()
    }
    /// Inverse dynamics (recursive Newton-Euler, planar): torques for `(q, qd, qdd)`, including gravity and friction.
    pub fn inverse_dynamics(&self, q: &[f64], qd: &[f64], qdd: &[f64]) -> DVector<f64> {
        let n = self.dof();
        let cross = |a: [f64; 2], b: [f64; 2]| a[0] * b[1] - a[1] * b[0];
        let (mut phi, mut w, mut al) = (0.0, 0.0, 0.0);
        let mut p = [0.0, 0.0];
        let mut ap = [0.0, self.gravity_mps2]; // base accelerates up = gravity
        let mut pts = vec![p];
        let mut acom = vec![[0.0; 2]; n];
        let mut com = vec![[0.0; 2]; n];
        let mut alpha = vec![0.0; n];
        for i in 0..n {
            phi += q[i];
            w += qd[i];
            al += qdd[i];
            let l = self.link_lengths_m[i];
            let (e, en) = ([phi.cos(), phi.sin()], [-phi.sin(), phi.cos()]);
            let acc = |r: f64| {
                [
                    ap[0] + r * (al * en[0] - w * w * e[0]),
                    ap[1] + r * (al * en[1] - w * w * e[1]),
                ]
            };
            acom[i] = acc(l / 2.0);
            com[i] = [p[0] + l / 2.0 * e[0], p[1] + l / 2.0 * e[1]];
            alpha[i] = al;
            ap = acc(l);
            p = [p[0] + l * e[0], p[1] + l * e[1]];
            pts.push(p);
        }
        let mut f_next = [self.payload_kg * ap[0], self.payload_kg * ap[1]];
        let mut tau_next = 0.0;
        let mut tau = DVector::zeros(n);
        for i in (0..n).rev() {
            let m = self.link_masses_kg[i];
            let l = self.link_lengths_m[i];
            let inertia = m * l * l / 12.0;
            let f = [m * acom[i][0] + f_next[0], m * acom[i][1] + f_next[1]];
            let r_in = [pts[i][0] - com[i][0], pts[i][1] - com[i][1]];
            let r_out = [pts[i + 1][0] - com[i][0], pts[i + 1][1] - com[i][1]];
            let t = inertia * alpha[i] + tau_next - cross(r_in, f) + cross(r_out, f_next);
            tau[i] = t + self.joint_friction * qd[i];
            f_next = f;
            tau_next = t;
        }
        tau
    }
    /// Mass matrix.
    pub fn mass_matrix(&self, q: &[f64]) -> DMatrix<f64> {
        let n = self.dof();
        let z = vec![0.0; n];
        let h0 = self.inverse_dynamics(q, &z, &z);
        let mut m = DMatrix::zeros(n, n);
        for j in 0..n {
            let mut e = vec![0.0; n];
            e[j] = 1.0;
            m.set_column(j, &(self.inverse_dynamics(q, &z, &e) - &h0));
        }
        m
    }
    /// Gravity torque vector.
    pub fn gravity(&self, q: &[f64]) -> DVector<f64> {
        let z = vec![0.0; self.dof()];
        self.inverse_dynamics(q, &z, &z)
    }
}

impl Dynamics for PlanarArmDyn {
    fn nx(&self) -> usize {
        2 * self.dof()
    }
    fn nu(&self) -> usize {
        self.dof()
    }
    fn deriv(&self, x: &DVector<f64>, u: &DVector<f64>) -> DVector<f64> {
        let n = self.dof();
        let q: Vec<f64> = x.rows(0, n).iter().copied().collect();
        let qd: Vec<f64> = x.rows(n, n).iter().copied().collect();
        let h = self.inverse_dynamics(&q, &qd, &vec![0.0; n]);
        let qdd = self.mass_matrix(&q).lu().solve(&(u - h)).expect("singular mass matrix");
        let mut d = DVector::zeros(2 * n);
        d.rows_mut(0, n).copy_from(&x.rows(n, n));
        d.rows_mut(n, n).copy_from(&qdd);
        d
    }
    fn u_min(&self) -> DVector<f64> {
        DVector::from_iterator(self.dof(), self.torque_max_nm.iter().map(|t| -t))
    }
    fn u_max(&self) -> DVector<f64> {
        DVector::from_column_slice(&self.torque_max_nm)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn single_link_matches_pendulum() {
        let arm = PlanarArmDyn {
            link_lengths_m: vec![1.0],
            link_masses_kg: vec![2.0],
            payload_kg: 0.0,
            joint_friction: 0.0,
            torque_max_nm: vec![100.0],
            gravity_mps2: 9.81,
        };
        // horizontal: gravity torque = m g l/2
        assert!((arm.gravity(&[0.0])[0] - 2.0 * 9.81 * 0.5).abs() < 1e-9);
        // inertia about the pivot = m l^2 / 3
        assert!((arm.mass_matrix(&[0.3])[(0, 0)] - 2.0 / 3.0).abs() < 1e-9);
    }

    #[test]
    fn mass_matrix_symmetric_positive_definite() {
        let arm = PlanarArmDyn::three_link();
        let m = arm.mass_matrix(&[0.3, -0.7, 1.1]);
        assert!((&m - m.transpose()).abs().max() < 1e-9);
        assert!(m.clone().cholesky().is_some());
    }

    #[test]
    fn energy_conserved_without_friction() {
        let mut arm = PlanarArmDyn::three_link();
        arm.joint_friction = 0.0;
        let energy = |x: &DVector<f64>| {
            let q: Vec<f64> = x.rows(0, 3).iter().copied().collect();
            let qd = x.rows(3, 3).clone_owned();
            let ke = 0.5 * (qd.transpose() * arm.mass_matrix(&q) * &qd)[(0, 0)];
            // potential: sum m g y_com
            let (mut phi, mut y, mut pe) = (0.0, 0.0, 0.0);
            for i in 0..3 {
                phi += q[i];
                pe += arm.link_masses_kg[i] * 9.81 * (y + arm.link_lengths_m[i] / 2.0 * phi.sin());
                y += arm.link_lengths_m[i] * phi.sin();
            }
            ke + pe
        };
        let mut x = DVector::from_vec(vec![0.5, 0.2, -0.3, 0.0, 0.0, 0.0]);
        let e0 = energy(&x);
        for _ in 0..500 {
            x = crate::model::step(&arm, &x, &DVector::zeros(3), 0.002);
        }
        assert!((energy(&x) - e0).abs() < 1e-3 * e0.abs().max(1.0));
    }
}
