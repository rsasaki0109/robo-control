//! Joint-space controllers for serial arms.

use crate::models::PlanarArmDyn;
use nalgebra::DVector;

/// Joint-space control law.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ArmLaw {
    /// independent joint PD
    Pd,
    /// PD plus model gravity compensation
    PdGravity,
    /// computed torque (inverse dynamics) with PD on the error
    ComputedTorque,
}

/// Arm controller using a *nominal* model (which may differ from the plant).
#[derive(Debug, Clone)]
pub struct ArmController {
    /// control law
    pub law: ArmLaw,
    /// nominal model
    pub model: PlanarArmDyn,
    /// proportional gains (PD laws) [N m/rad]
    pub kp: Vec<f64>,
    /// derivative gains (PD laws) [N m s/rad]
    pub kd: Vec<f64>,
    /// closed-loop natural frequency for computed torque [rad/s]
    pub wn_radps: f64,
}

impl ArmController {
    /// Torque for state `x = [q, qd]` and reference `(q_r, qd_r, qdd_r)`.
    pub fn control(&self, x: &DVector<f64>, q_r: &[f64], qd_r: &[f64], qdd_r: &[f64]) -> DVector<f64> {
        let n = self.model.dof();
        let q: Vec<f64> = x.rows(0, n).iter().copied().collect();
        let qd: Vec<f64> = x.rows(n, n).iter().copied().collect();
        let e = DVector::from_iterator(n, (0..n).map(|i| q_r[i] - q[i]));
        let ed = DVector::from_iterator(n, (0..n).map(|i| qd_r[i] - qd[i]));
        match self.law {
            ArmLaw::Pd => DVector::from_iterator(n, (0..n).map(|i| self.kp[i] * e[i] + self.kd[i] * ed[i])),
            ArmLaw::PdGravity => {
                DVector::from_iterator(n, (0..n).map(|i| self.kp[i] * e[i] + self.kd[i] * ed[i]))
                    + self.model.gravity(&q)
            }
            ArmLaw::ComputedTorque => {
                let (kp, kd) = (self.wn_radps * self.wn_radps, 2.0 * self.wn_radps);
                let a: Vec<f64> = (0..n).map(|i| qdd_r[i] + kp * e[i] + kd * ed[i]).collect();
                self.model.inverse_dynamics(&q, &qd, &a)
            }
        }
    }
}
