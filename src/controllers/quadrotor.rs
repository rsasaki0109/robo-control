//! Planar quadrotor: cascaded position/attitude PID and reference generation.

use crate::controllers::pid::Pid;
use crate::model::{v, wrap};
use crate::models::Quadrotor2D;
use nalgebra::DVector;

/// Reference state and feed-forward inputs from a position/velocity/acceleration reference
/// (differential flatness of the planar quadrotor, ignoring jerk).
pub fn flat_reference(q: &Quadrotor2D, p: [f64; 2], vel: [f64; 2], acc: [f64; 2]) -> (DVector<f64>, DVector<f64>) {
    let th = (-acc[0]).atan2(q.gravity_mps2 + acc[1]);
    let thrust = q.mass_kg * (acc[0].hypot(q.gravity_mps2 + acc[1]));
    (
        v(&[p[0], p[1], th, vel[0], vel[1], 0.0]),
        v(&[thrust / 2.0, thrust / 2.0]),
    )
}

/// Cascaded controller: outer PID on position -> desired tilt and thrust; inner PD on attitude.
#[derive(Debug, Clone)]
pub struct CascadedPid {
    /// model parameters (mass, inertia, arm)
    pub model: Quadrotor2D,
    /// x position loop
    pub px: Pid,
    /// z position loop
    pub pz: Pid,
    /// attitude stiffness [1/s^2]
    pub k_att: f64,
    /// attitude damping [1/s]
    pub d_att: f64,
    /// tilt limit [rad]
    pub tilt_max_rad: f64,
}

impl CascadedPid {
    /// Default tuning.
    pub fn new(model: Quadrotor2D) -> Self {
        Self {
            model,
            px: Pid::new(3.0, 0.5, 3.0, 2.0),
            pz: Pid::new(6.0, 2.0, 4.0, 2.0),
            k_att: 300.0,
            d_att: 30.0,
            tilt_max_rad: 0.6,
        }
    }
    /// Control law given state and reference position/velocity/acceleration.
    pub fn control(&mut self, x: &DVector<f64>, p: [f64; 2], vel: [f64; 2], acc: [f64; 2], dt_s: f64) -> DVector<f64> {
        let ax = acc[0] + self.px.update_with_rate(p[0] - x[0], vel[0] - x[3], dt_s);
        let az = acc[1] + self.pz.update_with_rate(p[1] - x[1], vel[1] - x[4], dt_s);
        let m = &self.model;
        let th_des = (-ax)
            .atan2(m.gravity_mps2 + az)
            .clamp(-self.tilt_max_rad, self.tilt_max_rad);
        let thrust = m.mass_kg * (m.gravity_mps2 + az).max(0.0) / th_des.cos();
        let thdd = self.k_att * wrap(th_des - x[2]) - self.d_att * x[5];
        let diff = thdd * m.inertia_kgm2 / m.arm_m; // f_right - f_left
        v(&[(thrust - diff) / 2.0, (thrust + diff) / 2.0])
    }
}
