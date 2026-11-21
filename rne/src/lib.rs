//! Runs robo-control's legged controllers on Robot Native Engine's articulated
//! Unitree models:
//!
//! * [`go2`]: the 12-DoF Go2 in `rne_locomotion`'s hard-contact terrain sim.
//!   robo-control's 3D single-rigid-body force controllers
//!   ([`robo_control::legged3d`]) choose the stance-foot forces, which become
//!   joint torques through `-J^T f`; swing legs land at Raibert footholds through
//!   the leg inverse kinematics.
//! * [`g1`]: the 12-DoF G1 legs on `rne_dynamics::contact_step` with flat-foot
//!   contacts. robo-control's LIPM walkers (ZMP preview, DCM step adjustment)
//!   choose the CoM motion and footholds; a force-QP whole-body controller
//!   built on `rne_dynamics` (mass matrix, bias forces, contact Jacobians) turns
//!   them into joint torques.

pub mod g1;
pub mod go2;

/// Converts an RNE vector to nalgebra.
pub fn na(v: rne_math::Vec3) -> nalgebra::Vector3<f64> {
    nalgebra::Vector3::new(v.x, v.y, v.z)
}

/// Converts an nalgebra vector to RNE.
pub fn rv(v: nalgebra::Vector3<f64>) -> rne_math::Vec3 {
    rne_math::Vec3::new(v.x, v.y, v.z)
}
