//! # robo-control
//!
//! Robot models, controllers and a fixed-step simulator for comparing control
//! laws on the robot and task they fit. Reference trajectories come from
//! [`robo_planning`].
//!
//! | Robot | Model | Controllers that fit |
//! |---|---|---|
//! | Differential-drive AMR | [`models::Unicycle`] | pure pursuit, Kanayama, LQR, LTV-MPC, DWA |
//! | Car | [`models::Bicycle`] | pure pursuit, Stanley, LQR, LTV-MPC |
//! | Planar serial arm | [`models::PlanarArmDyn`] | PD, PD + gravity compensation, computed torque |
//! | Cart-pole | [`models::CartPole`] | energy swing-up + LQR, iLQR + TV-LQR |
//! | Planar quadrotor | [`models::Quadrotor2D`] | cascaded PID, LQR, LTV-MPC |
//! | Quadruped | single rigid body ([`scenarios::quadruped`]) | PD + force QP, convex MPC |
//! | Humanoid | 3D LIPM ([`scenarios::humanoid`]) | ZMP preview control, DCM step adjustment |
//!
//! Generic pieces ([`lqr`], [`mpc`], [`ilqr`]) work on any [`model::Dynamics`].

#![allow(rustdoc::broken_intra_doc_links)] // unit annotations such as [m] in docs
// Index loops mirror the maths in the dynamics and solvers.
#![allow(clippy::needless_range_loop)]

pub mod controllers;
pub mod ilqr;
pub mod lqr;
pub mod metrics;
pub mod model;
pub mod models;
pub mod mpc;
pub mod scenarios;
pub mod sim;

pub use nalgebra::{DMatrix, DVector};
pub use robo_planning;
