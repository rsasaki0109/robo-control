//! robo-control's legged controllers on RNE's articulated Go2 and G1.

use robo_control::legged3d::{ConvexMpc3d, PdForceQp};
use robo_control::scenarios::humanoid::Strategy;
use robo_control_rne::{g1, go2};

#[test]
fn go2_trots_through_both_pushes_and_mpc_tracks_speed_better() {
    let q = rne_locomotion::Quadruped::go2().unwrap();
    let p = go2::go2_srb(&q);
    let cfg = go2::Go2Config::default();
    let a = go2::run(&mut PdForceQp::new(p), &cfg);
    let b = go2::run(&mut ConvexMpc3d::new(p), &cfg);
    assert!(a.fell_at_s.is_none() && b.fell_at_s.is_none());
    let m = |r: &go2::Go2Run, k: &str| r.metrics.iter().find(|(n, _)| n == k).unwrap().1;
    assert!(m(&b, "rms_speed_err_mps") < m(&a, "rms_speed_err_mps"));
    assert!(m(&b, "distance_m") > 4.0);
}

#[test]
fn g1_dcm_stepping_walks_through_the_push_and_fixed_steps_fall() {
    let dcm = g1::run(Strategy::DcmStepAdjustment, 8.0, true);
    assert!(dcm.fell_at_s.is_none(), "DCM fell at {:?}", dcm.fell_at_s);
    assert!(dcm.frames.last().unwrap().com[0] > 1.8);
    let preview = g1::run(Strategy::PreviewFixedSteps, 8.0, false);
    assert!(preview.fell_at_s.is_some(), "fixed footsteps unexpectedly walked");
}

#[test]
fn g1_also_walks_on_the_independent_force_qp_controller() {
    let r = g1::run_with(Strategy::DcmStepAdjustment, 8.0, true, g1::WbcBackend::ForceQp);
    assert!(r.fell_at_s.is_none(), "fell at {:?}", r.fell_at_s);
}
