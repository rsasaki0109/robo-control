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

#[test]
fn go2_trots_over_terraced_rough_terrain() {
    let q = rne_locomotion::Quadruped::go2().unwrap();
    let p = go2::go2_srb(&q);
    let cfg = go2::Go2Config {
        pushes: false,
        terrain_amplitude_m: 0.2,
        terrain_step_m: 0.04,
        ..Default::default()
    };
    let a = go2::run(&mut PdForceQp::new(p), &cfg);
    let b = go2::run(&mut ConvexMpc3d::new(p), &cfg);
    assert!(a.fell_at_s.is_none() && b.fell_at_s.is_none());
    // the ground under the path really varies
    let zs: Vec<f64> = b.frames.iter().map(|f| f.ground[20][1]).collect();
    let span = zs.iter().cloned().fold(f64::NEG_INFINITY, f64::max) - zs.iter().cloned().fold(f64::INFINITY, f64::min);
    assert!(span > 0.1, "terrain span {span}");
    let m = |r: &go2::Go2Run, k: &str| r.metrics.iter().find(|(n, _)| n == k).unwrap().1;
    assert!(m(&b, "rms_tilt_rad") < m(&a, "rms_tilt_rad"));
}

#[test]
fn g1_dcm_walks_up_the_ramp_through_the_push() {
    let r = g1::run_on(
        Strategy::DcmStepAdjustment,
        9.0,
        true,
        g1::WbcBackend::Rne,
        g1::G1Terrain::Ramp,
    );
    assert!(r.fell_at_s.is_none(), "fell at {:?}", r.fell_at_s);
    let f = r.frames.last().unwrap();
    assert!(f.com[0] > 2.0, "did not reach the top: {}", f.com[0]);
    assert!(f.points[0][2] > 0.75, "pelvis did not rise with the ramp");
}
