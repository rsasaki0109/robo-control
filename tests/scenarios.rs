//! Each body's scenario must reproduce its headline result.

use robo_control::scenarios::{arm, cartpole, drone, humanoid, quadruped, wheeled};

#[test]
fn amr_controllers_track_and_mpc_is_tightest() {
    let r = wheeled::amr_warehouse();
    for run in &r.runs {
        assert!(
            run.metric("goal_err_m") < 0.1,
            "{} goal error {}",
            run.name,
            run.metric("goal_err_m")
        );
        assert!(run.metric("max_cte_m") < 0.5, "{}", run.name);
    }
    let pp = r.runs[0].metric("rms_cte_m");
    let mpc = r.runs[3].metric("rms_cte_m");
    assert!(mpc < pp, "mpc {mpc} vs pure pursuit {pp}");
}

#[test]
fn car_controllers_complete_the_course() {
    let r = wheeled::car_course();
    for run in &r.runs {
        assert!(
            run.metric("max_cte_m") < 3.0,
            "{} max cte {}",
            run.name,
            run.metric("max_cte_m")
        );
    }
    assert!(r.runs[3].metric("rms_cte_m") < r.runs[2].metric("rms_cte_m"));
    // sampling-based MPPI tracks tighter than pure pursuit with no derivatives
    let mppi = r.runs.iter().find(|x| x.name == "MPPI").unwrap();
    assert!(mppi.metric("rms_cte_m") < r.runs[0].metric("rms_cte_m"));
    assert!(r.runs.iter().any(|x| x.name.starts_with("NMPC")));
}

#[test]
fn arm_model_based_control_beats_pd_under_payload() {
    let r = arm::pick_place();
    let e: Vec<f64> = r.runs.iter().map(|x| x.metric("rms_ee_err_m")).collect();
    assert!(e[2] < e[1] && e[1] < e[0], "{e:?}");
    assert!(r.runs[2].metric("min_link_clearance_m") > 0.0);
}

#[test]
fn drone_lqr_and_mpc_reject_the_gust_better_than_pid() {
    let r = drone::forest_flight();
    let pid = r.runs[0].metric("rms_path_err_m");
    assert!(r.runs[1].metric("rms_path_err_m") < pid);
    assert!(r.runs[2].metric("rms_path_err_m") < pid);
    for run in &r.runs {
        assert!(run.metric("min_clearance_m") > 0.0, "{} collided", run.name);
    }
}

#[test]
fn cartpole_both_swing_up_and_survive_push() {
    let r = cartpole::swing_up();
    for run in &r.runs {
        assert!(run.metric("final_angle_rad") < 0.05, "{}", run.name);
        assert!(run.metric("time_to_upright_s") < 4.0, "{}", run.name);
    }
}

#[test]
fn quadruped_trots_through_push_and_mpc_tracks_speed_better() {
    let r = quadruped::trot();
    for q in &r {
        assert!(q.fell_at_s.is_none(), "{} fell", q.run.name);
    }
    assert!(r[1].run.metric("rms_speed_err_mps") < r[0].run.metric("rms_speed_err_mps"));
}

#[test]
fn humanoid_both_walk_but_only_dcm_stepping_recovers_from_push() {
    let p = humanoid::LipmParams::default();
    assert!(humanoid::walk(p, false, 6.0, false).fell_at_s.is_none());
    assert!(humanoid::walk(p, true, 6.0, false).fell_at_s.is_none());
    let r = humanoid::push_recovery();
    assert!(r[0].fell_at_s.is_some(), "fixed footsteps should not survive the push");
    assert!(r[1].fell_at_s.is_none(), "step adjustment should recover");
}
