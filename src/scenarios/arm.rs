//! Serial arm: pick-and-place along a joint-space plan with an unmodelled payload.

use super::Run;
use crate::controllers::arm::{ArmController, ArmLaw};
use crate::metrics::{max_abs, rms};
use crate::models::PlanarArmDyn;
use crate::sim::{simulate, SimConfig};
use nalgebra::DVector;
use robo_planning::arm::{ArmSpace, PlanarArm};
use robo_planning::sampling::{rrt_connect, shortcut, SamplingConfig};
use robo_planning::trajectory::JointTrajectory;
use robo_planning::{scenes, Map2D, Vec2};

/// Arm scenario output.
#[derive(Debug, Clone)]
pub struct ArmResult {
    /// kinematic arm (for drawing)
    pub arm: PlanarArm,
    /// work cell
    pub map: Map2D,
    /// pick and place points
    pub pick_place: (Vec2, Vec2),
    /// reference joint trajectory
    pub reference: JointTrajectory,
    /// runs
    pub runs: Vec<Run>,
    /// payload carried by the plant but unknown to the controllers [kg]
    pub payload_kg: f64,
}

/// Plans pick -> place with RRT-Connect and tracks it with PD, PD + gravity
/// compensation and computed torque; the plant carries a 2 kg payload the
/// controllers' model does not know about.
pub fn pick_place() -> ArmResult {
    let (arm, map, pick, place) = scenes::arm_cell();
    let sp = ArmSpace {
        arm: &arm,
        map: &map,
        link_margin_m: 0.06,
    };
    let qa = sp.ik_valid(pick, 64, 1).expect("ik pick");
    let qb = sp.ik_valid(place, 64, 2).expect("ik place");
    let r = rrt_connect(
        &sp,
        &qa,
        &qb,
        &SamplingConfig {
            step: 0.15,
            max_iters: 20000,
            ..Default::default()
        },
    );
    let reference = JointTrajectory::new(shortcut(&sp, &r.path, 300, 9), 1.5, 3.0);
    let nominal = PlanarArmDyn::three_link();
    let payload_kg = 2.0;
    let plant = PlanarArmDyn {
        payload_kg,
        ..nominal.clone()
    };
    let t_end = reference.duration_s();
    let cfg = SimConfig {
        dt_s: 0.002,
        duration_s: t_end + 1.0,
        ..Default::default()
    };
    let x0 = DVector::from_iterator(6, qa.iter().copied().chain([0.0; 3]));
    let mut runs = Vec::new();
    for (name, law) in [
        ("PD", ArmLaw::Pd),
        ("PD + gravity", ArmLaw::PdGravity),
        ("computed torque", ArmLaw::ComputedTorque),
    ] {
        let c = ArmController {
            law,
            model: nominal.clone(),
            kp: vec![1500.0, 900.0, 300.0],
            kd: vec![150.0, 90.0, 25.0],
            wn_radps: 20.0,
        };
        let rf = reference.clone();
        let log = simulate(
            &plant,
            x0.clone(),
            &cfg,
            |t, x| {
                let (q, qd, qdd) = rf.sample(t);
                c.control(x, &q, &qd, &qdd)
            },
            |_, _| DVector::zeros(6),
            |_, _| false,
        );
        let mut ee_err = Vec::new();
        let mut clearance = f64::INFINITY;
        for (t, x) in log.t.iter().zip(&log.x) {
            let q: Vec<f64> = x.rows(0, 3).iter().copied().collect();
            ee_err.push(arm.end_effector(&q).dist(arm.end_effector(&reference.sample(*t).0)));
            clearance = clearance.min(sp.clearance(&q));
        }
        let final_err = *ee_err.last().unwrap();
        runs.push(Run {
            name: name.into(),
            metrics: vec![
                ("rms_ee_err_m".into(), rms(&ee_err)),
                ("max_ee_err_m".into(), max_abs(&ee_err)),
                ("final_ee_err_m".into(), final_err),
                ("min_link_clearance_m".into(), clearance),
            ],
            log,
        });
    }
    ArmResult {
        arm,
        map,
        pick_place: (pick, place),
        reference,
        runs,
        payload_kg,
    }
}
