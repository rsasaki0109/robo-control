use robo_control_rne::docking::{dock, dock_with, DockingConditions, Drive};

#[test]
fn both_wheel_layouts_dock_sideways_with_fixed_heading() {
    for drive in [Drive::Omni, Drive::Swerve] {
        let run = dock(drive);
        assert!(run.arrival_s.is_some_and(|t| (6.0..10.0).contains(&t)), "{drive:?}");
        assert!(run.min_clearance_m > 0.0, "{drive:?}: {}", run.min_clearance_m);
        assert!(run.max_heading_error_rad < 3_f64.to_radians(), "{drive:?}");
        assert!(run.rms_path_error_m < 0.15, "{drive:?}");
        assert!(run.rms_driven_slip_mps.is_finite(), "{drive:?}");
        match drive {
            Drive::Omni => {
                assert!(run.rms_lateral_slip_mps.is_none());
                assert!(run.steering_settle_s.is_none());
            }
            Drive::Swerve => {
                assert!(run.rms_lateral_slip_mps.is_some_and(f64::is_finite));
                assert!(run.steering_settle_s.is_some_and(|t| (0.1..1.0).contains(&t)));
            }
        }
    }
}

#[test]
fn displaced_start_and_wheel_grip_affect_docking() {
    let conditions = DockingConditions {
        start_offset_m: -0.05,
        friction: 0.25,
    };
    let low_grip = dock_with(Drive::Omni, conditions);
    let nominal_grip = dock_with(
        Drive::Omni,
        DockingConditions {
            friction: 0.75,
            ..conditions
        },
    );
    assert!((low_grip.frames[0].xy[0] + 1.05).abs() < 1e-12);
    assert!(low_grip.arrival_s.is_some());
    assert!(low_grip.min_clearance_m < nominal_grip.min_clearance_m);
    let no_grip = dock_with(
        Drive::Swerve,
        DockingConditions {
            friction: 0.0,
            ..conditions
        },
    );
    assert!(no_grip.arrival_s.is_none());
    assert_eq!(no_grip.frames.first().unwrap().xy, no_grip.frames.last().unwrap().xy);
}
