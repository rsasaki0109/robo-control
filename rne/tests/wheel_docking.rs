use robo_control_rne::docking::{dock, Drive};

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
