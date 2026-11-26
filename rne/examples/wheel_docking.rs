//! Runs the RNE docking comparison and writes poses to stdout as CSV.
//! cargo run --release -p robo-control-rne --example wheel_docking > docking.csv

use robo_control_rne::docking::{dock_with, DockingConditions, Drive};

const DRIVES: [(&str, Drive); 2] = [("omni", Drive::Omni), ("swerve", Drive::Swerve)];

fn sweep() {
    println!("drive,offset_m,friction,docked,arrival_s,min_clearance_m");
    for friction in [0.75, 0.45, 0.25] {
        for start_offset_m in [-0.05, 0.0, 0.05] {
            let conditions = DockingConditions {
                start_offset_m,
                friction,
            };
            for (name, drive) in DRIVES {
                let run = dock_with(drive, conditions);
                let arrival = run.arrival_s.map_or_else(String::new, |t| format!("{t:.3}"));
                println!(
                    "{name},{start_offset_m:.2},{friction:.2},{},{arrival},{:.4}",
                    run.arrival_s.is_some(),
                    run.min_clearance_m
                );
            }
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let conditions = match args.as_slice() {
        [] => DockingConditions::default(),
        [flag] if flag == "--sweep" => {
            sweep();
            return Ok(());
        }
        [flag, offset, friction] if flag == "--case" => DockingConditions {
            start_offset_m: offset.parse()?,
            friction: friction.parse()?,
        },
        _ => return Err("usage: wheel_docking [--sweep | --case OFFSET_M FRICTION]".into()),
    };
    if !conditions.start_offset_m.is_finite() || !conditions.friction.is_finite() || conditions.friction < 0.0 {
        return Err("offset must be finite and friction must be finite and nonnegative".into());
    }
    println!("drive,time_s,x_m,y_m,yaw_rad,ref_x_m,ref_y_m,wheel,wx_m,wy_m,steering_rad,spin_rad");
    for (name, drive) in DRIVES {
        let run = dock_with(drive, conditions);
        eprintln!(
            "{name}: arrival {:?} s, RMS path {:.4} m, max yaw {:.2} deg, clearance {:.4} m, driven slip {:.4} m/s, lateral slip {:?} m/s, steering settle {:?} s",
            run.arrival_s,
            run.rms_path_error_m,
            run.max_heading_error_rad.to_degrees(),
            run.min_clearance_m,
            run.rms_driven_slip_mps,
            run.rms_lateral_slip_mps,
            run.steering_settle_s
        );
        for f in run.frames {
            for (i, w) in f.wheels.iter().enumerate() {
                println!(
                    "{name},{},{},{},{},{},{},{i},{},{},{},{}",
                    f.t, f.xy[0], f.xy[1], f.yaw, f.reference[0], f.reference[1], w.xy[0], w.xy[1], w.steering, w.spin
                );
            }
        }
    }
    Ok(())
}
