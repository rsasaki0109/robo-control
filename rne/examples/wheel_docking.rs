//! Runs the RNE docking comparison and writes poses to stdout as CSV.
//! cargo run --release -p robo-control-rne --example wheel_docking > docking.csv

use robo_control_rne::docking::{dock, Drive};

fn main() {
    println!("drive,time_s,x_m,y_m,yaw_rad,ref_x_m,ref_y_m,wheel,wx_m,wy_m,steering_rad,spin_rad");
    for (name, drive) in [("omni", Drive::Omni), ("swerve", Drive::Swerve)] {
        let run = dock(drive);
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
}
