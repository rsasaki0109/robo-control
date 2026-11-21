//! GIFs of robo-control's legged controllers on RNE's articulated Unitree Go2
//! and G1, plus a metrics table.
//!
//! ```bash
//! cargo run --release -p robo-control-rne --example rne_gallery        # out/rne/{go2,g1}.gif + README.md
//! cargo run --release -p robo-control-rne --example rne_gallery g1     # one of: go2 g1 go2_terrain g1_ramp
//! ```

use robo_control::legged3d::{ConvexMpc3d, ForceController, PdForceQp};
use robo_control::robo_planning::render::{colors, Canvas, GifWriter, Rgba, View};
use robo_control::robo_planning::{Pose2, Vec2};
use robo_control::scenarios::humanoid::{Strategy, PUSH};
use robo_control_rne::{g1, go2};
use std::fmt::Write as _;

const OUT: &str = "out/rne";

fn title(c: &mut Canvas, s: &str) {
    c.rect_px(0.0, 0.0, c.width() as f32, 30.0, colors::INK);
    c.text(10.0, 8.0, s, 2.0, colors::WHITE);
}

fn table(rows: &[(String, Vec<(String, f64)>)]) -> String {
    let mut s = String::from("| controller |");
    for (k, _) in &rows[0].1 {
        let _ = write!(s, " {k} |");
    }
    s += "\n|---|";
    s += &"---|".repeat(rows[0].1.len());
    s += "\n";
    for (n, m) in rows {
        let _ = write!(s, "| {n} |");
        for (_, v) in m {
            let _ = write!(s, " {v:.3} |");
        }
        s += "\n";
    }
    s
}

/// Ground from a sampled profile `(x, z)`, filled down to `y_bottom` px.
fn ground_profile(c: &mut Canvas, v: &View, profile: &[[f64; 2]], y_bottom: f32) {
    let mut poly: Vec<(f32, f32)> = profile.iter().map(|g| v.px(Vec2::new(g[0], g[1]))).collect();
    if let (Some(first), Some(last)) = (poly.first().copied(), poly.last().copied()) {
        poly.push((last.0, y_bottom));
        poly.push((first.0, y_bottom));
    }
    c.polygon_px(&poly, Rgba::rgb(225, 220, 205));
    let line: Vec<(f32, f32)> = profile.iter().map(|g| v.px(Vec2::new(g[0], g[1]))).collect();
    c.polyline_px(&line, Rgba::rgb(150, 140, 120), 2.0);
    for g in profile.iter().step_by(5) {
        let (tx, ty) = v.px(Vec2::new(g[0], g[1]));
        c.rect_px(tx - 1.0, ty, 2.0, 8.0, colors::INK);
    }
}

fn go2_gif(runs: &[go2::Go2Run], file: &str, heading: &str) -> std::io::Result<()> {
    let (w, h) = (900u32, 600u32);
    let mut gif = GifWriter::create(format!("{OUT}/{file}.gif"), w, h)?;
    let n = runs[0].frames.len();
    for fi in (0..n).step_by(3) {
        let mut c = Canvas::new(w, h);
        title(&mut c, heading);
        for (k, r) in runs.iter().enumerate() {
            let f = &r.frames[fi.min(r.frames.len() - 1)];
            let y0 = 36.0 + k as f32 * 282.0;
            let cx = f.base_m[0];
            // side view (x-z)
            let gz = f.ground[20][1];
            let v = View::fit(
                Vec2::new(cx - 0.9, gz - 0.14),
                Vec2::new(cx + 0.9, gz + 0.5),
                10.0,
                y0,
                640.0,
                276.0,
            );
            c.rect_px(10.0, y0, 640.0, 276.0, colors::WHITE);
            ground_profile(&mut c, &v, &f.ground, y0 + 276.0);
            let p = |a: [f64; 3]| Vec2::new(a[0], a[2]);
            let leg = |c: &mut Canvas, i: usize, col: Rgba, wpx: f32| {
                let l = f.legs[i];
                c.polyline(&v, &[p(l[0]), p(l[1]), p(l[2])], col, wpx);
                c.dot(&v, p(l[2]), wpx * 0.6, col);
            };
            for i in [1, 3] {
                leg(&mut c, i, Rgba::rgb(150, 160, 175), 6.0);
            }
            let (front, rear) = (p(f.legs[0][0]), p(f.legs[2][0]));
            let mid = Vec2::new((front.x + rear.x) / 2.0, (front.y + rear.y) / 2.0);
            let pitch = (front.y - rear.y).atan2(front.x - rear.x);
            c.oriented_box(&v, Pose2::new(mid.x, mid.y + 0.02, pitch), 0.5, 0.1, colors::series(k));
            for i in [0, 2] {
                leg(&mut c, i, Rgba::rgb(60, 70, 85), 7.0);
            }
            for i in 0..4 {
                if f.stance[i] && f.forces_n[i][2] > 1.0 {
                    let ft = p(f.legs[i][2]);
                    let s = 0.0015;
                    c.polyline(
                        &v,
                        &[ft, ft + Vec2::new(f.forces_n[i][0] * s, f.forces_n[i][2] * s)],
                        colors::RED.alpha(200),
                        3.0,
                    );
                }
            }
            if f.push_n[0].abs() > 0.0 {
                c.arrow(
                    &v,
                    Pose2::new(f.base_m[0] + 0.45, f.base_m[2] + 0.1, std::f64::consts::PI),
                    0.3,
                    colors::ORANGE,
                    5.0,
                );
                c.label(300.0, y0 + 60.0, "BACK PUSH 120 N", 2.0, colors::ORANGE);
            }
            if f.push_n[1].abs() > 0.0 {
                c.label(300.0, y0 + 60.0, "SIDE PUSH 100 N", 2.0, colors::ORANGE);
            }
            c.label(18.0, y0 + 8.0, &r.name.to_uppercase(), 2.0, colors::series(k));
            c.label(
                18.0,
                y0 + 30.0,
                &format!(
                    "T {:.1} S  CMD {:.1} M/S  VX {:.2}  ROLL {:+.0} PITCH {:+.0} DEG",
                    f.t,
                    f.v_cmd_mps,
                    f.velocity_mps[0],
                    f.rpy_rad[0].to_degrees(),
                    f.rpy_rad[1].to_degrees()
                ),
                1.5,
                colors::INK,
            );
            let m = |key: &str| r.metrics.iter().find(|(k, _)| k == key).map_or(f64::NAN, |(_, v)| *v);
            c.label(
                18.0,
                y0 + 48.0,
                &format!(
                    "RMS SPEED ERR {:.2} M/S  RMS TILT {:.1} DEG",
                    m("rms_speed_err_mps"),
                    m("rms_tilt_rad").to_degrees()
                ),
                1.5,
                colors::INK,
            );
            // top view of the base trace
            let tv = View::fit(Vec2::new(-0.3, -0.5), Vec2::new(5.5, 0.5), 660.0, y0, 230.0, 276.0);
            c.rect_px(660.0, y0, 230.0, 276.0, colors::WHITE);
            c.polyline(&tv, &[Vec2::new(-0.3, 0.0), Vec2::new(5.5, 0.0)], colors::GREY, 1.0);
            let tr: Vec<Vec2> = r.frames[..=fi.min(r.frames.len() - 1)]
                .iter()
                .map(|g| Vec2::new(g.base_m[0], g.base_m[1]))
                .collect();
            c.polyline(&tv, &tr, colors::series(k), 2.5);
            c.dot(&tv, Vec2::new(f.base_m[0], f.base_m[1]), 4.0, colors::series(k));
            c.text(668.0, y0 + 6.0, "TOP VIEW", 1.5, colors::INK);
        }
        gif.frame(&c, 6)?;
    }
    Ok(())
}

fn g1_gif(runs: &[g1::G1Run], file: &str, heading: &str) -> std::io::Result<()> {
    let (w, h) = (920u32, 600u32);
    let mut gif = GifWriter::create(format!("{OUT}/{file}.gif"), w, h)?;
    let n = runs.iter().map(|r| r.frames.len()).max().unwrap();
    for fi in (0..n).step_by(2) {
        let mut c = Canvas::new(w, h);
        title(&mut c, heading);
        for (k, r) in runs.iter().enumerate() {
            let f = &r.frames[fi.min(r.frames.len() - 1)];
            let x0 = 10.0 + k as f32 * 455.0;
            let cx = f.com[0];
            let gz = f.ground[18][1];
            let sv = View::fit(
                Vec2::new(cx - 0.9, gz - 0.05),
                Vec2::new(cx + 0.9, gz + 1.45),
                x0,
                36.0,
                445.0,
                330.0,
            );
            c.rect_px(x0, 36.0, 445.0, 330.0, colors::WHITE);
            ground_profile(&mut c, &sv, &f.ground, 366.0);
            let p = |a: [f64; 3]| Vec2::new(a[0], a[2]);
            // points: pelvis, left hip/knee/ankle, right hip/knee/ankle
            let pelvis = p(f.points[0]);
            for (side, col) in [(1usize, Rgba::rgb(150, 160, 175)), (0usize, Rgba::rgb(60, 70, 85))] {
                let b = 1 + 3 * side;
                let (hip, knee, ankle) = (p(f.points[b]), p(f.points[b + 1]), p(f.points[b + 2]));
                c.polyline(&sv, &[pelvis, hip, knee, ankle], col, 7.0);
                c.polyline(
                    &sv,
                    &[ankle + Vec2::new(-0.05, -0.03), ankle + Vec2::new(0.12, -0.03)],
                    col,
                    6.0,
                );
            }
            // torso (fused into the pelvis in the 12-DoF model)
            let top = p(f.torso_top);
            c.polyline(&sv, &[pelvis, top], colors::series(k), 14.0);
            let dir = Vec2::new(top.x - pelvis.x, top.y - pelvis.y);
            c.disc(&sv, top + dir * (0.1 / dir.norm().max(1e-6)), 0.08, colors::series(k));
            c.dot(&sv, Vec2::new(f.zmp[0], 0.0), 5.0, colors::RED);
            c.dot(&sv, Vec2::new(f.dcm[0], 0.0), 5.0, colors::GREEN);
            c.label(x0 + 8.0, 44.0, &r.name.to_uppercase(), 1.8, colors::series(k));
            c.label(
                x0 + 8.0,
                66.0,
                &format!("T {:.1} S  X {:.2} M", f.t, f.com[0]),
                1.5,
                colors::INK,
            );
            if f.pushing {
                c.label(x0 + 260.0, 100.0, "PUSH", 2.5, colors::ORANGE);
            }
            if f.fallen {
                c.label(x0 + 160.0, 160.0, "FELL", 4.0, colors::RED);
            }
            // top view
            let tv = View::fit(
                Vec2::new(cx - 0.8, -0.45),
                Vec2::new(cx + 0.8, 0.55),
                x0,
                372.0,
                445.0,
                222.0,
            );
            c.rect_px(x0, 372.0, 445.0, 222.0, colors::WHITE);
            for s in &f.steps {
                c.oriented_box(&tv, Pose2::new(s[0], s[1], 0.0), 0.17, 0.06, Rgba::rgb(190, 190, 190));
            }
            c.oriented_box(
                &tv,
                Pose2::new(f.next[0], f.next[1], 0.0),
                0.17,
                0.06,
                colors::PURPLE.alpha(110),
            );
            let lo = fi.min(r.frames.len() - 1);
            let com_tr: Vec<Vec2> = r.frames[..=lo].iter().map(|g| Vec2::new(g.com[0], g.com[1])).collect();
            let zmp_tr: Vec<Vec2> = r.frames[..=lo].iter().map(|g| Vec2::new(g.zmp[0], g.zmp[1])).collect();
            c.polyline(&tv, &zmp_tr, colors::RED.alpha(150), 1.5);
            c.polyline(&tv, &com_tr, colors::BLUE, 2.5);
            c.dot(&tv, Vec2::new(f.dcm[0], f.dcm[1]), 5.0, colors::GREEN);
            if k == 0 {
                c.label(
                    x0 + 8.0,
                    576.0,
                    "COM BLUE  ZMP RED  DCM GREEN  NEXT STEP PURPLE",
                    1.5,
                    colors::INK,
                );
            }
        }
        gif.frame(&c, 4)?;
    }
    Ok(())
}

fn main() -> std::io::Result<()> {
    let only: Vec<String> = std::env::args().skip(1).collect();
    let on = |n: &str| only.is_empty() || only.iter().any(|o| o == n);
    let mut md = String::new();
    if on("go2") {
        let q = rne_locomotion::Quadruped::go2().expect("Go2");
        let p = go2::go2_srb(&q);
        let cfg = go2::Go2Config::default();
        let mut ctrls: Vec<Box<dyn ForceController>> = vec![Box::new(PdForceQp::new(p)), Box::new(ConvexMpc3d::new(p))];
        let runs: Vec<go2::Go2Run> = ctrls.iter_mut().map(|c| go2::run(c.as_mut(), &cfg)).collect();
        go2_gif(
            &runs,
            "go2",
            "RNE UNITREE GO2: SPEED STEPS, BACK PUSH 3 S, SIDE PUSH 6 S",
        )?;
        let rows: Vec<_> = runs.iter().map(|r| (r.name.clone(), r.metrics.clone())).collect();
        let _ = writeln!(md, "## go2\n\n{}", table(&rows));
    }
    if on("g1") {
        let runs = vec![
            g1::run(Strategy::PreviewFixedSteps, 8.0, true),
            g1::run(Strategy::DcmStepAdjustment, 8.0, true),
        ];
        g1_gif(
            &runs,
            "g1",
            &format!(
                "RNE UNITREE G1 (12 DOF): PUSH ({:.2}, {:.2}) M/S AT {:.1} S",
                PUSH.1[0], PUSH.1[1], PUSH.0
            ),
        )?;
        let mut rows: Vec<_> = runs
            .iter()
            .map(|r| (format!("{} (push)", r.name), r.metrics.clone()))
            .collect();
        for s in [Strategy::PreviewFixedSteps, Strategy::DcmStepAdjustment] {
            let r = g1::run(s, 8.0, false);
            rows.push((format!("{} (no push)", r.name), r.metrics));
        }
        let r = g1::run_with(Strategy::DcmStepAdjustment, 8.0, true, g1::WbcBackend::ForceQp);
        rows.push((format!("{} (push, ForceQpWbc)", r.name), r.metrics));
        let _ = writeln!(md, "## g1\n\n{}", table(&rows));
    }
    if on("go2_terrain") {
        let q = rne_locomotion::Quadruped::go2().expect("Go2");
        let p = go2::go2_srb(&q);
        let cfg = go2::Go2Config {
            pushes: false,
            terrain_amplitude_m: 0.2,
            terrain_step_m: 0.04,
            ..Default::default()
        };
        let mut ctrls: Vec<Box<dyn ForceController>> = vec![Box::new(PdForceQp::new(p)), Box::new(ConvexMpc3d::new(p))];
        let runs: Vec<go2::Go2Run> = ctrls.iter_mut().map(|c| go2::run(c.as_mut(), &cfg)).collect();
        go2_gif(&runs, "go2_terrain", "RNE GO2 ON TERRACED ROUGH TERRAIN (+-10 CM)")?;
        let rows: Vec<_> = runs.iter().map(|r| (r.name.clone(), r.metrics.clone())).collect();
        let _ = writeln!(md, "## go2 terrain\n\n{}", table(&rows));
    }
    if on("g1_ramp") {
        let runs = vec![
            g1::run_on(
                Strategy::PreviewFixedSteps,
                9.0,
                true,
                g1::WbcBackend::Rne,
                g1::G1Terrain::Ramp,
            ),
            g1::run_on(
                Strategy::DcmStepAdjustment,
                9.0,
                true,
                g1::WbcBackend::Rne,
                g1::G1Terrain::Ramp,
            ),
        ];
        g1_gif(&runs, "g1_ramp", "RNE G1 UP A 5% RAMP, PUSHED AT 3.1 S")?;
        let rows: Vec<_> = runs
            .iter()
            .map(|r| (format!("{} (ramp, push)", r.name), r.metrics.clone()))
            .collect();
        let _ = writeln!(md, "## g1 ramp\n\n{}", table(&rows));
    }
    std::fs::create_dir_all(OUT)?;
    std::fs::write(format!("{OUT}/README.md"), &md)?;
    println!("{md}");
    Ok(())
}
