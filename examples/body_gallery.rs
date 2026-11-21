//! Animated GIFs of control **by robot body**: wheeled AMR, car, quadruped,
//! humanoid, arm, drone and cart-pole. Each GIF overlays (or stacks) the
//! controllers that fit that body on the same task.
//!
//! ```bash
//! cargo run --release --example body_gallery            # all -> out/bodies/*.gif + README.md
//! cargo run --release --example body_gallery humanoid   # just one
//! ```

use robo_control::scenarios::{self, arm, cartpole, drone, humanoid, quadruped, wheeled, Run};
use robo_control::sim::Log;
use robo_planning::render::{colors, Canvas, GifWriter, Rgba, View};
use robo_planning::{Pose2, Vec2};
use std::fmt::Write as _;

const OUT: &str = "out/bodies";
const DELAY: u16 = 6;

fn title(c: &mut Canvas, s: &str) {
    c.rect_px(0.0, 0.0, c.width() as f32, 30.0, colors::INK);
    c.text(10.0, 8.0, s, 2.0, colors::WHITE);
}

fn legend(c: &mut Canvas, x: f32, y: f32, rows: &[(String, Rgba)]) {
    let w = rows.iter().map(|(s, _)| Canvas::text_width(s, 1.5)).fold(0.0, f32::max) + 30.0;
    c.rect_px(x, y, w, rows.len() as f32 * 16.0 + 8.0, colors::WHITE.alpha(225));
    for (i, (s, col)) in rows.iter().enumerate() {
        let yy = y + 6.0 + i as f32 * 16.0;
        c.rect_px(x + 6.0, yy + 2.0, 14.0, 7.0, *col);
        c.text(x + 26.0, yy, s, 1.5, colors::INK);
    }
}

fn at(log: &Log, t: f64) -> usize {
    log.t
        .partition_point(|&x| x <= t)
        .saturating_sub(1)
        .min(log.t.len() - 1)
}

fn trail(log: &Log, t: f64, ix: usize, iy: usize) -> Vec<Vec2> {
    let k = at(log, t);
    log.x[..=k].iter().step_by(5).map(|x| Vec2::new(x[ix], x[iy])).collect()
}

fn short(s: &str) -> String {
    s.to_uppercase()
}

fn wheeled_gif(res: &wheeled::WheeledResult, name: &str, heading: &str, car: bool) -> std::io::Result<()> {
    let (w, h) = if car { (920u32, 520u32) } else { (800u32, 520u32) };
    let mut gif = GifWriter::create(format!("{OUT}/{name}.gif"), w, h)?;
    let main_w = if car { 590.0 } else { 780.0 };
    let v = View::fit(res.map.min, res.map.max, 10.0, 40.0, main_w, 470.0);
    let refline: Vec<Vec2> = res.reference.iter().map(|p| Vec2::new(p.p.x, p.p.y)).collect();
    let t_end = res.runs.iter().map(|r| *r.log.t.last().unwrap()).fold(0.0, f64::max);
    let n = 130;
    let rows: Vec<(String, Rgba)> = res
        .runs
        .iter()
        .enumerate()
        .map(|(i, r)| {
            (
                format!("{} RMS {:.2} M", short(&r.name), r.metric("rms_cte_m")),
                colors::series(i),
            )
        })
        .collect();
    for f in 0..=n + 10 {
        let t = t_end * f.min(n) as f64 / n as f64;
        let mut c = Canvas::new(w, h);
        title(&mut c, heading);
        c.map(&v, &res.map);
        c.polyline(&v, &refline, colors::GREY, 3.0);
        let draw_robot = |c: &mut Canvas, v: &View, i: usize, r: &Run| {
            let x = &r.log.x[at(&r.log, t)];
            let pose = Pose2::new(x[0], x[1], x[2]);
            if car {
                let centre = Pose2::new(x[0] + 1.35 * x[2].cos(), x[1] + 1.35 * x[2].sin(), x[2]);
                c.oriented_box(v, centre, 4.4, 1.8, colors::series(i).alpha(200));
            } else {
                c.disc_robot(v, pose, 0.35, colors::series(i).alpha(220));
            }
        };
        for (i, r) in res.runs.iter().enumerate() {
            c.polyline(&v, &trail(&r.log, t, 0, 1), colors::series(i), 1.5);
            draw_robot(&mut c, &v, i, r);
        }
        if car {
            // zoom on the MPC car, drawn on its own canvas so it clips to the panel
            let x = &res.runs[3].log.x[at(&res.runs[3].log, t)];
            let mut z = Canvas::new(300, 400);
            z.clear(colors::WHITE);
            let zv = View::fit(
                Vec2::new(x[0] - 9.0, x[1] - 12.0),
                Vec2::new(x[0] + 9.0, x[1] + 12.0),
                0.0,
                0.0,
                300.0,
                400.0,
            );
            z.polyline(&zv, &refline, colors::GREY, 4.0);
            for (i, r) in res.runs.iter().enumerate() {
                z.polyline(&zv, &trail(&r.log, t, 0, 1), colors::series(i), 2.0);
                draw_robot(&mut z, &zv, i, r);
            }
            z.polyline_px(
                &[(0.5, 0.5), (299.5, 0.5), (299.5, 399.5), (0.5, 399.5), (0.5, 0.5)],
                colors::INK,
                1.5,
            );
            z.label(8.0, 8.0, "ZOOM (FOLLOWS MPC CAR)", 1.5, colors::INK);
            c.blit(&z, 610, 40);
            c.text(
                618.0,
                452.0,
                &format!("T {t:4.1} S  V {:4.1} M/S", x[3]),
                1.8,
                colors::INK,
            );
            legend(&mut c, 14.0, 44.0, &rows);
        } else {
            legend(&mut c, 14.0, 44.0, &rows);
            c.text(w as f32 - 130.0, 46.0, &format!("T {t:4.1} S"), 1.5, colors::INK);
        }
        gif.frame(&c, DELAY)?;
    }
    Ok(())
}

fn quadruped_gif(runs: &[quadruped::QuadRun]) -> std::io::Result<()> {
    let p = quadruped::QuadrupedParams::default();
    let (w, h) = (900u32, 560u32);
    let mut gif = GifWriter::create(format!("{OUT}/quadruped.gif"), w, h)?;
    let nf = runs[0].frames.len();
    for fi in (0..nf).step_by(3) {
        let mut c = Canvas::new(w, h);
        title(&mut c, "QUADRUPED TROT: SPEED STEPS + BACKWARD PUSH AT 2.5 S");
        for (k, r) in runs.iter().enumerate() {
            let f = &r.frames[fi];
            let y0 = 36.0 + k as f32 * 262.0;
            let cx = f.x[0];
            let v = View::fit(
                Vec2::new(cx - 1.4, -0.12),
                Vec2::new(cx + 1.4, 0.62),
                10.0,
                y0,
                880.0,
                250.0,
            );
            c.rect_px(10.0, y0, 880.0, 250.0, colors::WHITE);
            // ground with moving ticks
            let (gx0, gy) = v.px(Vec2::new(cx - 1.4, 0.0));
            let (gx1, _) = v.px(Vec2::new(cx + 1.4, 0.0));
            c.rect_px(gx0, gy, gx1 - gx0, 250.0 + y0 - gy, Rgba::rgb(225, 220, 205));
            let mut tick = (cx - 1.4).ceil();
            while tick < cx + 1.4 {
                let (tx, ty) = v.px(Vec2::new(tick, 0.0));
                c.rect_px(tx - 1.0, ty, 2.0, 8.0, colors::INK);
                tick += 0.25;
            }
            let (th, bx, bz) = (f.x[2], f.x[0], f.x[1]);
            let hip = |i: usize| {
                let s = if i < 2 { 1.0 } else { -1.0 };
                Vec2::new(bx + s * p.hip_offset_m * th.cos(), bz + s * p.hip_offset_m * th.sin())
            };
            let leg = |c: &mut Canvas, i: usize, col: Rgba, width: f32| {
                let hp = hip(i);
                let ft = Vec2::new(f.feet[i][0], f.feet[i][1]);
                let d = hp.dist(ft).min(2.0 * p.link_m - 1e-6);
                let ang = (ft.y - hp.y).atan2(ft.x - hp.x);
                let a = (d / (2.0 * p.link_m)).acos();
                // knee bends backwards (towards -x)
                let knee = hp + Vec2::new((ang - a).cos(), (ang - a).sin()) * p.link_m;
                let knee = if knee.x > hp.x.max(ft.x) {
                    hp + Vec2::new((ang + a).cos(), (ang + a).sin()) * p.link_m
                } else {
                    knee
                };
                c.polyline(&v, &[hp, knee, ft], col, width);
                c.dot(&v, ft, width * 0.6, col);
            };
            // far legs (FR, RR) lighter, near legs (FL, RL) darker
            for i in [1, 3] {
                leg(&mut c, i, Rgba::rgb(150, 160, 175), 6.0);
            }
            c.oriented_box(&v, Pose2::new(bx, bz, th), 0.62, 0.12, colors::series(k));
            for i in [0, 2] {
                leg(&mut c, i, Rgba::rgb(60, 70, 85), 7.0);
            }
            for i in 0..4 {
                if f.stance[i] && f.forces[i][1] > 1.0 {
                    let ft = Vec2::new(f.feet[i][0], f.feet[i][1]);
                    let s = 0.0012;
                    c.polyline(
                        &v,
                        &[ft, ft + Vec2::new(f.forces[i][0] * s, f.forces[i][1] * s)],
                        colors::RED.alpha(200),
                        3.0,
                    );
                }
            }
            if f.push_n.abs() > 0.0 {
                let pb = Vec2::new(bx + 0.5, bz + 0.12);
                c.arrow(
                    &v,
                    Pose2::new(pb.x, pb.y, std::f64::consts::PI),
                    0.35,
                    colors::ORANGE,
                    5.0,
                );
                c.label(v.px(pb).0 - 40.0, v.px(pb).1 - 40.0, "PUSH", 2.0, colors::ORANGE);
            }
            c.label(18.0, y0 + 8.0, &short(&r.run.name), 2.0, colors::series(k));
            c.label(
                18.0,
                y0 + 32.0,
                &format!(
                    "T {:.1} S  CMD {:.1} M/S  VX {:.2} M/S  PITCH {:+.0} DEG",
                    f.t,
                    f.v_des,
                    f.x[3],
                    f.x[2].to_degrees()
                ),
                1.5,
                colors::INK,
            );
            c.label(
                18.0,
                y0 + 50.0,
                &format!("RMS SPEED ERR {:.2} M/S", r.run.metric("rms_speed_err_mps")),
                1.5,
                colors::INK,
            );
            if let Some(tf) = r.fell_at_s {
                if f.t >= tf {
                    c.label(400.0, y0 + 100.0, "FELL", 4.0, colors::RED);
                }
            }
        }
        gif.frame(&c, 6)?;
    }
    Ok(())
}

fn humanoid_gif(runs: &[humanoid::HumanoidRun]) -> std::io::Result<()> {
    let p = humanoid::LipmParams::default();
    let (w, h) = (920u32, 600u32);
    let mut gif = GifWriter::create(format!("{OUT}/humanoid.gif"), w, h)?;
    let nf = runs[0].frames.len();
    for fi in (0..nf).step_by(3) {
        let mut c = Canvas::new(w, h);
        title(&mut c, "HUMANOID LIPM WALK: PUSH (0.35, 0.25) M/S AT 3.1 S");
        for (k, r) in runs.iter().enumerate() {
            let f = &r.frames[fi];
            let x0 = 10.0 + k as f32 * 455.0;
            let cx = f.com[0];
            // side view
            let sv = View::fit(
                Vec2::new(cx - 1.1, -0.1),
                Vec2::new(cx + 1.1, 1.75),
                x0,
                36.0,
                445.0,
                330.0,
            );
            c.rect_px(x0, 36.0, 445.0, 330.0, colors::WHITE);
            let (gx0, gy) = sv.px(Vec2::new(cx - 1.1, 0.0));
            c.rect_px(gx0, gy, 445.0, 366.0 - gy, Rgba::rgb(225, 220, 205));
            let mut tick = (cx - 1.1).ceil();
            while tick < cx + 1.1 {
                let (tx, ty) = sv.px(Vec2::new(tick, 0.0));
                c.rect_px(tx - 1.0, ty, 2.0, 8.0, colors::INK);
                tick += 0.25;
            }
            // after a fall the whole figure topples about the stance foot
            let tilt = if f.fallen {
                ((f.t - r.fell_at_s.unwrap()) * 2.5).min(1.4) * (f.dcm[0] - f.stance[0]).signum()
            } else {
                0.0
            };
            let stance = Vec2::new(f.stance[0], 0.0);
            let rot = |q: Vec2| {
                let d = q - stance;
                stance
                    + Vec2::new(
                        d.x * tilt.cos() + d.y * tilt.sin(),
                        -d.x * tilt.sin() + d.y * tilt.cos(),
                    )
            };
            let fig_x = if f.fallen { f.stance[0] } else { cx };
            let hip = rot(Vec2::new(fig_x, p.com_height_m));
            let (lo, tg, u) = f.swing;
            let sw = rot(Vec2::new(
                lo[0] + (tg[0] - lo[0]) * (0.5 - 0.5 * (std::f64::consts::PI * u).cos()),
                0.08 * (std::f64::consts::PI * u).sin(),
            ));
            let leg = |c: &mut Canvas, foot: Vec2, col: Rgba| {
                let d = hip.dist(foot).min(0.9 - 1e-6);
                let ang = (foot.y - hip.y).atan2(foot.x - hip.x);
                let a = (d / 0.9).acos();
                let knee = hip + Vec2::new((ang + a).cos(), (ang + a).sin()) * 0.45;
                c.polyline(&sv, &[hip, knee, foot], col, 7.0);
                c.polyline(
                    &sv,
                    &[foot + Vec2::new(-0.06, 0.0), foot + Vec2::new(0.14, 0.0)],
                    col,
                    6.0,
                );
            };
            leg(&mut c, sw, Rgba::rgb(150, 160, 175));
            let torso_top = rot(Vec2::new(fig_x, p.com_height_m + 0.5));
            c.polyline(&sv, &[hip, torso_top], colors::series(k), 14.0);
            c.disc(
                &sv,
                torso_top + Vec2::new(0.13 * tilt.sin(), 0.13 * tilt.cos()),
                0.1,
                colors::series(k),
            );
            leg(&mut c, stance, Rgba::rgb(60, 70, 85));
            c.dot(&sv, Vec2::new(f.zmp[0], 0.0), 5.0, colors::RED);
            c.dot(&sv, Vec2::new(f.dcm[0], 0.0), 5.0, colors::GREEN);
            c.label(x0 + 8.0, 44.0, &short(&r.run.name), 1.8, colors::series(k));
            c.label(x0 + 8.0, 66.0, &format!("T {:.1} S", f.t), 1.5, colors::INK);
            if (humanoid::PUSH.0..humanoid::PUSH.0 + 0.25).contains(&f.t) {
                let pb = Vec2::new(cx - 0.55, 1.0);
                c.arrow(&sv, Pose2::new(pb.x, pb.y, 0.0), 0.35, colors::ORANGE, 5.0);
                c.label(sv.px(pb).0, sv.px(pb).1 - 30.0, "PUSH", 2.0, colors::ORANGE);
            }
            if f.fallen {
                c.label(x0 + 150.0, 150.0, "FELL", 4.0, colors::RED);
            }
            // top view
            let tv = View::fit(
                Vec2::new(cx - 1.1, -0.55),
                Vec2::new(cx + 1.1, 0.75),
                x0,
                372.0,
                445.0,
                222.0,
            );
            c.rect_px(x0, 372.0, 445.0, 222.0, colors::WHITE);
            for s in &f.steps {
                c.oriented_box(
                    &tv,
                    Pose2::new(s[0], s[1], 0.0),
                    p.foot_size_m.0,
                    p.foot_size_m.1,
                    Rgba::rgb(200, 200, 200),
                );
            }
            c.oriented_box(
                &tv,
                Pose2::new(f.stance[0], f.stance[1], 0.0),
                p.foot_size_m.0,
                p.foot_size_m.1,
                Rgba::rgb(120, 120, 120),
            );
            c.oriented_box(
                &tv,
                Pose2::new(tg[0], tg[1], 0.0),
                p.foot_size_m.0,
                p.foot_size_m.1,
                colors::PURPLE.alpha(110),
            );
            let com_tr: Vec<Vec2> = r.frames[..=fi].iter().map(|g| Vec2::new(g.com[0], g.com[1])).collect();
            let zmp_tr: Vec<Vec2> = r.frames[..=fi].iter().map(|g| Vec2::new(g.zmp[0], g.zmp[1])).collect();
            c.polyline(&tv, &zmp_tr, colors::RED.alpha(160), 1.5);
            c.polyline(&tv, &com_tr, colors::BLUE, 2.5);
            c.dot(&tv, Vec2::new(f.dcm[0], f.dcm[1]), 5.0, colors::GREEN);
            c.dot(&tv, Vec2::new(f.com[0], f.com[1]), 5.0, colors::BLUE);
            if k == 0 {
                legend(
                    &mut c,
                    x0 + 6.0,
                    530.0,
                    &[("COM".into(), colors::BLUE), ("ZMP".into(), colors::RED)],
                );
                legend(
                    &mut c,
                    x0 + 160.0,
                    530.0,
                    &[("DCM".into(), colors::GREEN), ("NEXT STEP".into(), colors::PURPLE)],
                );
            }
        }
        gif.frame(&c, 6)?;
    }
    Ok(())
}

fn arm_gif(res: &arm::ArmResult) -> std::io::Result<()> {
    let (w, h) = (760u32, 540u32);
    let mut gif = GifWriter::create(format!("{OUT}/arm.gif"), w, h)?;
    let v = View::fit(res.map.min, res.map.max, 10.0, 40.0, 740.0, 490.0);
    let t_end = *res.runs[0].log.t.last().unwrap();
    let n = 110;
    let rows: Vec<(String, Rgba)> = std::iter::once(("REFERENCE".to_string(), colors::GREY))
        .chain(res.runs.iter().enumerate().map(|(i, r)| {
            (
                format!("{} RMS {:.3} M", short(&r.name), r.metric("rms_ee_err_m")),
                colors::series(i),
            )
        }))
        .collect();
    for f in 0..=n + 10 {
        let t = t_end * f.min(n) as f64 / n as f64;
        let mut c = Canvas::new(w, h);
        title(
            &mut c,
            &format!("ARM PICK+PLACE, {} KG PAYLOAD UNKNOWN TO CONTROLLER", res.payload_kg),
        );
        c.map(&v, &res.map);
        let (q, _, _) = res.reference.sample(t);
        c.polyline(&v, &res.arm.fk(&q), colors::GREY, 10.0);
        for (i, r) in res.runs.iter().enumerate() {
            let x = &r.log.x[at(&r.log, t)];
            let q: Vec<f64> = x.rows(0, 3).iter().copied().collect();
            let pts = res.arm.fk(&q);
            let ee: Vec<Vec2> = r.log.x[..=at(&r.log, t)]
                .iter()
                .step_by(20)
                .map(|x| res.arm.end_effector(&[x[0], x[1], x[2]]))
                .collect();
            c.polyline(&v, &ee, colors::series(i).alpha(150), 1.5);
            c.polyline(&v, &pts, colors::series(i).alpha(200), 6.0);
            c.oriented_box(
                &v,
                Pose2::new(pts[3].x, pts[3].y - 0.08, 0.0),
                0.2,
                0.16,
                colors::ORANGE.alpha(200),
            );
        }
        c.dot(&v, res.pick_place.0, 5.0, colors::GREEN);
        c.dot(&v, res.pick_place.1, 5.0, colors::RED);
        legend(&mut c, w as f32 - 290.0, 44.0, &rows);
        c.text(20.0, 46.0, &format!("T {t:.2} S"), 1.5, colors::INK);
        gif.frame(&c, DELAY)?;
    }
    Ok(())
}

fn drone_gif(res: &drone::DroneResult) -> std::io::Result<()> {
    let (w, h) = (860u32, 500u32);
    let mut gif = GifWriter::create(format!("{OUT}/drone.gif"), w, h)?;
    let v = View::fit(res.map.min, res.map.max, 10.0, 40.0, 840.0, 450.0);
    let refline: Vec<Vec2> = res.reference.iter().map(|p| Vec2::new(p.p.x, p.p.y)).collect();
    let t_end = *res.runs[0].log.t.last().unwrap();
    let n = 120;
    let rows: Vec<(String, Rgba)> = res
        .runs
        .iter()
        .enumerate()
        .map(|(i, r)| {
            (
                format!("{} MAX {:.2} M", short(&r.name), r.metric("max_path_err_m")),
                colors::series(i),
            )
        })
        .collect();
    for f in 0..=n + 10 {
        let t = t_end * f.min(n) as f64 / n as f64;
        let mut c = Canvas::new(w, h);
        title(&mut c, "DRONE (X-Z) THROUGH A FOREST WITH A WIND GUST");
        c.map(&v, &res.map);
        c.polyline(&v, &refline, colors::GREY, 3.0);
        if t >= res.gust.0 && t < res.gust.1 {
            for k in 0..6 {
                let y = 1.0 + k as f64 * 1.1;
                let x = 1.0 + (t * 6.0 + k as f64 * 1.7) % 14.0;
                c.arrow(&v, Pose2::new(x, y, -0.45), 0.9, Rgba::rgb(120, 180, 230), 3.0);
            }
            c.label(w as f32 - 160.0, 44.0, "WIND GUST", 2.0, Rgba::rgb(60, 130, 200));
        }
        for (i, r) in res.runs.iter().enumerate() {
            c.polyline(&v, &trail(&r.log, t, 0, 1), colors::series(i), 1.5);
            let x = &r.log.x[at(&r.log, t)];
            let (cx, cz, th) = (x[0], x[1], x[2]);
            let arm = Vec2::new(0.3 * th.cos(), 0.3 * th.sin());
            let ctr = Vec2::new(cx, cz);
            c.polyline(&v, &[ctr - arm, ctr + arm], colors::series(i), 5.0);
            let up = Vec2::new(-0.08 * th.sin(), 0.08 * th.cos());
            for s in [-1.0, 1.0] {
                let r0 = ctr + arm * s + up;
                c.polyline(&v, &[r0 - arm * 0.45, r0 + arm * 0.45], colors::INK, 3.0);
            }
        }
        legend(&mut c, 14.0, 44.0, &rows);
        gif.frame(&c, DELAY)?;
    }
    Ok(())
}

fn cartpole_gif(res: &cartpole::CartPoleResult) -> std::io::Result<()> {
    let (w, h) = (900u32, 500u32);
    let mut gif = GifWriter::create(format!("{OUT}/cartpole.gif"), w, h)?;
    let l = res.model.pole_length_m;
    let t_end = *res.runs[0].log.t.last().unwrap();
    let n = 160;
    for f in 0..=n + 10 {
        let t = t_end * f.min(n) as f64 / n as f64;
        let mut c = Canvas::new(w, h);
        title(&mut c, "CART-POLE SWING-UP FROM HANGING, PUSH AT 6 S");
        for (k, r) in res.runs.iter().enumerate() {
            let y0 = 36.0 + k as f32 * 232.0;
            let v = View::fit(Vec2::new(-2.5, -0.62), Vec2::new(2.5, 0.62), 10.0, y0, 880.0, 226.0);
            c.rect_px(10.0, y0, 880.0, 226.0, colors::WHITE);
            c.polyline(&v, &[Vec2::new(-2.45, -0.08), Vec2::new(2.45, -0.08)], colors::INK, 2.0);
            let x = &r.log.x[at(&r.log, t)];
            let cart = Vec2::new(x[0], 0.0);
            let tip = cart + Vec2::new(l * x[2].sin(), l * x[2].cos());
            let tr: Vec<Vec2> = r.log.x[at(&r.log, (t - 0.6).max(0.0))..=at(&r.log, t)]
                .iter()
                .map(|x| Vec2::new(x[0] + l * x[2].sin(), l * x[2].cos()))
                .collect();
            c.polyline(&v, &tr, colors::RED.alpha(90), 2.0);
            c.oriented_box(&v, Pose2::new(cart.x, cart.y, 0.0), 0.36, 0.16, colors::series(k));
            c.polyline(&v, &[cart, tip], colors::INK, 6.0);
            c.disc(&v, tip, 0.05, colors::RED);
            let u = r.log.u[at(&r.log, t)][0];
            if u.abs() > 0.5 {
                c.arrow(
                    &v,
                    Pose2::new(cart.x, -0.2, if u >= 0.0 { 0.0 } else { std::f64::consts::PI }),
                    u.abs() / 20.0 * 0.6,
                    colors::ORANGE,
                    3.0,
                );
            }
            if (res.push_t_s..res.push_t_s + 0.3).contains(&t) {
                c.label(700.0, y0 + 10.0, "PUSH", 2.0, colors::ORANGE);
            }
            c.label(18.0, y0 + 8.0, &short(&r.name), 2.0, colors::series(k));
            c.label(
                18.0,
                y0 + 30.0,
                &format!(
                    "T {t:.1} S  ANGLE {:+4.0} DEG  FORCE {:+5.1} N",
                    robo_control::model::wrap(x[2]).to_degrees(),
                    u
                ),
                1.5,
                colors::INK,
            );
            c.label(
                18.0,
                y0 + 48.0,
                &format!(
                    "UPRIGHT AT {:.2} S, CART TRAVEL {:.2} M",
                    r.metric("time_to_upright_s"),
                    r.metric("max_cart_travel_m")
                ),
                1.5,
                colors::INK,
            );
        }
        gif.frame(&c, 5)?;
    }
    Ok(())
}

fn main() -> std::io::Result<()> {
    let only: Vec<String> = std::env::args().skip(1).collect();
    let on = |n: &str| only.is_empty() || only.iter().any(|o| o == n);
    let mut md = String::new();
    if on("amr") {
        let r = wheeled::amr_warehouse();
        wheeled_gif(&r, "amr", "DIFF-DRIVE AMR: A* PLAN, 4 TRACKING CONTROLLERS", false)?;
        let _ = writeln!(md, "## amr\n\n{}", scenarios::table(&r.runs));
    }
    if on("car") {
        let r = wheeled::car_course();
        wheeled_gif(&r, "car", "CAR (BICYCLE + STEER LAG) UP TO 12 M/S", true)?;
        let _ = writeln!(md, "## car\n\n{}", scenarios::table(&r.runs));
    }
    if on("quadruped") {
        let r = quadruped::trot();
        quadruped_gif(&r)?;
        let runs: Vec<Run> = r.iter().map(|q| q.run.clone()).collect();
        let _ = writeln!(md, "## quadruped\n\n{}", scenarios::table(&runs));
    }
    if on("humanoid") {
        let r = humanoid::push_recovery();
        humanoid_gif(&r)?;
        let runs: Vec<Run> = r.iter().map(|q| q.run.clone()).collect();
        let _ = writeln!(md, "## humanoid\n\n{}", scenarios::table(&runs));
    }
    if on("arm") {
        let r = arm::pick_place();
        arm_gif(&r)?;
        let _ = writeln!(md, "## arm\n\n{}", scenarios::table(&r.runs));
    }
    if on("drone") {
        let r = drone::forest_flight();
        drone_gif(&r)?;
        let _ = writeln!(md, "## drone\n\n{}", scenarios::table(&r.runs));
    }
    if on("cartpole") {
        let r = cartpole::swing_up();
        cartpole_gif(&r)?;
        let _ = writeln!(md, "## cartpole\n\n{}", scenarios::table(&r.runs));
    }
    std::fs::create_dir_all(OUT)?;
    std::fs::write(format!("{OUT}/README.md"), &md)?;
    println!("{md}");
    Ok(())
}
