//! Quadruped (dog-type) trotting in the sagittal plane.
//!
//! The body is a single rigid body `[x, z, pitch, vx, vz, pitch_rate]`; the four
//! massless legs are projected onto the sagittal plane (front pair / rear pair).
//! A trot alternates diagonal pairs, so each phase has one front and one rear
//! foot on the ground. Ground reaction forces commanded by the controller are
//! applied after clipping to unilateral contact and the friction cone; feet in
//! stance do not slip. Swing feet follow a Raibert foot-placement target.

use super::Run;
use crate::metrics::rms;
use crate::mpc::box_qp;
use nalgebra::{DMatrix, DVector};

/// Robot and gait parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuadrupedParams {
    /// body mass [kg]
    pub mass_kg: f64,
    /// pitch inertia [kg m^2]
    pub inertia_kgm2: f64,
    /// hip offset from the body centre along the body [m]
    pub hip_offset_m: f64,
    /// nominal body height [m]
    pub height_m: f64,
    /// thigh and calf length [m]
    pub link_m: f64,
    /// trot period [s]
    pub period_s: f64,
    /// friction coefficient
    pub mu: f64,
    /// max normal force per foot [N]
    pub fz_max_n: f64,
    /// swing apex height [m]
    pub swing_height_m: f64,
}

impl Default for QuadrupedParams {
    fn default() -> Self {
        Self {
            mass_kg: 15.0,
            inertia_kgm2: 0.5,
            hip_offset_m: 0.22,
            height_m: 0.30,
            link_m: 0.22,
            period_s: 0.36,
            mu: 0.6,
            fz_max_n: 350.0,
            swing_height_m: 0.08,
        }
    }
}

const G: f64 = 9.81;
/// Legs: 0 front-left, 1 front-right, 2 rear-left, 3 rear-right.
pub const LEG_NAMES: [&str; 4] = ["FL", "FR", "RL", "RR"];

fn hip_sign(leg: usize) -> f64 {
    if leg < 2 {
        1.0
    } else {
        -1.0
    }
}

/// Stance flags for gait time `t`: phase A = {FL, RR}, phase B = {FR, RL}.
pub fn stance(p: &QuadrupedParams, t: f64) -> [bool; 4] {
    let a = (t / (p.period_s / 2.0)).floor() as i64 % 2 == 0;
    [a, !a, !a, a]
}

/// Time left in the current half-cycle.
fn phase_left(p: &QuadrupedParams, t: f64) -> f64 {
    let h = p.period_s / 2.0;
    h - t.rem_euclid(h)
}

/// One recorded frame for drawing.
#[derive(Debug, Clone)]
pub struct QuadFrame {
    /// time [s]
    pub t: f64,
    /// body state
    pub x: [f64; 6],
    /// foot positions (x, z)
    pub feet: [[f64; 2]; 4],
    /// applied ground forces (fx, fz)
    pub forces: [[f64; 2]; 4],
    /// stance flags
    pub stance: [bool; 4],
    /// commanded forward speed [m/s]
    pub v_des: f64,
    /// external push force on the body [N]
    pub push_n: f64,
}

/// Force controller interface.
pub trait ForceController {
    /// Desired foot forces given time, body state, feet, stance and speed command.
    fn forces(
        &mut self,
        t: f64,
        x: &[f64; 6],
        feet: &[[f64; 2]; 4],
        st: [bool; 4],
        v_des: f64,
        targets: &[[f64; 2]; 4],
    ) -> [[f64; 2]; 4];
}

fn lever(x: &[f64; 6], foot: [f64; 2]) -> [f64; 2] {
    [foot[0] - x[0], foot[1] - x[1]]
}

/// Desired-wrench PD with QP force distribution over the stance feet.
#[derive(Debug, Clone)]
pub struct PdForceQp {
    /// parameters
    pub p: QuadrupedParams,
    /// speed gain [1/s]
    pub kv: f64,
    /// height stiffness / damping
    pub kz: (f64, f64),
    /// pitch stiffness / damping
    pub kth: (f64, f64),
}

impl ForceController for PdForceQp {
    fn forces(
        &mut self,
        _t: f64,
        x: &[f64; 6],
        feet: &[[f64; 2]; 4],
        st: [bool; 4],
        v_des: f64,
        _: &[[f64; 2]; 4],
    ) -> [[f64; 2]; 4] {
        let p = &self.p;
        let w = [
            p.mass_kg * self.kv * (v_des - x[3]),
            p.mass_kg * (G + self.kz.0 * (p.height_m - x[1]) - self.kz.1 * x[4]),
            p.inertia_kgm2 * (self.kth.0 * (-x[2]) - self.kth.1 * x[5]),
        ];
        let mut a = DMatrix::zeros(3, 8);
        for i in 0..4 {
            let r = lever(x, feet[i]);
            a[(0, 2 * i)] = 1.0;
            a[(1, 2 * i + 1)] = 1.0;
            a[(2, 2 * i)] = -r[1];
            a[(2, 2 * i + 1)] = r[0];
        }
        let s = DMatrix::from_diagonal(&DVector::from_vec(vec![1.0, 1.0, 10.0]));
        let wv = DVector::from_vec(w.to_vec());
        let h = a.transpose() * &s * &a + DMatrix::identity(8, 8) * 1e-4;
        let g = -(a.transpose() * &s * wv);
        let (mut lo, mut hi) = (DVector::zeros(8), DVector::zeros(8));
        let fx_max = p.mu * p.mass_kg * G / 2.0;
        for i in 0..4 {
            if st[i] {
                lo[2 * i] = -fx_max;
                hi[2 * i] = fx_max;
                lo[2 * i + 1] = 5.0;
                hi[2 * i + 1] = p.fz_max_n;
            }
        }
        let f = box_qp(&h, &g, &lo, &hi, DVector::zeros(8), 200);
        std::array::from_fn(|i| [f[2 * i], f[2 * i + 1]])
    }
}

/// Convex MPC on the linearised single-rigid-body model over one gait cycle,
/// with the contact schedule and planned footholds known in advance.
#[derive(Debug, Clone)]
pub struct ConvexMpc {
    /// parameters
    pub p: QuadrupedParams,
    /// horizon [steps]
    pub horizon: usize,
    /// model step [s]
    pub dt_s: f64,
    /// state weights `[x, z, pitch, vx, vz, pitch_rate]`
    pub q: [f64; 6],
    /// force weight
    pub r: f64,
    warm: Option<DVector<f64>>,
}

impl ConvexMpc {
    /// Default tuning: 12 steps of 30 ms (one trot cycle).
    pub fn new(p: QuadrupedParams) -> Self {
        Self {
            p,
            horizon: 12,
            dt_s: 0.03,
            q: [0.0, 50.0, 50.0, 20.0, 1.0, 1.0],
            r: 1e-6,
            warm: None,
        }
    }
}

impl ForceController for ConvexMpc {
    fn forces(
        &mut self,
        t: f64,
        x: &[f64; 6],
        feet: &[[f64; 2]; 4],
        _st: [bool; 4],
        v_des: f64,
        targets: &[[f64; 2]; 4],
    ) -> [[f64; 2]; 4] {
        let (n, dt, p) = (self.horizon, self.dt_s, self.p);
        let nu = 8;
        // reference along the horizon
        let xr = |k: usize| [x[0] + v_des * dt * k as f64, p.height_m, 0.0, v_des, 0.0, 0.0];
        // condensed prediction: X = Sx x0 + Su U + Sd
        let mut sx = DMatrix::zeros(6 * n, 6);
        let mut su = DMatrix::zeros(6 * n, nu * n);
        let mut sd = DVector::zeros(6 * n);
        let mut a = DMatrix::<f64>::identity(6, 6);
        for i in 0..3 {
            a[(i, i + 3)] = dt;
        }
        let mut phi = DMatrix::<f64>::identity(6, 6);
        let mut gam = DMatrix::<f64>::zeros(6, nu * n);
        let mut dvec = DVector::<f64>::zeros(6);
        let mut lo = DVector::zeros(nu * n);
        let mut hi = DVector::zeros(nu * n);
        let fx_max = p.mu * p.mass_kg * G / 2.0;
        for k in 0..n {
            let tk = t + k as f64 * dt;
            let st = stance(&p, tk);
            let mut b = DMatrix::zeros(6, nu);
            let c = xr(k);
            for i in 0..4 {
                if !st[i] {
                    continue;
                }
                // current stance feet stay; feet that land later use their target
                let foot = if stance(&p, t)[i] && phase_left(&p, t) > tk - t {
                    feet[i]
                } else {
                    targets[i]
                };
                let r = [foot[0] - c[0], foot[1] - c[1]];
                b[(3, 2 * i)] = dt / p.mass_kg;
                b[(4, 2 * i + 1)] = dt / p.mass_kg;
                b[(5, 2 * i)] = -r[1] * dt / p.inertia_kgm2;
                b[(5, 2 * i + 1)] = r[0] * dt / p.inertia_kgm2;
                lo[nu * k + 2 * i] = -fx_max;
                hi[nu * k + 2 * i] = fx_max;
                lo[nu * k + 2 * i + 1] = 5.0;
                hi[nu * k + 2 * i + 1] = p.fz_max_n;
            }
            let mut gconst = DVector::zeros(6);
            gconst[4] = -G * dt;
            phi = &a * phi;
            gam = &a * gam;
            gam.view_mut((0, nu * k), (6, nu)).copy_from(&b);
            dvec = &a * dvec + gconst;
            sx.view_mut((6 * k, 0), (6, 6)).copy_from(&phi);
            su.view_mut((6 * k, 0), (6, nu * n)).copy_from(&gam);
            sd.rows_mut(6 * k, 6).copy_from(&dvec);
        }
        let mut qbar = DMatrix::zeros(6 * n, 6 * n);
        let mut xref = DVector::zeros(6 * n);
        for k in 0..n {
            let c = xr(k + 1);
            for i in 0..6 {
                qbar[(6 * k + i, 6 * k + i)] = self.q[i];
                xref[6 * k + i] = c[i];
            }
        }
        let x0 = DVector::from_row_slice(x);
        let sut_q = su.transpose() * &qbar;
        let h = &sut_q * &su + DMatrix::identity(nu * n, nu * n) * self.r;
        let g = &sut_q * (&sx * &x0 + &sd - &xref);
        let z0 = match &self.warm {
            Some(w) => {
                let mut z = DVector::zeros(nu * n);
                z.rows_mut(0, nu * (n - 1)).copy_from(&w.rows(nu, nu * (n - 1)));
                z
            }
            None => DVector::zeros(nu * n),
        };
        let z = box_qp(&h, &g, &lo, &hi, z0, 300);
        self.warm = Some(z.clone());
        std::array::from_fn(|i| [z[2 * i], z[2 * i + 1]])
    }
}

/// Simulation result for one controller.
#[derive(Debug, Clone)]
pub struct QuadRun {
    /// summary
    pub run: Run,
    /// frames for drawing (every 10 ms)
    pub frames: Vec<QuadFrame>,
    /// time of falling, if it fell
    pub fell_at_s: Option<f64>,
}

/// Speed command: stand, then trot at 0.6 m/s, then 1.0 m/s.
pub fn speed_command(t: f64) -> f64 {
    if t < 1.0 {
        0.0
    } else if t < 4.0 {
        0.6 * ((t - 1.0) / 0.5).min(1.0)
    } else {
        0.6 + 0.4 * ((t - 4.0) / 0.5).min(1.0)
    }
}

/// Backward push applied to the body.
pub fn push_force(t: f64) -> f64 {
    if (2.5..2.65).contains(&t) {
        -160.0
    } else {
        0.0
    }
}

/// Simulates a controller for `duration_s` at 2 ms, controller at `ctrl_every` steps.
pub fn simulate_quadruped(
    p: QuadrupedParams,
    ctrl: &mut dyn ForceController,
    ctrl_every: usize,
    duration_s: f64,
) -> QuadRun {
    let dt = 0.002;
    let mut x = [0.0, p.height_m, 0.0, 0.0, 0.0, 0.0];
    let hip = |x: &[f64; 6], i: usize| {
        [
            x[0] + hip_sign(i) * p.hip_offset_m * x[2].cos(),
            x[1] + hip_sign(i) * p.hip_offset_m * x[2].sin(),
        ]
    };
    let mut feet: [[f64; 2]; 4] = std::array::from_fn(|i| [hip(&x, i)[0], 0.0]);
    let mut liftoff = feet;
    let mut targets = feet;
    let mut prev_st = stance(&p, 0.0);
    let mut f_cmd = [[0.0; 2]; 4];
    let mut frames = Vec::new();
    let (mut ev, mut eh, mut eth) = (Vec::new(), Vec::new(), Vec::new());
    let mut fell = None;
    let steps = (duration_s / dt).round() as usize;
    for k in 0..steps {
        let t = k as f64 * dt;
        let v_des = speed_command(t);
        let st = stance(&p, t);
        // swing planning (Raibert): target at touchdown
        let tl = phase_left(&p, t);
        for i in 0..4 {
            if st[i] && !prev_st[i] {
                feet[i] = [targets[i][0], 0.0];
            }
            if !st[i] && prev_st[i] {
                liftoff[i] = feet[i];
            }
            if !st[i] {
                let h = hip(&x, i);
                let tx = h[0] + x[3] * tl + x[3] * p.period_s / 4.0 + 0.08 * (x[3] - v_des);
                targets[i] = [tx, 0.0];
                let s = 1.0 - tl / (p.period_s / 2.0);
                let u = 0.5 - 0.5 * (std::f64::consts::PI * s).cos();
                feet[i] = [
                    liftoff[i][0] + (tx - liftoff[i][0]) * u,
                    p.swing_height_m * (std::f64::consts::PI * s).sin(),
                ];
            }
        }
        prev_st = st;
        if fell.is_none() && k % ctrl_every == 0 {
            f_cmd = ctrl.forces(t, &x, &feet, st, v_des, &targets);
        }
        // contact model: unilateral, friction cone, swing feet carry no force
        let mut f = [[0.0; 2]; 4];
        for i in 0..4 {
            if st[i] {
                let fz = f_cmd[i][1].clamp(0.0, p.fz_max_n);
                f[i] = [f_cmd[i][0].clamp(-p.mu * fz, p.mu * fz), fz];
            }
        }
        let push = push_force(t);
        if fell.is_none() {
            let (mut fx, mut fz, mut tau) = (push, -p.mass_kg * G, 0.0);
            for i in 0..4 {
                let r = lever(&x, feet[i]);
                fx += f[i][0];
                fz += f[i][1];
                tau += r[0] * f[i][1] - r[1] * f[i][0];
            }
            x[3] += fx / p.mass_kg * dt;
            x[4] += fz / p.mass_kg * dt;
            x[5] += tau / p.inertia_kgm2 * dt;
            x[0] += x[3] * dt;
            x[1] += x[4] * dt;
            x[2] += x[5] * dt;
            // leg length limit = fall
            let reach = (0..4)
                .filter(|&i| st[i])
                .map(|i| {
                    let h = hip(&x, i);
                    (h[0] - feet[i][0]).hypot(h[1] - feet[i][1])
                })
                .fold(0.0, f64::max);
            if x[1] < 0.12 || x[2].abs() > 0.8 || reach > 2.0 * p.link_m {
                fell = Some(t);
            }
        }
        if t >= 1.0 {
            ev.push(x[3] - v_des);
            eh.push(x[1] - p.height_m);
            eth.push(x[2]);
        }
        if k % 5 == 0 {
            frames.push(QuadFrame {
                t,
                x,
                feet,
                forces: f,
                stance: st,
                v_des,
                push_n: push,
            });
        }
    }
    let run = Run {
        name: String::new(),
        log: Default::default(),
        metrics: vec![
            ("rms_speed_err_mps".into(), rms(&ev)),
            ("rms_height_err_m".into(), rms(&eh)),
            ("rms_pitch_rad".into(), rms(&eth)),
            ("fell".into(), if fell.is_some() { 1.0 } else { 0.0 }),
        ],
    };
    QuadRun {
        run,
        frames,
        fell_at_s: fell,
    }
}

/// Runs both controllers.
pub fn trot() -> Vec<QuadRun> {
    let p = QuadrupedParams::default();
    let mut out = Vec::new();
    let mut a = PdForceQp {
        p,
        kv: 3.0,
        kz: (150.0, 20.0),
        kth: (150.0, 20.0),
    };
    let mut r = simulate_quadruped(p, &mut a, 1, 7.0);
    r.run.name = "PD + force QP".into();
    out.push(r);
    let mut b = ConvexMpc::new(p);
    let mut r = simulate_quadruped(p, &mut b, 5, 7.0);
    r.run.name = "convex MPC".into();
    out.push(r);
    out
}
