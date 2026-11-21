//! Humanoid walking on the 3D linear inverted pendulum (LIPM).
//!
//! Plant: `c_ddot = w^2 (c - p)` with the ZMP `p` confined to the stance foot
//! (a 0.22 m x 0.12 m rectangle) and instantaneous support switches every
//! `step_s`. Two controllers walk the same gait and take the same push:
//!
//! * ZMP preview control (Kajita 2003) on fixed, pre-planned footsteps;
//! * DCM (capture point) control that re-plans each foothold online from the
//!   predicted DCM at touchdown (step adjustment).

use super::Run;
use crate::lqr::dlqr;
use nalgebra::{DMatrix, DVector};

/// Gait and robot parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LipmParams {
    /// CoM height [m]
    pub com_height_m: f64,
    /// single-support duration [s]
    pub step_s: f64,
    /// nominal step length [m]
    pub step_length_m: f64,
    /// lateral distance between feet [m]
    pub step_width_m: f64,
    /// foot length / width [m]
    pub foot_size_m: (f64, f64),
    /// maximum step reach from the stance foot [m]
    pub max_reach_m: f64,
}

impl Default for LipmParams {
    fn default() -> Self {
        Self {
            com_height_m: 0.8,
            step_s: 0.5,
            step_length_m: 0.25,
            step_width_m: 0.2,
            foot_size_m: (0.22, 0.12),
            max_reach_m: 0.6,
        }
    }
}

impl LipmParams {
    /// Natural frequency `sqrt(g / z_c)`.
    pub fn omega(&self) -> f64 {
        (9.81 / self.com_height_m).sqrt()
    }
}

/// Frame for drawing.
#[derive(Debug, Clone)]
pub struct HumanoidFrame {
    /// time [s]
    pub t: f64,
    /// CoM position (x, y)
    pub com: [f64; 2],
    /// ZMP (x, y)
    pub zmp: [f64; 2],
    /// DCM (x, y)
    pub dcm: [f64; 2],
    /// stance foot centre
    pub stance: [f64; 2],
    /// swing foot: lift-off point, target, progress in [0, 1]
    pub swing: ([f64; 2], [f64; 2], f64),
    /// footsteps placed so far
    pub steps: Vec<[f64; 2]>,
    /// fallen flag
    pub fallen: bool,
}

/// Result for one controller.
#[derive(Debug, Clone)]
pub struct HumanoidRun {
    /// summary
    pub run: Run,
    /// frames every 20 ms
    pub frames: Vec<HumanoidFrame>,
    /// fall time
    pub fell_at_s: Option<f64>,
}

/// Push: velocity change [m/s] applied over 0.1 s at `t = 3.1 s`.
pub const PUSH: (f64, [f64; 2]) = (3.1, [0.35, 0.25]);

fn nominal_steps(p: &LipmParams, n: usize) -> Vec<[f64; 2]> {
    // step 0 is the right foot at x = 0; feet alternate right/left
    (0..n)
        .map(|k| {
            [
                k as f64 * p.step_length_m,
                if k % 2 == 0 {
                    -p.step_width_m / 2.0
                } else {
                    p.step_width_m / 2.0
                },
            ]
        })
        .collect()
}

/// ZMP preview controller for one axis (cart-table model with jerk input).
#[derive(Debug, Clone)]
struct Preview {
    a: DMatrix<f64>,
    b: DVector<f64>,
    c: DVector<f64>,
    ki: f64,
    kx: DVector<f64>,
    gp: Vec<f64>,
}

impl Preview {
    fn new(dt: f64, w2: f64, n_preview: usize) -> Self {
        let a = DMatrix::from_row_slice(3, 3, &[1.0, dt, dt * dt / 2.0, 0.0, 1.0, dt, 0.0, 0.0, 1.0]);
        let b = DVector::from_vec(vec![dt.powi(3) / 6.0, dt * dt / 2.0, dt]);
        let c = DVector::from_vec(vec![1.0, 0.0, -1.0 / w2]);
        // augmented system [e_sum; x]
        let mut aa = DMatrix::zeros(4, 4);
        aa[(0, 0)] = 1.0;
        let ca = c.transpose() * &a;
        for j in 0..3 {
            aa[(0, j + 1)] = ca[j];
        }
        aa.view_mut((1, 1), (3, 3)).copy_from(&a);
        let mut bb = DMatrix::zeros(4, 1);
        bb[(0, 0)] = c.dot(&b);
        bb.view_mut((1, 0), (3, 1)).copy_from(&b);
        let mut q = DMatrix::zeros(4, 4);
        q[(0, 0)] = 1.0;
        let r = DMatrix::from_element(1, 1, 1e-6);
        let k = dlqr(&aa, &bb, &q, &r);
        // Riccati solution for preview gains
        let mut p = q.clone();
        for _ in 0..5000 {
            let bt_p = bb.transpose() * &p;
            let s = &r + &bt_p * &bb;
            let kk = s.try_inverse().unwrap() * &bt_p * &aa;
            let pn = &q + aa.transpose() * &p * &aa - aa.transpose() * &p * &bb * kk;
            if (&pn - &p).abs().max() < 1e-12 {
                p = pn;
                break;
            }
            p = pn;
        }
        let s_inv = 1.0 / (r[(0, 0)] + (bb.transpose() * &p * &bb)[(0, 0)]);
        let ac = &aa - &bb * &k;
        let mut i_vec = DVector::zeros(4);
        i_vec[0] = 1.0;
        // Gp(1) = -Ki, Gp(j) = (R + B'PB)^-1 B' X(j-1), X(1) = -Ac' P I, X(j) = Ac' X(j-1)
        let mut xv = -ac.transpose() * &p * &i_vec;
        let mut gp = vec![0.0; n_preview + 1];
        gp[1] = -k[(0, 0)];
        for g in gp.iter_mut().skip(2) {
            *g = s_inv * (bb.transpose() * &xv)[(0, 0)];
            xv = ac.transpose() * xv;
        }
        Self {
            a,
            b,
            c,
            ki: k[(0, 0)],
            kx: DVector::from_vec(vec![k[(0, 1)], k[(0, 2)], k[(0, 3)]]),
            gp,
        }
    }
}

/// Double-support time before the first step [s].
pub const START_S: f64 = 1.0;

/// Walking strategy of a [`LipmWalker`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Strategy {
    /// ZMP preview control (Kajita 2003) on fixed, pre-planned footsteps.
    PreviewFixedSteps,
    /// DCM control that re-plans each foothold from the predicted DCM at touchdown.
    DcmStepAdjustment,
}

impl Strategy {
    /// Display name.
    pub fn name(self) -> &'static str {
        match self {
            Strategy::PreviewFixedSteps => "ZMP preview (fixed steps)",
            Strategy::DcmStepAdjustment => "DCM + step adjustment",
        }
    }
}

/// One walker update.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WalkerOutput {
    /// desired ZMP before clamping to the support region
    pub zmp_des: [f64; 2],
    /// support region (axis-aligned) lower and upper corners
    pub support: ([f64; 2], [f64; 2]),
    /// whether both feet are on the ground (initial double support)
    pub double_support: bool,
    /// stance foot centre (single support)
    pub stance: [f64; 2],
    /// stance foot is the left foot
    pub stance_left: bool,
    /// swing foot lift-off point, current target and progress in `[0, 1]`
    pub swing: ([f64; 2], [f64; 2], f64),
    /// index of the current step (0 before walking)
    pub step_index: usize,
}

/// Online LIPM walking controller: feed it the measured CoM and CoM velocity
/// every `dt_s`, get the desired ZMP and the footholds. It drives both the
/// point-mass simulation of [`walk`] and full robots (e.g. the Unitree G1 on RNE).
#[derive(Debug, Clone)]
pub struct LipmWalker {
    /// gait parameters
    pub p: LipmParams,
    /// strategy
    pub strategy: Strategy,
    dt: f64,
    plan: Vec<[f64; 2]>,
    left0: [f64; 2],
    /// footsteps placed so far (centres)
    pub steps: Vec<[f64; 2]>,
    stance: [f64; 2],
    next: [f64; 2],
    liftoff: [f64; 2],
    step_idx: usize,
    preview: Preview,
    /// preview reference state per axis: CoM, CoM velocity, CoM acceleration
    reference: Option<[[f64; 3]; 2]>,
    esum: [f64; 2],
}

impl LipmWalker {
    /// Walker standing with its right foot at `right0` (first stance foot) and
    /// its left foot `step_width_m` to the left, CoM between them.
    pub fn new(p: LipmParams, strategy: Strategy, dt_s: f64, right0: [f64; 2], n_steps: usize) -> Self {
        let plan: Vec<[f64; 2]> = nominal_steps(&p, n_steps)
            .into_iter()
            .map(|s| [s[0] + right0[0], s[1] + right0[1] + p.step_width_m / 2.0])
            .collect();
        let left0 = [right0[0], right0[1] + p.step_width_m];
        Self {
            p,
            strategy,
            dt: dt_s,
            steps: vec![plan[0], left0],
            stance: plan[0],
            next: plan[1],
            liftoff: left0,
            left0,
            step_idx: 0,
            preview: Preview::new(dt_s, p.omega() * p.omega(), (1.6 / dt_s) as usize),
            reference: None,
            esum: [0.0, 0.0],
            plan,
        }
    }

    fn zref(&self, t: f64) -> [f64; 2] {
        let mid = [
            (self.plan[0][0] + self.left0[0]) / 2.0,
            (self.plan[0][1] + self.left0[1]) / 2.0,
        ];
        if t < START_S {
            let u = ((t - (START_S - 0.4)) / 0.4).clamp(0.0, 1.0);
            [
                mid[0] + (self.plan[0][0] - mid[0]) * u,
                mid[1] + (self.plan[0][1] - mid[1]) * u,
            ]
        } else {
            self.plan[(((t - START_S) / self.p.step_s) as usize).min(self.plan.len() - 1)]
        }
    }

    /// Replaces the current stance foot with where it actually landed (a robot's
    /// swing foot rarely lands exactly on target). Footholds planned afterwards
    /// start from it, and the support region follows it.
    pub fn set_stance_position(&mut self, centre: [f64; 2]) {
        self.stance = centre;
        if let Some(last) = self.steps.last_mut() {
            *last = centre;
        }
    }

    /// Advances to time `t` with the measured CoM position and velocity.
    pub fn update(&mut self, t: f64, c: [f64; 2], cd: [f64; 2]) -> WalkerOutput {
        let p = self.p;
        let w = p.omega();
        let w2 = w * w;
        let ewt = (w * p.step_s).exp();
        let walking = t >= START_S;
        let s_idx = if walking {
            ((t - START_S) / p.step_s) as usize
        } else {
            0
        };
        let dcm_stepping = self.strategy == Strategy::DcmStepAdjustment;
        // step index 0 stands on the right foot; even indices are right
        let right_y_sign = |idx: usize| if idx % 2 == 0 { -1.0 } else { 1.0 };
        if s_idx != self.step_idx {
            self.step_idx = s_idx;
            self.liftoff = self.stance;
            self.stance = self.next;
            self.steps.push(self.stance);
            self.next = if dcm_stepping {
                let side = right_y_sign(s_idx + 1);
                [self.stance[0] + p.step_length_m, self.stance[1] + side * p.step_width_m]
            } else {
                self.plan[(s_idx + 1).min(self.plan.len() - 1)]
            };
        }
        let tau = if walking {
            t - START_S - s_idx as f64 * p.step_s
        } else {
            0.0
        };
        let dcm = [c[0] + cd[0] / w, c[1] + cd[1] / w];
        let stance = self.stance;
        let support = if walking {
            (
                [stance[0] - p.foot_size_m.0 / 2.0, stance[1] - p.foot_size_m.1 / 2.0],
                [stance[0] + p.foot_size_m.0 / 2.0, stance[1] + p.foot_size_m.1 / 2.0],
            )
        } else {
            (
                [
                    self.plan[0][0].min(self.left0[0]) - p.foot_size_m.0 / 2.0,
                    self.plan[0][1].min(self.left0[1]) - p.foot_size_m.1 / 2.0,
                ],
                [
                    self.plan[0][0].max(self.left0[0]) + p.foot_size_m.0 / 2.0,
                    self.plan[0][1].max(self.left0[1]) + p.foot_size_m.1 / 2.0,
                ],
            )
        };
        let zmp_des = if dcm_stepping {
            if walking {
                let side = right_y_sign(s_idx + 1);
                let b = [p.step_length_m / (ewt - 1.0), -side * p.step_width_m / (ewt + 1.0)];
                let er = (w * (p.step_s - tau)).exp();
                let xi_td = [
                    stance[0] + (dcm[0] - stance[0]) * er,
                    stance[1] + (dcm[1] - stance[1]) * er,
                ];
                let dx = (xi_td[0] - b[0] - stance[0]).clamp(-p.max_reach_m, p.max_reach_m);
                let dy = xi_td[1] - b[1] - stance[1];
                let dy = if side > 0.0 {
                    dy.clamp(0.12, p.max_reach_m)
                } else {
                    dy.clamp(-p.max_reach_m, -0.12)
                };
                if tau < p.step_s - 0.05 {
                    self.next = [stance[0] + dx, stance[1] + dy];
                }
                // DCM error feedback inside the foot: p = f + (1 + k)(xi - xi_nominal)
                let b_cur = [p.step_length_m / (ewt - 1.0), side * p.step_width_m / (ewt + 1.0)];
                let grow = (w * tau).exp();
                let xi_nom = [stance[0] + b_cur[0] * grow, stance[1] + b_cur[1] * grow];
                let k_fb = 1.0;
                [
                    stance[0] + (1.0 + k_fb) * (dcm[0] - xi_nom[0]),
                    stance[1] + (1.0 + k_fb) * (dcm[1] - xi_nom[1]),
                ]
            } else {
                // drive the DCM to its steady-walking value at the first step
                let b0 = [p.step_length_m / (ewt - 1.0), p.step_width_m / (ewt + 1.0)];
                let u = (t / (0.7 * START_S)).clamp(0.0, 1.0);
                let u = u * u * (3.0 - 2.0 * u);
                let mid = [
                    (self.plan[0][0] + self.left0[0]) / 2.0,
                    (self.plan[0][1] + self.left0[1]) / 2.0,
                ];
                let tgt0 = [self.plan[0][0] + b0[0], self.plan[0][1] + b0[1]];
                let tgt = [mid[0] + (tgt0[0] - mid[0]) * u, mid[1] + (tgt0[1] - mid[1]) * u];
                [dcm[0] + 1.5 * (dcm[0] - tgt[0]), dcm[1] + 1.5 * (dcm[1] - tgt[1])]
            }
        } else {
            // Preview control generates the CoM reference on its own cart-table
            // model (Kajita 2003); a DCM stabilizer tracks it with the ZMP.
            let mut r = self.reference.unwrap_or([[c[0], 0.0, 0.0], [c[1], 0.0, 0.0]]);
            let mut p_ref = [0.0, 0.0];
            for ax in 0..2 {
                let x = DVector::from_vec(r[ax].to_vec());
                self.esum[ax] += self.preview.c.dot(&x) - self.zref(t)[ax];
                let mut u = -self.preview.ki * self.esum[ax] - self.preview.kx.dot(&x);
                for (j, g) in self.preview.gp.iter().enumerate().skip(1) {
                    u -= g * self.zref(t + j as f64 * self.dt)[ax];
                }
                let xn = &self.preview.a * &x + &self.preview.b * u;
                r[ax] = [xn[0], xn[1], xn[2]];
                p_ref[ax] = self.preview.c.dot(&xn);
            }
            self.reference = Some(r);
            let k_fb = 1.0;
            let mut z = [0.0, 0.0];
            for ax in 0..2 {
                let xi_ref = r[ax][0] + r[ax][1] / w;
                z[ax] = p_ref[ax] + (1.0 + k_fb) * (dcm[ax] - xi_ref);
            }
            let _ = w2;
            z
        };
        WalkerOutput {
            zmp_des,
            support,
            double_support: !walking,
            stance,
            stance_left: s_idx % 2 == 1,
            swing: (
                if walking { self.liftoff } else { self.left0 },
                self.next,
                if walking { tau / p.step_s } else { 0.0 },
            ),
            step_index: s_idx,
        }
    }
}

/// Simulates point-mass LIPM walking with a [`LipmWalker`]; `push` enables [`PUSH`].
pub fn walk(p: LipmParams, dcm_stepping: bool, duration_s: f64, push: bool) -> HumanoidRun {
    let dt = 0.005;
    let w = p.omega();
    let w2 = w * w;
    let strategy = if dcm_stepping {
        Strategy::DcmStepAdjustment
    } else {
        Strategy::PreviewFixedSteps
    };
    let mut walker = LipmWalker::new(
        p,
        strategy,
        dt,
        [0.0, -p.step_width_m / 2.0],
        (duration_s / p.step_s) as usize + 8,
    );
    let mut c = [0.0, 0.0];
    let mut cd = [0.0, 0.0];
    let mut frames = Vec::new();
    let mut fell = None;
    let (mut ev, mut zmp_sat) = (Vec::new(), 0usize);
    let total = (duration_s / dt) as usize;
    for k in 0..total {
        let t = k as f64 * dt;
        let out = walker.update(t, c, cd);
        let (lo, hi) = out.support;
        let zmp = [out.zmp_des[0].clamp(lo[0], hi[0]), out.zmp_des[1].clamp(lo[1], hi[1])];
        if !out.double_support && (zmp[0] - out.zmp_des[0]).abs() + (zmp[1] - out.zmp_des[1]).abs() > 1e-3 {
            zmp_sat += 1;
        }
        if fell.is_none() {
            let f = if push && (PUSH.0..PUSH.0 + 0.1).contains(&t) {
                [PUSH.1[0] / 0.1, PUSH.1[1] / 0.1]
            } else {
                [0.0, 0.0]
            };
            for ax in 0..2 {
                cd[ax] += (w2 * (c[ax] - zmp[ax]) + f[ax]) * dt;
                c[ax] += cd[ax] * dt;
            }
            let support = if out.double_support { [0.0, 0.0] } else { out.stance };
            if (c[0] - support[0]).hypot(c[1] - support[1]) > 0.55 {
                fell = Some(t);
            }
        }
        if t > START_S + 1.0 {
            ev.push(cd[0] - p.step_length_m / p.step_s);
        }
        if k % 4 == 0 {
            frames.push(HumanoidFrame {
                t,
                com: c,
                zmp,
                dcm: [c[0] + cd[0] / w, c[1] + cd[1] / w],
                stance: out.stance,
                swing: out.swing,
                steps: walker.steps.clone(),
                fallen: fell.is_some(),
            });
        }
    }
    let run = Run {
        name: strategy.name().into(),
        log: Default::default(),
        metrics: vec![
            ("fell".into(), if fell.is_some() { 1.0 } else { 0.0 }),
            ("fall_time_s".into(), fell.unwrap_or(f64::NAN)),
            ("rms_speed_err_mps".into(), crate::metrics::rms(&ev)),
            ("zmp_saturated_frac".into(), zmp_sat as f64 / total as f64),
        ],
    };
    HumanoidRun {
        run,
        frames,
        fell_at_s: fell,
    }
}

/// Both controllers on the same gait and push.
pub fn push_recovery() -> Vec<HumanoidRun> {
    let p = LipmParams::default();
    vec![walk(p, false, 7.0, true), walk(p, true, 7.0, true)]
}
