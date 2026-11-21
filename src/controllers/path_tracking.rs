//! Path and trajectory tracking for wheeled robots: pure pursuit, Stanley and
//! Kanayama. Generic LQR / MPC live in [`crate::lqr`] and [`crate::mpc`].

use crate::model::wrap;
use robo_planning::trajectory::{PathPoint, TrajPoint};

/// Reference path with a monotone nearest-point search.
#[derive(Debug, Clone)]
pub struct RefPath {
    /// time-parameterised samples
    pub traj: Vec<TrajPoint>,
    last: usize,
}

impl RefPath {
    /// Wraps a trajectory.
    pub fn new(traj: Vec<TrajPoint>) -> Self {
        Self { traj, last: 0 }
    }
    /// Nearest sample to `(x, y)` searched forward from the previous match.
    pub fn nearest(&mut self, x: f64, y: f64) -> usize {
        let end = (self.last + 200).min(self.traj.len());
        let mut best = self.last;
        let mut bd = f64::INFINITY;
        for i in self.last..end {
            let p = &self.traj[i].p;
            let d = (p.x - x).hypot(p.y - y);
            if d < bd {
                bd = d;
                best = i;
            }
        }
        self.last = best;
        best
    }
    /// Sample `ahead_m` of arc length after index `i`.
    pub fn ahead(&self, i: usize, ahead_m: f64) -> PathPoint {
        let s = self.traj[i].p.s_m + ahead_m;
        let j = self.traj.partition_point(|t| t.p.s_m < s).min(self.traj.len() - 1);
        self.traj[j].p
    }
    /// Signed lateral error (left positive) and heading error of pose relative to sample `i`.
    pub fn errors(&self, i: usize, x: f64, y: f64, yaw: f64) -> (f64, f64) {
        let p = &self.traj[i].p;
        let (dx, dy) = (x - p.x, y - p.y);
        (-p.yaw_rad.sin() * dx + p.yaw_rad.cos() * dy, wrap(yaw - p.yaw_rad))
    }
    /// Speed command along the path: profile speed, but at least `v_min` until the last `stop_m`.
    pub fn speed(&self, i: usize, v_min: f64, stop_m: f64) -> f64 {
        let remaining = self.traj.last().unwrap().p.s_m - self.traj[i].p.s_m;
        if remaining < stop_m {
            if remaining < 0.02 {
                0.0
            } else {
                v_min * (remaining / stop_m).sqrt()
            }
        } else {
            self.traj[i].v_mps.max(v_min)
        }
    }
}

/// Pure pursuit: steer along the arc through a look-ahead point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PurePursuit {
    /// minimum look-ahead [m]
    pub lookahead_min_m: f64,
    /// look-ahead growth with speed [s]
    pub lookahead_gain_s: f64,
}

impl PurePursuit {
    /// Path curvature command for the pose at speed `v`.
    pub fn curvature(&self, path: &RefPath, i: usize, x: f64, y: f64, yaw: f64, v: f64) -> f64 {
        let ld = self.lookahead_min_m + self.lookahead_gain_s * v.abs();
        let t = path.ahead(i, ld);
        let alpha = wrap((t.y - y).atan2(t.x - x) - yaw);
        let dist = (t.x - x).hypot(t.y - y).max(1e-3);
        2.0 * alpha.sin() / dist
    }
}

/// Stanley front-axle controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stanley {
    /// cross-track gain
    pub k: f64,
    /// low-speed softening [m/s]
    pub k_soft_mps: f64,
}

impl Stanley {
    /// Steering angle for a car whose front axle is at `(fx, fy)` with heading `yaw`.
    pub fn steer(&self, path: &mut RefPath, fx: f64, fy: f64, yaw: f64, v: f64) -> f64 {
        let i = path.nearest(fx, fy);
        let (e, he) = path.errors(i, fx, fy, yaw);
        -he - (self.k * e).atan2(self.k_soft_mps + v.abs())
    }
}

/// Kanayama's nonlinear trajectory-tracking law for unicycles (time-based reference).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Kanayama {
    /// along-track gain [1/s]
    pub kx: f64,
    /// cross-track gain [1/m^2]
    pub ky: f64,
    /// heading gain [1/m]
    pub kth: f64,
}

impl Kanayama {
    /// Returns `(v, w)` given the pose and the reference sample.
    pub fn control(&self, x: f64, y: f64, yaw: f64, r: &TrajPoint) -> (f64, f64) {
        let (dx, dy) = (r.p.x - x, r.p.y - y);
        let ex = yaw.cos() * dx + yaw.sin() * dy;
        let ey = -yaw.sin() * dx + yaw.cos() * dy;
        let eth = wrap(r.p.yaw_rad - yaw);
        let wr = r.v_mps * r.p.curvature;
        (
            r.v_mps * eth.cos() + self.kx * ex,
            wr + r.v_mps * (self.ky * ey + self.kth * eth.sin()),
        )
    }
}
