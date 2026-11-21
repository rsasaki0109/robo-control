//! PID with integral clamping (anti-windup).

/// PID controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pid {
    /// proportional gain
    pub kp: f64,
    /// integral gain
    pub ki: f64,
    /// derivative gain
    pub kd: f64,
    /// integral state clamp
    pub i_limit: f64,
    integral: f64,
    prev_err: Option<f64>,
}

impl Pid {
    /// New controller with zero state.
    pub fn new(kp: f64, ki: f64, kd: f64, i_limit: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            i_limit,
            integral: 0.0,
            prev_err: None,
        }
    }
    /// Update with error `e`; derivative from finite differences.
    pub fn update(&mut self, e: f64, dt_s: f64) -> f64 {
        let d = self.prev_err.map_or(0.0, |p| (e - p) / dt_s);
        self.prev_err = Some(e);
        self.update_with_rate(e, d, dt_s)
    }
    /// Update with error `e` and a measured error rate `de` (no derivative kick).
    pub fn update_with_rate(&mut self, e: f64, de: f64, dt_s: f64) -> f64 {
        self.integral = (self.integral + e * dt_s).clamp(-self.i_limit, self.i_limit);
        self.kp * e + self.ki * self.integral + self.kd * de
    }
}
