//! Scalar performance metrics.

/// Root mean square.
pub fn rms(xs: &[f64]) -> f64 {
    if xs.is_empty() {
        return 0.0;
    }
    (xs.iter().map(|x| x * x).sum::<f64>() / xs.len() as f64).sqrt()
}

/// Maximum absolute value.
pub fn max_abs(xs: &[f64]) -> f64 {
    xs.iter().fold(0.0, |m, x| m.max(x.abs()))
}

/// Integral of the squared input norm (control effort) with step `dt_s`.
pub fn effort(us: &[nalgebra::DVector<f64>], dt_s: f64) -> f64 {
    us.iter().map(|u| u.norm_squared()).sum::<f64>() * dt_s
}

/// Distance from a point to a polyline (lateral / cross-track error).
pub fn distance_to_polyline(p: robo_planning::Vec2, line: &[robo_planning::Vec2]) -> f64 {
    line.windows(2)
        .map(|w| robo_planning::geometry::point_segment_distance(p, w[0], w[1]))
        .fold(f64::INFINITY, f64::min)
}
