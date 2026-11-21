//! Closed-loop scenarios, one module per robot body. Each returns logs and
//! metrics; the `body_gallery` example renders them as GIFs.

pub mod arm;
pub mod cartpole;
pub mod drone;
pub mod humanoid;
pub mod parking;
pub mod quadruped;
pub mod wheeled;

/// One controller's result in a scenario.
#[derive(Debug, Clone)]
pub struct Run {
    /// controller name
    pub name: String,
    /// closed-loop log
    pub log: crate::sim::Log,
    /// named scalar metrics
    pub metrics: Vec<(String, f64)>,
}

impl Run {
    /// Metric by name.
    pub fn metric(&self, key: &str) -> f64 {
        self.metrics
            .iter()
            .find(|(k, _)| k == key)
            .map_or(f64::NAN, |(_, v)| *v)
    }
}

/// Markdown table of runs.
pub fn table(runs: &[Run]) -> String {
    let mut s = String::from("| controller |");
    for (k, _) in &runs[0].metrics {
        s += &format!(" {k} |");
    }
    s += "\n|---|";
    s += &"---|".repeat(runs[0].metrics.len());
    s += "\n";
    for r in runs {
        s += &format!("| {} |", r.name);
        for (_, v) in &r.metrics {
            s += &format!(" {v:.3} |");
        }
        s += "\n";
    }
    s
}
