//! Client-side state of engine runs and of trajectory replays. Everything shown here comes from
//! engine messages; the UI computes nothing about the search itself.

use eframe::egui;
use serde_json::Value;

use crate::model::{self, Trajectory};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RunKind {
    Lab,
    Testing,
    Benchmark,
    Gp,
    Repair,
    Arm,
}

#[derive(Clone, Debug, PartialEq)]
pub enum RunStatus {
    Starting,
    Running,
    Paused,
    Cancelling,
    Finished(String),
    Failed(String),
}

impl RunStatus {
    pub fn active(&self) -> bool {
        matches!(self, RunStatus::Starting | RunStatus::Running | RunStatus::Paused | RunStatus::Cancelling)
    }

    pub fn label(&self) -> String {
        match self {
            RunStatus::Starting => "starting".into(),
            RunStatus::Running => "running".into(),
            RunStatus::Paused => "paused".into(),
            RunStatus::Cancelling => "cancelling".into(),
            RunStatus::Finished(reason) => format!("finished ({})", reason.to_lowercase().replace('_', " ")),
            RunStatus::Failed(message) => format!("failed: {message}"),
        }
    }
}

pub struct RunState {
    pub id: String,
    pub kind: RunKind,
    pub status: RunStatus,
    pub evaluations: u64,
    pub max_evaluations: u64,
    pub best_fitness: f64,
    pub current_fitness: f64,
    pub elapsed_ms: u64,
    pub stats: Value,
    pub convergence: Vec<[f64; 2]>,
    pub best_solution: Value,
    pub best_metrics: Value,
    pub targets: u64,
    pub first_target: i64,
    pub result: Option<Value>,
    pub config: Value,
    pub bench_rows: Vec<Value>,
    pub bench_done: (u64, u64),
}

impl RunState {
    pub fn new(id: String, kind: RunKind, config: Value, max_evaluations: u64) -> RunState {
        RunState {
            id,
            kind,
            status: RunStatus::Starting,
            evaluations: 0,
            max_evaluations,
            best_fitness: f64::NAN,
            current_fitness: f64::NAN,
            elapsed_ms: 0,
            stats: Value::Null,
            convergence: Vec::new(),
            best_solution: Value::Null,
            best_metrics: Value::Null,
            targets: 0,
            first_target: -1,
            result: None,
            config,
            bench_rows: Vec::new(),
            bench_done: (0, 0),
        }
    }

    pub fn apply_progress(&mut self, m: &Value) {
        self.evaluations = m["evaluations"].as_u64().unwrap_or(self.evaluations);
        self.max_evaluations = m["maxEvaluations"].as_u64().unwrap_or(self.max_evaluations);
        self.best_fitness = model::num(&m["bestFitness"]);
        self.current_fitness = model::num(&m["currentFitness"]);
        self.elapsed_ms = m["elapsedMs"].as_u64().unwrap_or(self.elapsed_ms);
        self.stats = m["stats"].clone();
        self.targets = m["targets"].as_u64().unwrap_or(self.targets);
        self.first_target = m["firstTarget"].as_i64().unwrap_or(self.first_target);
        self.convergence.extend(model::points(&m["convergence"]));
        if self.status == RunStatus::Starting {
            self.status = RunStatus::Running;
        }
    }

    pub fn apply_best(&mut self, m: &Value) {
        self.best_fitness = model::num(&m["fitness"]);
        self.best_solution = m["solution"].clone();
        self.best_metrics = m["metrics"].clone();
    }

    /// Final result: the engine's convergence trace is authoritative.
    pub fn apply_finished(&mut self, result: Value) {
        self.status = RunStatus::Finished(result["stopReason"].as_str().unwrap_or("done").to_string());
        self.evaluations = result["evaluations"].as_u64().unwrap_or(self.evaluations);
        self.elapsed_ms = result["elapsedMs"].as_u64().unwrap_or(self.elapsed_ms);
        self.best_fitness = model::num(&result["best"]["fitness"]);
        self.best_solution = result["best"]["solution"].clone();
        self.best_metrics = result["best"]["metrics"].clone();
        self.targets = result["targets"]["count"].as_u64().unwrap_or(self.targets);
        self.first_target = result["targets"]["firstEvaluation"].as_i64().unwrap_or(self.first_target);
        let trace = model::points(&result["convergence"]);
        if !trace.is_empty() {
            self.convergence = trace;
        }
        if let Some(stats) = result.get("stats") {
            self.stats = stats.clone();
        }
        self.result = Some(result);
    }

    pub fn progress_fraction(&self) -> f32 {
        if self.max_evaluations == 0 {
            0.0
        } else {
            (self.evaluations as f64 / self.max_evaluations as f64).clamp(0.0, 1.0) as f32
        }
    }
}

/// Playback of a recorded trajectory.
#[derive(Default)]
pub struct Replay {
    pub trajectory: Option<Trajectory>,
    pub time: f64,
    pub playing: bool,
    pub speed: f32,
    pub title: String,
    pub outcome: String,
    pub route: Option<Vec<[f64; 2]>>,
    pub metrics: Value,
}

impl Replay {
    pub fn load(&mut self, trajectory: &Value, title: impl Into<String>, outcome: &str, route: Option<Vec<[f64; 2]>>,
                metrics: Value) -> bool {
        match serde_json::from_value::<Trajectory>(trajectory.clone()) {
            Ok(t) if t.len() > 0 => {
                self.trajectory = Some(t);
                self.time = 0.0;
                self.playing = true;
                if self.speed <= 0.0 {
                    self.speed = 2.0;
                }
                self.title = title.into();
                self.outcome = outcome.to_string();
                self.route = route;
                self.metrics = metrics;
                true
            }
            _ => false,
        }
    }

    pub fn clear(&mut self) {
        *self = Replay { speed: self.speed, ..Replay::default() };
    }

    pub fn duration(&self) -> f64 {
        self.trajectory.as_ref().map(|t| t.duration()).unwrap_or(0.0)
    }

    /// Advances playback; returns true while animating (the caller requests repaints).
    pub fn tick(&mut self, dt: f64) -> bool {
        if !self.playing || self.trajectory.is_none() {
            return false;
        }
        self.time += dt * self.speed as f64;
        if self.time >= self.duration() {
            self.time = self.duration();
            self.playing = false;
        }
        true
    }

    pub fn controls(&mut self, ui: &mut egui::Ui) {
        let duration = self.duration();
        let enabled = self.trajectory.is_some();
        ui.add_enabled_ui(enabled, |ui| {
            let label = if self.playing { "Pause" } else { "Play" };
            if ui.button(label).clicked() {
                if !self.playing && self.time >= duration {
                    self.time = 0.0;
                }
                self.playing = !self.playing;
            }
            if ui.button("Restart").clicked() {
                self.time = 0.0;
                self.playing = true;
            }
            ui.label("speed");
            egui::ComboBox::from_id_salt(("replay-speed", ui.id()))
                .selected_text(format!("{}x", self.speed))
                .width(56.0)
                .show_ui(ui, |ui| {
                    for s in [0.25f32, 0.5, 1.0, 2.0, 4.0, 8.0] {
                        ui.selectable_value(&mut self.speed, s, format!("{s}x"));
                    }
                });
            let mut t = self.time;
            let slider = ui.add(egui::Slider::new(&mut t, 0.0..=duration.max(0.01)).show_value(false));
            if slider.changed() {
                self.time = t;
                self.playing = false;
            }
            ui.monospace(format!("{:6.2} / {:.2} s", self.time, duration));
            if let Some(f) = self.trajectory.as_ref().and_then(|t| t.frame_at(self.time)) {
                ui.monospace(format!(
                    "v {:.2} m/s  battery {:.0}%  clearance {}",
                    f.v,
                    f.battery * 100.0,
                    if f.clearance.is_finite() { format!("{:.2} m", f.clearance) } else { "-".into() }
                ));
            }
        });
        if !self.title.is_empty() {
            ui.separator();
            ui.label(egui::RichText::new(&self.title).strong());
            if !self.outcome.is_empty() {
                let color = if self.outcome == "REACHED_GOAL" { crate::canvas::START } else { crate::canvas::DANGER };
                ui.label(egui::RichText::new(&self.outcome).color(color).monospace());
            }
        }
    }
}
