//! Experiment History: every run writes a self-contained result directory; this page lists them,
//! shows their summary, and can re-open or re-run them.

use std::path::{Path, PathBuf};

use eframe::egui::{self, RichText};
use serde_json::Value;

use crate::app::{widgets, RoboApp};
use crate::model;
use crate::settings::Settings;

#[derive(Clone)]
pub struct Entry {
    pub dir: PathBuf,
    pub name: String,
    pub benchmark: bool,
}

pub struct HistoryState {
    pub entries: Vec<Entry>,
    pub selected: Option<usize>,
    pub loaded: Option<Value>,
    pub filter: String,
    pub log_y: bool,
}

impl HistoryState {
    pub fn new() -> Self {
        HistoryState { entries: Vec::new(), selected: None, loaded: None, filter: String::new(), log_y: true }
    }

    pub fn rescan(&mut self, settings: &Settings) {
        let mut entries = Vec::new();
        for (dir, benchmark) in [(settings.results_dir(), false), (settings.benchmarks_dir(), true)] {
            if let Ok(read) = std::fs::read_dir(&dir) {
                for e in read.flatten() {
                    let path = e.path();
                    let file = if benchmark { "benchmark.json" } else { "result.json" };
                    if path.join(file).exists() {
                        entries.push(Entry {
                            name: path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default(),
                            dir: path,
                            benchmark,
                        });
                    }
                }
            }
        }
        entries.sort_by(|a, b| b.name.cmp(&a.name));
        self.entries = entries;
        self.selected = None;
    }
}

fn load(entry: &Entry) -> Option<Value> {
    let file = if entry.benchmark { "benchmark.json" } else { "result.json" };
    std::fs::read_to_string(entry.dir.join(file)).ok().and_then(|t| serde_json::from_str(&t).ok())
}

impl RoboApp {
    pub fn history_page(&mut self, ui: &mut egui::Ui) {
        egui::Panel::left("history-list").default_size(360.0).resizable(true).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.heading("Results");
                if ui.button("Rescan").clicked() {
                    self.history.rescan(&self.settings);
                }
            });
            ui.horizontal(|ui| {
                ui.label("filter");
                ui.text_edit_singleline(&mut self.history.filter);
            });
            ui.label(RichText::new(format!("{}", self.settings.experiments_dir)).small().weak());
            let mut clicked = None;
            egui::ScrollArea::vertical().show(ui, |ui| {
                for (i, e) in self.history.entries.iter().enumerate() {
                    if !self.history.filter.is_empty() && !e.name.contains(&self.history.filter) {
                        continue;
                    }
                    let label = format!("{} {}", if e.benchmark { "[bench]" } else { "[run]" }, e.name);
                    if ui.selectable_label(self.history.selected == Some(i), RichText::new(label).monospace().small()).clicked() {
                        clicked = Some(i);
                    }
                }
                if self.history.entries.is_empty() {
                    ui.label("No results yet. Runs started from the UI or with 'robosbse-engine run' appear here.");
                }
            });
            if let Some(i) = clicked {
                self.history.selected = Some(i);
                self.history.loaded = load(&self.history.entries[i]);
            }
        });
        egui::CentralPanel::default().show(ui, |ui| {
            let Some(i) = self.history.selected else {
                ui.label("Select a result on the left.");
                return;
            };
            let entry = self.history.entries[i].clone();
            let Some(result) = self.history.loaded.clone() else {
                ui.label("Could not read this result.");
                return;
            };
            ui.horizontal_wrapped(|ui| {
                if ui.button("Open folder").clicked() {
                    widgets::open_in_file_manager(&entry.dir);
                }
                if !entry.benchmark {
                    if ui.button("Open configuration in its page").clicked() {
                        let cfg = result["experiment"].clone();
                        self.apply_experiment_config(&cfg, &entry.name);
                    }
                    if ui.button("Re-run (same seed)").clicked() {
                        let cfg = result["experiment"].clone();
                        self.apply_experiment_config(&cfg, &entry.name);
                        if cfg["problem"]["type"] == "controller-testing" {
                            self.start_testing_run();
                        } else {
                            self.start_lab_run();
                        }
                    }
                    if ui.button("Replay in Laboratory").clicked() && result["experiment"]["problem"]["type"] == "navigation" {
                        let cfg = result["experiment"].clone();
                        self.apply_experiment_config(&cfg, &entry.name);
                        self.lab.on_finished(&result);
                    }
                }
            });
            ui.separator();
            let mut log_y = self.history.log_y;
            egui::ScrollArea::vertical().show(ui, |ui| {
                if entry.benchmark {
                    crate::pages::benchmark::benchmark_report(ui, &result, &mut log_y);
                } else {
                    result_summary(ui, &result, &entry.dir);
                }
            });
            self.history.log_y = log_y;
        });
    }
}

fn result_summary(ui: &mut egui::Ui, result: &Value, dir: &Path) {
    let exp = &result["experiment"];
    ui.heading(exp["name"].as_str().unwrap_or("experiment"));
    egui::Grid::new("hist-summary").num_columns(2).striped(true).show(ui, |ui| {
        let rows = [
            ("problem", exp["problem"]["type"].to_string()),
            ("map", exp["problem"]["map"]["name"].to_string()),
            ("algorithm", exp["algorithm"]["type"].to_string()),
            ("seed", exp["seed"].to_string()),
            ("budget", exp["budget"]["maxEvaluations"].to_string()),
            ("stop reason", result["stopReason"].to_string()),
            ("evaluations", result["evaluations"].to_string()),
            ("elapsed ms", result["elapsedMs"].to_string()),
            ("best fitness", widgets::fmt(model::num(&result["best"]["fitness"]))),
            ("best found at", result["best"]["evaluation"].to_string()),
            ("success", result["success"].to_string()),
            ("targets (first)", format!("{} ({})", result["targets"]["count"], result["targets"]["firstEvaluation"])),
            ("engine", result["engineVersion"].to_string()),
            ("started", result["startedAt"].to_string()),
            ("directory", dir.display().to_string()),
        ];
        for (k, v) in rows {
            ui.label(k);
            ui.monospace(v.trim_matches('"'));
            ui.end_row();
        }
    });
    ui.add_space(6.0);
    ui.label(RichText::new("Algorithm parameters").strong());
    ui.monospace(serde_json::to_string(&result["algorithmParams"]).unwrap_or_default());
    ui.add_space(6.0);
    let mut fake = crate::runs::RunState::new("h".into(), crate::runs::RunKind::Lab, Value::Null, 0);
    fake.apply_finished(result.clone());
    ui.columns(2, |cols| {
        widgets::convergence_plot(&mut cols[0], "hist-conv", &fake, 240.0, false);
        // The selected series is remembered in egui's temporary memory (free function, no page state).
        let key = egui::Id::new("hist-series-choice");
        let mut choice = cols[1].data(|d| d.get_temp::<String>(key)).unwrap_or_default();
        widgets::series_plot(&mut cols[1], "hist-series", &result["series"], 240.0, &mut choice);
        cols[1].data_mut(|d| d.insert_temp(key, choice));
    });
    ui.label(RichText::new("Best metrics").strong());
    widgets::metrics_grid(ui, "hist-metrics", &[("best", &result["best"]["metrics"])]);
    if result["pareto"].is_object() {
        ui.add_space(6.0);
        ui.label(RichText::new(format!(
            "Pareto front: {} non-dominated feasible solutions of {}, normalised hypervolume {} (reference {})",
            result["pareto"]["points"].as_array().map(|a| a.len()).unwrap_or(0),
            result["pareto"]["objectives"],
            widgets::fmt(model::num(&result["pareto"]["hypervolume"])),
            result["pareto"]["referencePoint"]
        )).strong());
    }
    if let Some(failures) = result["failures"].as_array() {
        ui.add_space(6.0);
        ui.label(RichText::new(format!("{} failure record(s) in failures/", failures.len())).strong());
        for f in failures {
            ui.monospace(format!("{} -> {} (outcome {})", f["controller"], f["violated"], f["outcome"]));
        }
    }
}
