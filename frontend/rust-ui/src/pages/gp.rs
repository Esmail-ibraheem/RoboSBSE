//! Genetic Programming: evolve reactive controller programs in the engine's typed robot language on
//! training maps, inspect the best program, and check how it generalises to unseen test maps against
//! the hand-written reactive controller.

use std::collections::HashMap;

use eframe::egui::{self, RichText};
use serde_json::{json, Value};

use crate::app::{widgets, RoboApp};
use crate::canvas::{self, Overlay, View};
use crate::model::{self, Capabilities, WorldMap};
use crate::params::{self, Values};
use crate::runs::{Replay, RunKind, RunStatus};

pub const PROBLEM: &str = "controller-synthesis";

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum GpTab {
    Progress,
    Program,
    Generalisation,
    Log,
}

pub struct GpState {
    pub problem: Values,
    pub algorithm: String,
    pub algo_values: HashMap<String, Values>,
    pub budget: u64,
    pub seed: u64,
    pub threads: u64,
    pub run_id: Option<String>,
    pub result: Option<Value>,
    pub episodes: Vec<Value>,
    pub episode: usize,
    pub map: Option<WorldMap>,
    pub view: View,
    pub replay: Replay,
    pub tab: GpTab,
    pub series_choice: String,
}

impl GpState {
    pub fn new() -> GpState {
        GpState {
            problem: Values::new(),
            algorithm: "genetic-algorithm".into(),
            algo_values: HashMap::new(),
            budget: 3000,
            seed: 1,
            threads: 4,
            run_id: None,
            result: None,
            episodes: Vec::new(),
            episode: 0,
            map: None,
            view: View::default(),
            replay: Replay { speed: 2.0, ..Replay::default() },
            tab: GpTab::Progress,
            series_choice: String::new(),
        }
    }

    pub fn on_capabilities(&mut self, caps: &Capabilities) {
        if let Some(p) = caps.problem(PROBLEM) {
            for (k, v) in params::defaults(&p.params) {
                self.problem.entry(k).or_insert(v);
            }
        }
        for a in &caps.algorithms {
            let values = self.algo_values.entry(a.id.clone()).or_default();
            for (k, v) in params::defaults(&a.params) {
                values.entry(k).or_insert(v);
            }
        }
        if let Some(v) = self.algo_values.get_mut("genetic-algorithm") {
            v.insert("populationSize".into(), json!(50));
        }
    }

    pub fn experiment(&self, caps: &Capabilities) -> Value {
        let specs = caps.problem(PROBLEM).map(|p| p.params.clone()).unwrap_or_default();
        let algo_specs = caps.algorithm(&self.algorithm).map(|a| a.params.clone()).unwrap_or_default();
        let operators = caps.problem(PROBLEM).map(|p| p.operators.clone());
        let empty = Values::new();
        json!({
            "schema": "robosbse.experiment/1",
            "name": "gp-controller",
            "seed": self.seed,
            "budget": {"maxEvaluations": self.budget},
            "problem": {"type": PROBLEM, "params": params::to_json(&specs, &self.problem)},
            "algorithm": {
                "type": self.algorithm,
                "params": params::to_json_for(&algo_specs, self.algo_values.get(&self.algorithm).unwrap_or(&empty), operators.as_ref()),
            },
            "options": {"archiveSize": 5, "threads": self.threads}
        })
    }

    pub fn on_finished(&mut self, result: &Value) {
        self.result = Some(result.clone());
        self.episodes = result["replay"]["episodes"].as_array().cloned().unwrap_or_default();
        self.select(0);
        self.tab = GpTab::Program;
    }

    pub fn select(&mut self, i: usize) {
        let Some(e) = self.episodes.get(i).cloned() else { return };
        self.episode = i;
        self.map = serde_json::from_value(e["map"].clone()).ok();
        self.view.reset();
        let title = format!("{} map: {}", e["split"].as_str().unwrap_or(""), e["map"]["name"].as_str().unwrap_or(""));
        self.replay.load(&e["trajectory"], title, e["outcome"].as_str().unwrap_or(""), None, e["metrics"].clone());
    }
}

impl RoboApp {
    pub fn start_gp_run(&mut self) {
        let Some(caps) = self.caps.clone() else { return };
        let experiment = self.gp.experiment(&caps);
        if let Some(id) = self.start_run(RunKind::Gp, experiment) {
            self.gp.run_id = Some(id);
            self.gp.result = None;
            self.gp.episodes.clear();
            self.gp.replay.clear();
            self.gp.tab = GpTab::Progress;
        }
    }

    fn gp_config(&mut self, ui: &mut egui::Ui) {
        let Some(caps) = self.caps.clone() else {
            ui.label("Waiting for the engine...");
            return;
        };
        let running = self.run(&self.gp.run_id).map(|r| r.status.active()).unwrap_or(false);
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("Genetic programming");
            ui.add(egui::Label::new(RichText::new(
                "Evolves a reactive controller program from typed primitives:\n\
                 actions  forward, turnLeft, turnRight, veerLeft, veerRight, stop, if(cond, a, b)\n\
                 conditions  obstacleAhead, goalLeft, goalAhead, <, >, and, or, not\n\
                 numbers  front, left, right (ranges, m), goalBearing, goalDistance, constants, +, -, *\n\
                 No loops; size, depth and per-tick evaluation steps are limited.").small().monospace()).wrap());
            ui.add_space(6.0);
            ui.label(RichText::new("Environments and limits").strong());
            if let Some(p) = caps.problem(PROBLEM) {
                params::form(ui, "gp-problem", &p.params, &mut self.gp.problem, &[]);
            }
            ui.label(RichText::new("Maps: builtin names, files, or generated:scatter:<seed>. Test maps are never seen by the search.")
                .small().weak());
            ui.add_space(6.0);
            ui.separator();
            ui.label(RichText::new("Search").strong());
            let current = caps.algorithm(&self.gp.algorithm).map(|a| a.name.clone()).unwrap_or_default();
            egui::ComboBox::from_id_salt("gp-alg").selected_text(current).width(220.0).show_ui(ui, |ui| {
                for a in caps.algorithms_for(PROBLEM) {
                    ui.selectable_value(&mut self.gp.algorithm, a.id.clone(), &a.name);
                }
            });
            if let Some(a) = caps.algorithm(&self.gp.algorithm) {
                let operators = caps.problem(PROBLEM).map(|p| p.operators.clone());
                let values = self.gp.algo_values.entry(a.id.clone()).or_default();
                params::form_for(ui, "gp-alg-params", &a.params, values, &[], operators.as_ref());
            }
            egui::Grid::new("gp-budget").num_columns(2).show(ui, |ui| {
                ui.label("Fitness evaluations");
                ui.add(egui::DragValue::new(&mut self.gp.budget).range(1..=10_000_000).speed(50));
                ui.end_row();
                ui.label("Random seed");
                ui.add(egui::DragValue::new(&mut self.gp.seed));
                ui.end_row();
                ui.label("Evaluation threads");
                ui.add(egui::DragValue::new(&mut self.gp.threads).range(1..=64));
                ui.end_row();
            });
            ui.add_space(8.0);
            ui.horizontal_wrapped(|ui| {
                if ui.add_enabled(!running, egui::Button::new(RichText::new("Evolve controller").strong())).clicked() {
                    self.start_gp_run();
                }
                let id = self.gp.run_id.clone();
                let status = self.run(&id).map(|r| r.status.clone());
                if let (Some(id), Some(status)) = (id, status) {
                    if status == RunStatus::Running && ui.button("Pause").clicked() {
                        self.control_run(&id, "pause");
                    }
                    if status == RunStatus::Paused && ui.button("Resume").clicked() {
                        self.control_run(&id, "resume");
                    }
                    if status.active() && ui.button("Cancel").clicked() {
                        self.control_run(&id, "cancel");
                    }
                }
            });
        });
    }

    fn gp_bottom(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.selectable_value(&mut self.gp.tab, GpTab::Progress, "Progress");
            ui.selectable_value(&mut self.gp.tab, GpTab::Program, "Best program");
            ui.selectable_value(&mut self.gp.tab, GpTab::Generalisation, "Generalisation");
            ui.selectable_value(&mut self.gp.tab, GpTab::Log, "Log");
        });
        ui.separator();
        match self.gp.tab {
            GpTab::Progress => match self.runs.get(self.gp.run_id.as_deref().unwrap_or("")) {
                Some(run) => {
                    widgets::run_status(ui, run, "programs solving every training map");
                    let h = ui.available_height().max(80.0);
                    let series = run.result.as_ref().map(|r| r["series"].clone()).unwrap_or(Value::Null);
                    let choice = &mut self.gp.series_choice;
                    ui.columns(2, |cols| {
                        widgets::convergence_plot(&mut cols[0], "gp-conv", run, h - 8.0, true);
                        widgets::series_plot(&mut cols[1], "gp-series", &series, h - 8.0, choice);
                    });
                }
                None => {
                    ui.label("Configure the environments on the right and press 'Evolve controller'.");
                }
            },
            GpTab::Program => {
                let live = self.run(&self.gp.run_id).map(|r| r.best_solution.clone()).unwrap_or(Value::Null);
                let best = self.gp.result.as_ref().map(|r| r["best"]["solution"].clone()).unwrap_or(live);
                if best["pretty"].is_string() {
                    ui.label(format!("size {} nodes, depth {}", best["size"], best["depth"]));
                    egui::ScrollArea::vertical().id_salt("gp-program").show(ui, |ui| {
                        ui.label(RichText::new(best["pretty"].as_str().unwrap_or("")).monospace());
                        ui.add_space(6.0);
                        ui.label(RichText::new(best["program"].as_str().unwrap_or("")).small().monospace().weak());
                    });
                } else {
                    ui.label("The best program appears here once a run has started.");
                }
            }
            GpTab::Generalisation => self.gp_generalisation(ui),
            GpTab::Log => crate::pages::lab::log_view(ui, &self.log),
        }
    }

    fn gp_generalisation(&mut self, ui: &mut egui::Ui) {
        let Some(result) = self.gp.result.clone() else {
            ui.label("Run a search to compare training and unseen test maps.");
            return;
        };
        let train = &result["replay"]["training"];
        let test = &result["replay"]["test"];
        let reference = &result["reference"]["reactive"];
        egui::Grid::new("gp-summary").striped(true).num_columns(5).show(ui, |ui| {
            for h in ["controller", "training success", "training cost", "test success", "test cost"] {
                ui.label(RichText::new(h).strong());
            }
            ui.end_row();
            ui.label("evolved program");
            ui.monospace(format!("{:.0}%", 100.0 * model::num(&train["successRate"])));
            ui.monospace(widgets::fmt(model::num(&train["meanCost"])));
            ui.monospace(format!("{:.0}%", 100.0 * model::num(&test["successRate"])));
            ui.monospace(widgets::fmt(model::num(&test["meanCost"])));
            ui.end_row();
            if reference.is_object() {
                ui.label("hand-written reactive controller");
                ui.monospace(format!("{:.0}%", 100.0 * model::num(&reference["training"]["successRate"])));
                ui.monospace(widgets::fmt(model::num(&reference["training"]["meanCost"])));
                ui.monospace(format!("{:.0}%", 100.0 * model::num(&reference["test"]["successRate"])));
                ui.monospace(widgets::fmt(model::num(&reference["test"]["meanCost"])));
                ui.end_row();
            }
        });
        ui.label(RichText::new("Cost per episode: travel time on success, else 1000 + 10 x remaining distance. \
            Click a row to replay that episode.").small().weak());
        let mut pick = None;
        egui::ScrollArea::vertical().id_salt("gp-episodes").show(ui, |ui| {
            egui::Grid::new("gp-episode-grid").striped(true).num_columns(5).show(ui, |ui| {
                for h in ["", "split", "map", "outcome", "cost"] {
                    ui.label(RichText::new(h).strong());
                }
                ui.end_row();
                for (i, e) in self.gp.episodes.iter().enumerate() {
                    if ui.selectable_label(self.gp.episode == i, "replay").clicked() {
                        pick = Some(i);
                    }
                    ui.label(e["split"].as_str().unwrap_or(""));
                    ui.monospace(e["map"]["name"].as_str().unwrap_or(""));
                    let outcome = e["outcome"].as_str().unwrap_or("");
                    ui.label(RichText::new(outcome).color(if outcome == "REACHED_GOAL" { canvas::START } else { canvas::DANGER }));
                    ui.monospace(widgets::fmt(model::num(&e["cost"])));
                    ui.end_row();
                }
            });
        });
        if let Some(i) = pick {
            self.gp.select(i);
        }
    }

    pub fn gp_page(&mut self, ui: &mut egui::Ui) {
        egui::Panel::right("gp-config").default_size(360.0).resizable(true).show(ui, |ui| self.gp_config(ui));
        egui::Panel::bottom("gp-bottom").default_size(280.0).resizable(true).show(ui, |ui| self.gp_bottom(ui));
        egui::CentralPanel::default().show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                self.gp.replay.controls(ui);
                if !self.gp.episodes.is_empty() {
                    ui.separator();
                    let current = self.gp.episodes.get(self.gp.episode)
                        .map(|e| format!("{}: {}", e["split"].as_str().unwrap_or(""), e["map"]["name"].as_str().unwrap_or("")))
                        .unwrap_or_default();
                    let mut pick = None;
                    egui::ComboBox::from_id_salt("gp-episode").selected_text(current).show_ui(ui, |ui| {
                        for (i, e) in self.gp.episodes.iter().enumerate() {
                            let label = format!("{}: {}", e["split"].as_str().unwrap_or(""), e["map"]["name"].as_str().unwrap_or(""));
                            if ui.selectable_label(i == self.gp.episode, label).clicked() {
                                pick = Some(i);
                            }
                        }
                    });
                    if let Some(i) = pick {
                        self.gp.select(i);
                    }
                }
            });
            let dark = self.dark();
            let g = &mut self.gp;
            match g.map.as_mut() {
                Some(map) => {
                    let caption = Some(format!("{}  |  evolved program", map.name));
                    let overlay = Overlay {
                        trajectory: g.replay.trajectory.as_ref(),
                        time: Some(g.replay.time),
                        show_rays: true,
                        robot_radius: 0.25,
                        sensor_range: 4.0,
                        caption,
                        ..Overlay::default()
                    };
                    canvas::show(ui, &mut g.view, map, &overlay, None, dark);
                }
                None => {
                    ui.add_space(20.0);
                    ui.label("Episode replays of the best program (training and test maps) appear here after a run.");
                }
            }
        });
    }
}
