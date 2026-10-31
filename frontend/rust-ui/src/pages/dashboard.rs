//! Dashboard: engine status, quick starts, the implementation status of every module, and recent
//! results.

use eframe::egui::{self, RichText};
use serde_json::json;

use crate::app::{MapTarget, Page, RoboApp};
use crate::canvas;
use crate::engine::EngineStatus;

/// Implementation status per roadmap item — shown honestly in the UI.
const STATUS: &[(&str, &str, bool)] = &[
    ("Phase 1", "Deterministic 2D simulation, unicycle robot, range sensor, energy model", true),
    ("Phase 1", "Random search, random walk, hill climbing (3 variants + restarts), simulated annealing, GA", true),
    ("Phase 1", "Route optimisation with simulated fitness; A* and Dijkstra references", true),
    ("Phase 1", "Search-based failure discovery with 4 documented faulty controllers", true),
    ("Phase 1", "Failure records, reproduction, minimisation, regression tests", true),
    ("Phase 1", "Benchmarks: equal budgets, Mann-Whitney U, A12, CSV/JSON export", true),
    ("Phase 2", "Tabu search, PSO, evolution strategies, steady-state GA, exhaustive search", true),
    ("Phase 2", "Controller parameter tuning over several maps with held-out validation", true),
    ("Phase 2", "Warehouse scheduling: permutations, capacity, dependencies, deadlines, charging", true),
    ("Phase 2", "Multi-objective: Pareto archive, NSGA-II, exact hypervolume, Pareto view", true),
    ("Phase 3", "Memetic algorithm (HC / SA / route-smoothing local search, Lamarckian or Baldwinian)", true),
    ("Phase 3", "Novelty search with behaviour descriptors; behaviour coverage for every run", true),
    ("Phase 3", "Parallel fitness evaluation (identical results for any thread count), island-model GA", true),
    ("Phase 3", "Branch coverage of the reactive controllers: approach level, branch distance, campaigns", true),
    ("Phase 3", "Failure minimisation: ddmin over obstacles, numeric shrinking, rounding", true),
    ("Phase 4", "Genetic programming: typed robot language, size/step limits, parsimony, test-map generalisation", true),
    ("Phase 4", "Program repair: AST patches, isolated compilation and testing, minimisation, held-out validation", true),
    ("Phase 4", "Genetic improvement of a working controller (time and counted work), experiment suites", true),
    ("Phase 5", "Planar robotic arm: kinematics, joint limits, collisions; IK with analytical and numerical references", true),
    ("Phase 5", "Arm trajectory optimisation (configuration-space A* reference) and pick-and-place planning", true),
    ("Phase 5", "Multi-robot task allocation with simulated execution and robot-robot conflict detection", true),
    ("Phase 5", "Dynamic environments: moving and timed obstacles, restricted zones, rough terrain", true),
    ("Phase 5", "Multi-robot coordination (robots avoiding each other) and dynamic obstacles in the editor", false),
];

impl RoboApp {
    pub fn dashboard_page(&mut self, ui: &mut egui::Ui) {
        egui::CentralPanel::default().show(ui, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.heading("RoboSBSE — Search-Based Robotics and Software Engineering Laboratory");
                ui.label("Optimise robot routes with metaheuristics and use search to find failures in robot-controller software. \
                          Every number on screen comes from the Java engine's simulation; nothing is scripted.");
                ui.add_space(10.0);
                ui.horizontal(|ui| {
                    let (text, color) = match &self.engine.status {
                        EngineStatus::Ready => ("Engine ready".to_string(), canvas::START),
                        EngineStatus::Starting => ("Engine starting".to_string(), canvas::GOAL),
                        EngineStatus::Stopped => ("Engine stopped".to_string(), canvas::DANGER),
                        EngineStatus::Failed(e) => (format!("Engine problem: {e}"), canvas::DANGER),
                    };
                    ui.label(RichText::new(text).strong().color(color));
                    if let Some(caps) = &self.caps {
                        ui.label(format!(
                            "{} v{} | protocol {} | Java {} | {} algorithms | {} problem types | {} controllers ({} faulty) | {} maps",
                            caps.engine,
                            caps.version,
                            caps.protocol,
                            caps.java,
                            caps.algorithms.len(),
                            caps.problems.len(),
                            caps.controllers.len(),
                            caps.controllers.iter().filter(|c| c.faulty_of.is_some()).count(),
                            caps.maps.len()
                        ));
                    }
                    if !self.engine.is_ready() && ui.button("Settings").clicked() {
                        self.page = Page::Settings;
                    }
                });
                ui.add_space(12.0);
                ui.label(RichText::new("Quick start").strong());
                ui.horizontal_wrapped(|ui| {
                    let ready = self.engine.is_ready();
                    if ui.add_enabled(ready, egui::Button::new("Optimise a warehouse route (GA)")).clicked() {
                        self.request_map(json!("builtin:warehouse"), MapTarget::Lab);
                        self.lab.algorithm = "genetic-algorithm".into();
                        self.page = Page::Lab;
                    }
                    if ui.add_enabled(ready, egui::Button::new("Find BUG-1 in the reactive controller")).clicked() {
                        self.testing.problem.insert("controller".into(), json!("reactive-bug1-dropout"));
                        self.testing.problem.insert("fitness".into(), json!("collision"));
                        self.page = Page::Testing;
                    }
                    if ui.add_enabled(ready, egui::Button::new("Plan a pick-and-place job (arm)")).clicked() {
                        if let Some(caps) = self.caps.clone() {
                            self.arm.set_problem(&caps, crate::pages::arm::PICK_PLACE);
                        }
                        self.page = Page::Arm;
                    }
                    if ui.add_enabled(ready, egui::Button::new("Allocate jobs to three robots")).clicked() {
                        self.set_lab_problem(crate::pages::lab::MULTI);
                        self.page = Page::Lab;
                    }
                    if ui.add_enabled(ready, egui::Button::new("Compare algorithms")).clicked() {
                        self.page = Page::Benchmark;
                    }
                    if ui.add_enabled(ready, egui::Button::new("Open an example experiment...")).clicked() {
                        self.load_experiment_file();
                    }
                    if ui.button("Edit a map").clicked() {
                        self.page = Page::Editor;
                    }
                });
                ui.add_space(12.0);
                ui.label(RichText::new("Implementation status").strong());
                egui::Grid::new("status").striped(true).num_columns(3).show(ui, |ui| {
                    for (phase, item, done) in STATUS {
                        ui.label(*phase);
                        ui.label(*item);
                        ui.label(if *done {
                            RichText::new("implemented").color(canvas::START)
                        } else {
                            RichText::new("not implemented").weak()
                        });
                        ui.end_row();
                    }
                });
                ui.add_space(12.0);
                ui.label(RichText::new("Recent results").strong());
                let recent: Vec<_> = self.history.entries.iter().take(8).cloned().collect();
                if recent.is_empty() {
                    ui.label(RichText::new("none yet").weak());
                }
                for e in recent {
                    if ui.link(RichText::new(&e.name).monospace()).clicked() {
                        self.page = Page::History;
                        if let Some(i) = self.history.entries.iter().position(|x| x.dir == e.dir) {
                            self.history.selected = Some(i);
                            let file = if e.benchmark { "benchmark.json" } else { "result.json" };
                            self.history.loaded = std::fs::read_to_string(e.dir.join(file)).ok().and_then(|t| serde_json::from_str(&t).ok());
                        }
                    }
                }
            });
        });
    }
}
