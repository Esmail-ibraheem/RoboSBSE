//! Program Repair and Genetic Improvement: search for small source patches to a Java controller.
//! Candidates are compiled and tested by the engine in isolated worker processes; this page shows the
//! ranked patches (or improvements), their validation and the original and patched code side by side.

use std::collections::HashMap;

use eframe::egui::{self, Color32, RichText};
use serde_json::{json, Value};

use crate::app::{widgets, RoboApp};
use crate::canvas;
use crate::model::{self, Capabilities};
use crate::params::{self, Values};
use crate::runs::{RunKind, RunStatus};

pub const REPAIR: &str = "program-repair";
pub const IMPROVEMENT: &str = "genetic-improvement";

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum RepairTab {
    Progress,
    Tests,
    Log,
}

pub struct RepairState {
    /// Problem type: program-repair or genetic-improvement.
    pub mode: String,
    pub problems: HashMap<String, Values>,
    pub algorithm: String,
    pub algo_values: HashMap<String, Values>,
    pub budget: u64,
    pub seed: u64,
    pub threads: u64,
    pub run_id: Option<String>,
    pub result: Option<Value>,
    pub selected: usize,
    pub tab: RepairTab,
    pub context: usize,
}

impl RepairState {
    pub fn new() -> RepairState {
        RepairState {
            mode: REPAIR.into(),
            problems: HashMap::new(),
            algorithm: "genetic-algorithm".into(),
            algo_values: HashMap::new(),
            budget: 300,
            seed: 1,
            threads: 2,
            run_id: None,
            result: None,
            selected: 0,
            tab: RepairTab::Progress,
            context: 4,
        }
    }

    pub fn on_capabilities(&mut self, caps: &Capabilities) {
        for id in [REPAIR, IMPROVEMENT] {
            if let Some(p) = caps.problem(id) {
                let values = self.problems.entry(id.into()).or_default();
                for (k, v) in params::defaults(&p.params) {
                    values.entry(k).or_insert(v);
                }
            }
        }
        for a in &caps.algorithms {
            let values = self.algo_values.entry(a.id.clone()).or_default();
            for (k, v) in params::defaults(&a.params) {
                values.entry(k).or_insert(v);
            }
        }
        for id in ["genetic-algorithm", "nsga2"] {
            if let Some(v) = self.algo_values.get_mut(id) {
                v.insert("populationSize".into(), json!(20));
            }
        }
    }

    fn values(&mut self) -> &mut Values {
        self.problems.entry(self.mode.clone()).or_default()
    }

    pub fn experiment(&self, caps: &Capabilities) -> Value {
        let specs = caps.problem(&self.mode).map(|p| p.params.clone()).unwrap_or_default();
        let algo_specs = caps.algorithm(&self.algorithm).map(|a| a.params.clone()).unwrap_or_default();
        let operators = caps.problem(&self.mode).map(|p| p.operators.clone());
        let empty = Values::new();
        let mut problem_values = self.problems.get(&self.mode).cloned().unwrap_or_default();
        problem_values.insert("workers".into(), json!(self.threads));
        json!({
            "schema": "robosbse.experiment/1",
            "name": if self.mode == REPAIR { "program-repair" } else { "genetic-improvement" },
            "seed": self.seed,
            "budget": {"maxEvaluations": self.budget},
            "problem": {"type": self.mode, "params": params::to_json(&specs, &problem_values)},
            "algorithm": {
                "type": self.algorithm,
                "params": params::to_json_for(&algo_specs, self.algo_values.get(&self.algorithm).unwrap_or(&empty), operators.as_ref()),
            },
            "options": {"archiveSize": 0, "threads": self.threads, "maxFailureRecords": 8, "maxTargets": 100}
        })
    }

    pub fn on_finished(&mut self, result: &Value) {
        self.result = Some(result.clone());
        self.selected = 0;
    }

    /// Patches (repair) or improvements (genetic improvement) of the last result.
    fn candidates(&self) -> Vec<Value> {
        let r = self.result.as_ref();
        let key = if self.result_mode() == REPAIR { "patches" } else { "improvements" };
        r.and_then(|r| r[key].as_array().cloned()).unwrap_or_default()
    }

    fn result_mode(&self) -> &str {
        self.result.as_ref().and_then(|r| r["experiment"]["problem"]["type"].as_str()).unwrap_or(REPAIR)
    }
}

impl RoboApp {
    pub fn start_repair_run(&mut self) {
        let Some(caps) = self.caps.clone() else { return };
        let experiment = self.repair.experiment(&caps);
        if let Some(id) = self.start_run(RunKind::Repair, experiment) {
            self.repair.run_id = Some(id);
            self.repair.result = None;
            self.repair.tab = RepairTab::Progress;
        }
    }

    fn repair_config(&mut self, ui: &mut egui::Ui) {
        let Some(caps) = self.caps.clone() else {
            ui.label("Waiting for the engine...");
            return;
        };
        let running = self.run(&self.repair.run_id).map(|r| r.status.active()).unwrap_or(false);
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("Source patches");
            ui.horizontal(|ui| {
                ui.radio_value(&mut self.repair.mode, REPAIR.to_string(), "Program repair");
                ui.radio_value(&mut self.repair.mode, IMPROVEMENT.to_string(), "Genetic improvement");
            });
            let purpose = if self.repair.mode == REPAIR { "program-repair" } else { "genetic-improvement" };
            let current = self.repair.values().get("subject").and_then(Value::as_str).unwrap_or("").to_string();
            ui.label(RichText::new("Subject").strong());
            egui::ComboBox::from_id_salt("repair-subject")
                .selected_text(caps.subjects.iter().find(|s| s.id == current).map(|s| s.name.clone()).unwrap_or(current.clone()))
                .width(300.0)
                .show_ui(ui, |ui| {
                    for s in caps.subjects.iter().filter(|s| s.purpose == purpose) {
                        if ui.selectable_label(s.id == current, &s.name).clicked() {
                            self.repair.values().insert("subject".into(), json!(s.id));
                        }
                    }
                });
            if let Some(s) = caps.subjects.iter().find(|s| s.id == current) {
                let text = match (&s.defect_kind, &s.defect) {
                    (Some(kind), Some(defect)) => format!("Injected defect ({kind}): {defect}"),
                    _ => s.description.clone(),
                };
                ui.add(egui::Label::new(RichText::new(text).small()).wrap());
            }
            ui.add_space(6.0);
            ui.label(RichText::new("Tests and limits").strong());
            if let Some(p) = caps.problem(&self.repair.mode) {
                let values = self.repair.problems.entry(self.repair.mode.clone()).or_default();
                params::form(ui, "repair-problem", &p.params, values, &["subject", "workers"]);
            }
            ui.add_space(6.0);
            ui.separator();
            ui.label(RichText::new("Search").strong());
            let name = caps.algorithm(&self.repair.algorithm).map(|a| a.name.clone()).unwrap_or_default();
            let mode = self.repair.mode.clone();
            egui::ComboBox::from_id_salt("repair-alg").selected_text(name).width(220.0).show_ui(ui, |ui| {
                for a in caps.algorithms_for(&mode) {
                    ui.selectable_value(&mut self.repair.algorithm, a.id.clone(), &a.name);
                }
            });
            if let Some(a) = caps.algorithm(&self.repair.algorithm) {
                let operators = caps.problem(&mode).map(|p| p.operators.clone());
                let values = self.repair.algo_values.entry(a.id.clone()).or_default();
                params::form_for(ui, "repair-alg-params", &a.params, values, &[], operators.as_ref());
            }
            egui::Grid::new("repair-budget").num_columns(2).show(ui, |ui| {
                ui.label("Candidate evaluations");
                ui.add(egui::DragValue::new(&mut self.repair.budget).range(1..=1_000_000).speed(10));
                ui.end_row();
                ui.label("Random seed");
                ui.add(egui::DragValue::new(&mut self.repair.seed));
                ui.end_row();
                ui.label("Worker processes")
                    .on_hover_text("Candidates are compiled and tested in separate JVMs (heap limit, time limit).");
                ui.add(egui::DragValue::new(&mut self.repair.threads).range(1..=16));
                ui.end_row();
            });
            ui.add_space(8.0);
            ui.horizontal_wrapped(|ui| {
                let label = if self.repair.mode == REPAIR { "Search for patches" } else { "Search for improvements" };
                if ui.add_enabled(!running, egui::Button::new(RichText::new(label).strong())).clicked() {
                    self.start_repair_run();
                }
                let id = self.repair.run_id.clone();
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
            ui.add_space(6.0);
            ui.add(egui::Label::new(RichText::new(
                "Passing a test suite does not prove a patch correct. Plausible patches are minimised and checked on \
                 held-out tests and against the reference fix; treat them as candidates for review.").small().weak()).wrap());
        });
    }

    fn repair_bottom(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.selectable_value(&mut self.repair.tab, RepairTab::Progress, "Progress");
            ui.selectable_value(&mut self.repair.tab, RepairTab::Tests, "Tests of the original");
            ui.selectable_value(&mut self.repair.tab, RepairTab::Log, "Log");
        });
        ui.separator();
        match self.repair.tab {
            RepairTab::Progress => match self.runs.get(self.repair.run_id.as_deref().unwrap_or("")) {
                Some(run) => {
                    let target = if self.repair.mode == REPAIR { "plausible patches" } else { "valid improvements" };
                    widgets::run_status(ui, run, target);
                    let best = &run.best_solution;
                    if best["edits"].is_array() {
                        ui.label(RichText::new(format!("current best: {}", best["edits"])).small().monospace());
                    }
                    let h = ui.available_height().max(80.0);
                    widgets::convergence_plot(ui, "repair-conv", run, h - 8.0, false);
                }
                None => {
                    ui.label("Choose a subject on the right and start a search.");
                }
            },
            RepairTab::Tests => {
                let tests = self.repair.result.as_ref().map(|r| r["reference"]["original"]["tests"].clone()).unwrap_or(Value::Null);
                match tests.as_array() {
                    Some(list) => {
                        egui::ScrollArea::vertical().id_salt("repair-tests").show(ui, |ui| {
                            egui::Grid::new("repair-tests-grid").striped(true).num_columns(5).show(ui, |ui| {
                                for h in ["test", "kind", "unpatched subject", "outcome", "violated"] {
                                    ui.label(RichText::new(h).strong());
                                }
                                ui.end_row();
                                for t in list {
                                    ui.monospace(t["name"].as_str().unwrap_or(""));
                                    ui.label(t["kind"].as_str().unwrap_or(""));
                                    let passed = t["passed"].as_bool() == Some(true);
                                    ui.label(RichText::new(if passed { "passes" } else { "FAILS" })
                                        .color(if passed { canvas::START } else { canvas::DANGER }));
                                    ui.monospace(t["outcome"].as_str().unwrap_or(""));
                                    ui.monospace(t["violated"].to_string());
                                    ui.end_row();
                                }
                            });
                        });
                    }
                    None => {
                        ui.label("The search tests (scenarios the faulty subject fails or passes) appear after a run.");
                    }
                }
            }
            RepairTab::Log => crate::pages::lab::log_view(ui, &self.log),
        }
    }

    fn repair_central(&mut self, ui: &mut egui::Ui) {
        let Some(result) = self.repair.result.clone() else {
            ui.add_space(20.0);
            ui.label("Results of a repair or improvement search appear here: the ranked candidates and the original and \
                      patched source side by side.");
            return;
        };
        let mode = self.repair.result_mode().to_string();
        let info = if mode == REPAIR { &result["repair"] } else { &result["improvement"] };
        ui.horizontal_wrapped(|ui| {
            ui.heading(info["subject"]["name"].as_str().unwrap_or(""));
            ui.label(format!(
                "{} mutation points, {} single edits  |  candidates compiled {}, cache hits {}, compile errors {}, timeouts {}, crashes {}",
                info["subject"]["mutationPoints"], info["subject"]["singleEdits"], info["candidates"]["evaluated"],
                info["candidates"]["cacheHits"], info["candidates"]["compileErrors"], info["candidates"]["timeouts"],
                info["candidates"]["crashes"]
            ));
        });
        if mode == REPAIR {
            ui.label(format!(
                "search tests: {} failing + {} passing  |  held-out: {} failing + {} passing",
                info["searchTests"]["failing"], info["searchTests"]["passing"], info["heldOutTests"]["failing"],
                info["heldOutTests"]["passing"]
            ));
        } else {
            let b = &result["bestExtras"];
            ui.label(format!(
                "correctness tests: {} (held-out {})  |  baseline: time {:.2} s, work/step {:.1}  |  best: time ratio {}, work ratio {}",
                info["tests"], info["heldOutTests"], model::num(&b["baseline"]["meanTime"]),
                model::num(&b["baseline"]["workPerStep"]), widgets::fmt(model::num(&b["improved"]["timeRatio"])),
                widgets::fmt(model::num(&b["improved"]["workRatio"]))
            ));
        }
        let candidates = self.repair.candidates();
        if candidates.is_empty() {
            ui.label(RichText::new(if mode == REPAIR {
                "No plausible patch found within the budget."
            } else {
                "No valid improvement found within the budget."
            }).strong().color(canvas::GOAL));
            return;
        }
        let mut pick = None;
        egui::ScrollArea::vertical().id_salt("repair-list").max_height(170.0).show(ui, |ui| {
            egui::Grid::new("repair-list-grid").striped(true).num_columns(6).show(ui, |ui| {
                let headers: &[&str] = if mode == REPAIR {
                    &["#", "edits", "held-out", "equivalent to fix", "identical", "found at"]
                } else {
                    &["#", "edits", "held-out", "time ratio", "work ratio", "found at"]
                };
                for h in headers {
                    ui.label(RichText::new(*h).strong());
                }
                ui.end_row();
                for (i, c) in candidates.iter().enumerate() {
                    if ui.selectable_label(self.repair.selected == i, format!("{}", i + 1)).clicked() {
                        pick = Some(i);
                    }
                    let edits: Vec<String> = c["edits"].as_array().map(|a| a.iter().filter_map(|e| e.as_str().map(str::to_string)).collect())
                        .unwrap_or_default();
                    ui.label(RichText::new(edits.join("\n")).small().monospace());
                    let held = c["passesHeldOut"].as_bool() == Some(true);
                    ui.label(RichText::new(format!("{} / {}", c["heldOutPassed"], c["heldOutTests"]))
                        .color(if held { canvas::START } else { canvas::DANGER }));
                    if mode == REPAIR {
                        yes_no(ui, c["equivalentToReferenceFix"].as_bool() == Some(true));
                        yes_no(ui, c["identicalToReferenceFix"].as_bool() == Some(true));
                    } else {
                        ui.monospace(widgets::fmt(model::num(&c["metrics"]["timeRatio"])));
                        ui.monospace(widgets::fmt(model::num(&c["metrics"]["workRatio"])));
                    }
                    ui.monospace(format!("{}", c["foundAtEvaluation"]));
                    ui.end_row();
                }
            });
        });
        if let Some(i) = pick {
            self.repair.selected = i;
        }
        ui.separator();
        ui.horizontal(|ui| {
            ui.label(RichText::new("Original (left) and patched (right) source").strong());
            ui.label("context lines");
            ui.add(egui::DragValue::new(&mut self.repair.context).range(0..=400));
        });
        let selected = candidates.get(self.repair.selected).cloned().unwrap_or(Value::Null);
        diff_view(ui, &selected["diff"], self.repair.context, self.dark());
    }

    pub fn repair_page(&mut self, ui: &mut egui::Ui) {
        egui::Panel::right("repair-config").default_size(360.0).resizable(true).show(ui, |ui| self.repair_config(ui));
        egui::Panel::bottom("repair-bottom").default_size(220.0).resizable(true).show(ui, |ui| self.repair_bottom(ui));
        egui::CentralPanel::default().show(ui, |ui| self.repair_central(ui));
    }
}

fn yes_no(ui: &mut egui::Ui, yes: bool) {
    ui.label(RichText::new(if yes { "yes" } else { "no" }).color(if yes { canvas::START } else { ui.visuals().weak_text_color() }));
}

/// Two-column diff with line numbers; unchanged lines further than `context` from a change are elided.
fn diff_view(ui: &mut egui::Ui, rows: &Value, context: usize, dark: bool) {
    let Some(rows) = rows.as_array() else {
        ui.label("No diff available.");
        return;
    };
    let changed: Vec<usize> = rows.iter().enumerate().filter(|(_, r)| r["changed"].as_bool() == Some(true)).map(|(i, _)| i).collect();
    let near = |i: usize| changed.iter().any(|&c| c.abs_diff(i) <= context);
    let highlight = if dark { Color32::from_rgb(70, 50, 20) } else { Color32::from_rgb(255, 236, 190) };
    egui::ScrollArea::both().id_salt("repair-diff").show(ui, |ui| {
        egui::Grid::new("repair-diff-grid").num_columns(4).spacing([8.0, 0.0]).show(ui, |ui| {
            let mut elided = false;
            for (i, r) in rows.iter().enumerate() {
                if !near(i) {
                    if !elided {
                        ui.label(RichText::new("...").weak());
                        ui.label("");
                        ui.label(RichText::new("...").weak());
                        ui.label("");
                        ui.end_row();
                        elided = true;
                    }
                    continue;
                }
                elided = false;
                let is_changed = r["changed"].as_bool() == Some(true);
                let line = |n: &Value| n.as_u64().map(|n| n.to_string()).unwrap_or_default();
                let text = |t: &Value| {
                    let mut rt = RichText::new(t.as_str().unwrap_or("")).monospace();
                    if is_changed {
                        rt = rt.background_color(highlight);
                    }
                    rt
                };
                ui.label(RichText::new(line(&r["leftLine"])).monospace().weak());
                ui.label(text(&r["left"]));
                ui.label(RichText::new(line(&r["rightLine"])).monospace().weak());
                ui.label(text(&r["right"]));
                ui.end_row();
            }
        });
    });
}
