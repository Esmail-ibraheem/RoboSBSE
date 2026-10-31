//! Search-Based Testing: search for scenarios that make a controller violate its declared
//! properties, inspect and replay discovered failures, reproduce them and save them as regression
//! tests.

use std::collections::{BTreeSet, HashMap};

use eframe::egui::{self, RichText};
use serde_json::{json, Value};

use crate::app::{widgets, MapTarget, RoboApp};
use crate::canvas::{self, Overlay, View};
use crate::model::{self, Capabilities, Point, Pose, Shape, WorldMap};
use crate::params::{self, Values};
use crate::runs::{Replay, RunKind, RunStatus};

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum TestTab {
    Progress,
    Failures,
    Coverage,
    Log,
}

pub struct TestingState {
    pub arena: WorldMap,
    pub problem: Values,
    pub properties: BTreeSet<String>,
    pub algorithm: String,
    pub algo_values: HashMap<String, Values>,
    pub budget: u64,
    pub seed: u64,
    pub name: String,
    pub minimize: bool,
    pub stop_on_failure: bool,
    pub run_id: Option<String>,
    pub view: View,
    pub replay: Replay,
    pub scenario: Option<Value>,
    pub verdicts: Value,
    pub failures: Vec<Value>,
    pub selected: Option<usize>,
    pub reproduction: Option<Value>,
    pub tab: TestTab,
    pub show_rays: bool,
    /// Fitness evaluations computed in parallel (results do not depend on it).
    pub threads: u64,
    pub coverage_budget: u64,
    pub coverage_id: Option<String>,
    pub coverage_status: String,
    /// Targets reported so far by the running campaign (or all, once finished).
    pub coverage_targets: Vec<Value>,
    pub coverage_total: u64,
    pub coverage: Option<Value>,
    /// Branch coverage of all scenarios of the last test-generation run.
    pub run_coverage: Value,
    pub pending_simulation: Option<String>,
    /// Decision outcomes covered by the displayed scenario, and which target it was replayed for.
    pub covered_outcomes: Vec<String>,
    pub covering: Option<String>,
    /// Show targets that earlier searches covered by accident.
    pub show_collateral: bool,
}

impl TestingState {
    pub fn new(arena: WorldMap) -> TestingState {
        TestingState {
            arena,
            problem: Values::new(),
            properties: ["collision", "bounds", "valid-command", "velocity", "completion"]
                .iter()
                .map(|s| s.to_string())
                .collect(),
            algorithm: "genetic-algorithm".into(),
            algo_values: HashMap::new(),
            budget: 1500,
            seed: 7,
            name: "failure-discovery".into(),
            minimize: true,
            stop_on_failure: false,
            run_id: None,
            view: View::default(),
            replay: Replay { speed: 1.0, ..Replay::default() },
            scenario: None,
            verdicts: Value::Null,
            failures: Vec::new(),
            selected: None,
            reproduction: None,
            tab: TestTab::Progress,
            show_rays: true,
            threads: 1,
            coverage_budget: 3000,
            coverage_id: None,
            coverage_status: String::new(),
            coverage_targets: Vec::new(),
            coverage_total: 0,
            coverage: None,
            run_coverage: Value::Null,
            pending_simulation: None,
            covered_outcomes: Vec::new(),
            covering: None,
            show_collateral: true,
        }
    }

    pub fn on_capabilities(&mut self, caps: &Capabilities) {
        if let Some(p) = caps.problem("controller-testing") {
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
        self.problem.insert("fitness".into(), json!("collision"));
        if let Some(values) = self.algo_values.get_mut("genetic-algorithm") {
            values.insert("populationSize".into(), json!(30));
        }
    }

    pub fn set_arena(&mut self, map: WorldMap) {
        self.arena = map;
        self.view.reset();
    }

    pub fn controller(&self) -> String {
        self.problem.get("controller").and_then(Value::as_str).unwrap_or("reactive").to_string()
    }

    pub fn experiment(&self, caps: &Capabilities) -> Value {
        let problem_specs = caps.problem("controller-testing").map(|p| p.params.clone()).unwrap_or_default();
        let algo_specs = caps.algorithm(&self.algorithm).map(|a| a.params.clone()).unwrap_or_default();
        let operators = caps.problem("controller-testing").map(|p| p.operators.clone());
        let mut problem = self.problem.clone();
        problem.insert("properties".into(), json!(self.properties.iter().cloned().collect::<Vec<_>>().join(",")));
        let empty = Values::new();
        json!({
            "schema": "robosbse.experiment/1",
            "name": self.name,
            "seed": self.seed,
            "budget": {"maxEvaluations": self.budget},
            "problem": {
                "type": "controller-testing",
                "map": self.arena.to_json(),
                "params": params::to_json(&problem_specs, &problem),
            },
            "algorithm": {
                "type": self.algorithm,
                "params": params::to_json_for(&algo_specs, self.algo_values.get(&self.algorithm).unwrap_or(&empty),
                    operators.as_ref()),
            },
            "options": {"minimizeFailures": self.minimize, "stopOnTarget": self.stop_on_failure,
                        "maxFailureRecords": 10, "archiveSize": 10, "threads": self.threads}
        })
    }

    /// A coverage campaign (robosbse.coverage/1) for the current controller, scenario space and generator.
    pub fn campaign(&self, caps: &Capabilities) -> Value {
        let experiment = self.experiment(caps);
        json!({
            "schema": "robosbse.coverage-campaign/1",
            "name": format!("coverage-{}", self.controller()),
            "seed": self.seed,
            "budget": {"maxEvaluations": self.coverage_budget},
            "problem": experiment["problem"],
            "algorithm": experiment["algorithm"],
            "randomBaseline": true,
            "threads": self.threads,
        })
    }

    pub fn on_coverage_progress(&mut self, m: &Value) {
        if m["id"].as_str() != self.coverage_id.as_deref() {
            return;
        }
        self.coverage_targets.push(m["target"].clone());
        self.coverage_total = m["total"].as_u64().unwrap_or(self.coverage_total);
        self.coverage_status = format!("running: {} / {} targets", m["finished"], self.coverage_total);
    }

    pub fn on_coverage_finished(&mut self, result: &Value) {
        self.coverage_targets = result["targets"].as_array().cloned().unwrap_or_default();
        self.coverage_total = self.coverage_targets.len() as u64;
        self.coverage_status = if result["cancelled"].as_bool() == Some(true) { "cancelled".into() } else { "finished".into() };
        self.coverage = Some(result.clone());
        self.coverage_id = None;
    }

    /// A covering scenario re-simulated by the engine (`simulate` reply).
    pub fn on_simulation(&mut self, m: &Value) {
        let replay = &m["replay"];
        self.scenario = Some(replay["scenario"].clone());
        self.verdicts = replay["verdicts"].clone();
        self.covered_outcomes = string_list(&replay["coveredOutcomes"]);
        let title = match &self.covering {
            Some(label) => format!("scenario covering {label}"),
            None => "scenario".into(),
        };
        self.replay.load(&replay["trajectory"], &title, replay["outcome"].as_str().unwrap_or(""), None,
            replay["metrics"].clone());
    }

    pub fn apply_experiment(&mut self, cfg: &Value) -> Result<Option<Value>, String> {
        self.name = cfg["name"].as_str().unwrap_or("experiment").to_string();
        self.seed = cfg["seed"].as_u64().unwrap_or(self.seed);
        self.budget = cfg["budget"]["maxEvaluations"].as_u64().unwrap_or(self.budget);
        if let Some(obj) = cfg["problem"]["params"].as_object() {
            for (k, v) in obj {
                if k == "properties" {
                    let spec = v.as_str().unwrap_or("safety");
                    if spec != "safety" && spec != "all" {
                        self.properties = spec.split(',').map(|s| s.trim().to_string()).collect();
                    }
                } else {
                    self.problem.insert(k.clone(), v.clone());
                }
            }
        }
        if let Some(alg) = cfg["algorithm"]["type"].as_str() {
            self.algorithm = alg.into();
            let values = self.algo_values.entry(alg.into()).or_default();
            if let Some(obj) = cfg["algorithm"]["params"].as_object() {
                for (k, v) in obj {
                    values.insert(k.clone(), v.clone());
                }
            }
        }
        self.minimize = cfg["options"]["minimizeFailures"].as_bool().unwrap_or(self.minimize);
        let map = &cfg["problem"]["map"];
        if map.is_object() {
            self.set_arena(serde_json::from_value(map.clone()).map_err(|e| e.to_string())?);
            Ok(None)
        } else {
            Ok(Some(map.clone()))
        }
    }

    pub fn on_finished(&mut self, result: &Value) {
        self.failures = result["failures"].as_array().cloned().unwrap_or_default();
        self.selected = None;
        self.reproduction = None;
        let replay = &result["replay"];
        let title = if self.failures.is_empty() { "most critical scenario (no failure)" } else { "best scenario" };
        self.scenario = Some(replay["scenario"].clone());
        self.verdicts = replay["verdicts"].clone();
        self.covered_outcomes = string_list(&replay["coveredOutcomes"]);
        self.covering = None;
        self.run_coverage = result["coverage"].clone();
        self.replay.load(&replay["trajectory"], title, replay["outcome"].as_str().unwrap_or(""), None,
            replay["metrics"].clone());
        self.tab = if self.failures.is_empty() { TestTab::Progress } else { TestTab::Failures };
    }

    pub fn on_reproduction(&mut self, m: &Value) {
        self.verdicts = m["verdicts"].clone();
        let reproduced = m["reproduced"].as_bool() == Some(true);
        self.replay.load(&m["trajectory"], if reproduced { "reproduced failure" } else { "replay (NOT reproduced)" },
            m["outcome"].as_str().unwrap_or(""), None, Value::Null);
        self.reproduction = Some(m.clone());
    }

    fn scenario_parts(&self) -> (Vec<Shape>, Option<Pose>, Option<Point>) {
        let Some(s) = &self.scenario else { return (vec![], None, None) };
        let obstacles: Vec<Shape> = serde_json::from_value(s["obstacles"].clone()).unwrap_or_default();
        let start: Option<Pose> = serde_json::from_value(s["start"].clone()).ok();
        let goal: Option<Point> = serde_json::from_value(s["goal"].clone()).ok();
        (obstacles, start, goal)
    }
}

impl RoboApp {
    pub fn start_testing_run(&mut self) {
        let Some(caps) = self.caps.clone() else { return };
        if self.testing.properties.is_empty() {
            return self.error("select at least one property");
        }
        let fitness = self.testing.problem.get("fitness").and_then(Value::as_str).unwrap_or("min").to_string();
        if fitness != "min" && !fitness.starts_with("branch:") && !self.testing.properties.contains(&fitness) {
            return self.error(format!("the fitness property '{fitness}' must be one of the selected properties"));
        }
        let branch_ids: Vec<String> = caps.controller(&self.testing.controller())
            .map(|c| c.branches.iter().map(|b| b.id.clone()).collect())
            .unwrap_or_default();
        if let Some(rest) = fitness.strip_prefix("branch:") {
            let decision = rest.rsplit_once(':').map(|(d, _)| d).unwrap_or(rest);
            if !branch_ids.iter().any(|b| b == decision) {
                return self.error(format!("the controller under test has no decision '{decision}'"));
            }
        }
        let experiment = self.testing.experiment(&caps);
        if let Some(id) = self.start_run(RunKind::Testing, experiment) {
            self.testing.run_id = Some(id);
            self.testing.failures.clear();
            self.testing.replay.clear();
            self.testing.scenario = None;
            self.testing.tab = TestTab::Progress;
        }
    }

    pub fn start_coverage_campaign(&mut self) {
        let Some(caps) = self.caps.clone() else { return };
        let campaign = self.testing.campaign(&caps);
        let output = self.settings.results_dir().display().to_string();
        if let Some(id) = self.request("coverage", json!({"campaign": campaign, "output": output})) {
            self.info(format!("started coverage campaign {id} ({} evaluations)", self.testing.coverage_budget));
            self.testing.coverage_id = Some(id);
            self.testing.coverage_targets.clear();
            self.testing.coverage = None;
            self.testing.coverage_status = "starting".into();
            self.testing.tab = TestTab::Coverage;
        }
    }

    /// Re-simulates the scenario that covered a target (engine `simulate`, recorded replay).
    pub fn replay_covering(&mut self, target: &Value) {
        let Some(caps) = self.caps.clone() else { return };
        let problem = match &self.testing.coverage {
            Some(result) if result["problem"].is_object() => result["problem"].clone(),
            _ => self.testing.experiment(&caps)["problem"].clone(),
        };
        if let Some(id) = self.request("simulate", json!({"problem": problem, "genome": target["scenario"]})) {
            self.testing.pending_simulation = Some(id);
            self.testing.covering = target["label"].as_str().map(str::to_string);
        }
    }

    fn coverage_tab(&mut self, ui: &mut egui::Ui) {
        let caps = self.caps.clone();
        let controller = self.testing.controller();
        let Some(info) = caps.as_ref().and_then(|c| c.controller(&controller).cloned()) else { return };
        if info.branches.is_empty() {
            ui.label(format!("Coverage is unavailable for {controller}: it has no instrumentation. Nothing is estimated."));
            return;
        }
        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new("Decision (branch) coverage").strong());
            ui.label(RichText::new(info.instrumentation.clone().unwrap_or_default()).small().weak());
        });
        ui.horizontal_wrapped(|ui| {
            ui.label("Campaign budget");
            ui.add(egui::DragValue::new(&mut self.testing.coverage_budget).range(10..=1_000_000).speed(100));
            let running = self.testing.coverage_id.is_some();
            if ui.add_enabled(!running, egui::Button::new(RichText::new("Run coverage campaign").strong()))
                .on_hover_text("Targets every decision outcome in turn with the selected generator (fitness = approach \
                    level + normalised branch distance), splitting the budget over the targets still uncovered, \
                    then measures random testing with the same budget.")
                .clicked()
            {
                self.start_coverage_campaign();
            }
            if let Some(id) = self.testing.coverage_id.clone() {
                if ui.button("Cancel").clicked() {
                    self.control_run(&id, "cancel");
                }
            }
            ui.label(RichText::new(&self.testing.coverage_status).weak());
            ui.checkbox(&mut self.testing.show_collateral, "show collateral targets");
        });
        if let Some(r) = &self.testing.coverage {
            let s = &r["summary"];
            let mut line = format!("search-based: {} / {} outcomes ({:.1}%) in {} evaluations", s["covered"], s["outcomes"],
                100.0 * model::num(&s["coverage"]), s["evaluations"]);
            if r["random"].is_object() {
                line += &format!("   |   random testing, same budget: {} / {} ({:.1}%)", r["random"]["covered"], s["outcomes"],
                    100.0 * model::num(&r["random"]["coverage"]));
            }
            ui.label(RichText::new(line).monospace());
        }
        let run_cov = self.testing.run_coverage.clone();
        if run_cov["available"].as_bool() == Some(true) {
            ui.label(RichText::new(format!("last test-generation run: {} / {} outcomes over {} valid scenarios",
                run_cov["covered"], run_cov["outcomes"], run_cov["episodes"])).small());
        }
        if !self.testing.covered_outcomes.is_empty() {
            ui.label(RichText::new(format!("displayed scenario covers {} outcomes{}", self.testing.covered_outcomes.len(),
                self.testing.covering.as_ref().map(|c| format!(" (replayed for {c})")).unwrap_or_default())).small());
        }
        ui.separator();
        let mut replay: Option<Value> = None;
        let targets = self.testing.coverage_targets.clone();
        let shown: std::collections::HashSet<String> = self.testing.covered_outcomes.iter().cloned().collect();
        egui::ScrollArea::vertical().id_salt("coverage-table").show(ui, |ui| {
            egui::Grid::new("coverage-grid").striped(true).num_columns(7).show(ui, |ui| {
                for h in ["outcome", "status", "evaluations", "first hit", "approach / distance", "condition", ""] {
                    ui.label(RichText::new(h).strong());
                }
                ui.end_row();
                if targets.is_empty() {
                    // No campaign yet: list the decisions with the scenario counts of the last run.
                    let counts = run_cov["branches"].as_array().cloned().unwrap_or_default();
                    for b in &info.branches {
                        for outcome in [true, false] {
                            let label = format!("{}:{}", b.id, outcome);
                            let n = counts.iter().find(|c| c["id"] == b.id.as_str())
                                .map(|c| c[if outcome { "trueScenarios" } else { "falseScenarios" }].as_u64().unwrap_or(0));
                            ui.monospace(&label);
                            match n {
                                Some(0) => ui.label(RichText::new("not covered").color(canvas::DANGER)),
                                Some(n) => ui.label(format!("{n} scenarios")),
                                None => ui.label(RichText::new("-").weak()),
                            };
                            ui.label("");
                            ui.label("");
                            ui.label("");
                            ui.label(RichText::new(&b.description).small()).on_hover_text(match &b.parent {
                                Some(p) => format!("evaluated only under {p} (approach-level parent)"),
                                None => "evaluated in every control step".to_string(),
                            });
                            ui.label(if shown.contains(&label) { "covered by shown scenario" } else { "" });
                            ui.end_row();
                        }
                    }
                }
                for t in targets.iter().filter(|t| self.testing.show_collateral || t["status"] != "collateral") {
                    let label = t["label"].as_str().unwrap_or("").to_string();
                    ui.monospace(&label);
                    let status = t["status"].as_str().unwrap_or("");
                    let color = match status {
                        "targeted" => canvas::START,
                        "uncovered" => canvas::DANGER,
                        _ => ui.visuals().text_color(),
                    };
                    ui.label(RichText::new(status).color(color).strong());
                    ui.monospace(t["evaluations"].as_u64().map(|e| e.to_string()).unwrap_or_default());
                    ui.monospace(t["firstEvaluation"].as_u64().map(|e| e.to_string()).unwrap_or_default());
                    ui.monospace(if status == "uncovered" && t["approachLevel"].is_number() {
                        format!("{} / {}", t["approachLevel"], widgets::fmt(model::num(&t["branchDistance"])))
                    } else {
                        String::new()
                    });
                    ui.label(RichText::new(t["description"].as_str().unwrap_or("")).small()).on_hover_text(
                        match t["parent"].as_str() {
                            Some(p) => format!("evaluated only under {p} (approach-level parent)"),
                            None => "evaluated in every control step".to_string(),
                        });
                    ui.horizontal(|ui| {
                        if t["scenario"].is_array() && ui.small_button("replay").clicked() {
                            replay = Some(t.clone());
                        }
                        if shown.contains(&label) {
                            ui.label(RichText::new("shown").small().weak());
                        }
                    });
                    ui.end_row();
                }
            });
        });
        if let Some(t) = replay {
            self.replay_covering(&t);
        }
    }

    fn reproduce_failure(&mut self, failure: Value) {
        self.testing.scenario = Some(failure["scenario"].clone());
        self.request("reproduce", json!({"failure": failure}));
    }

    fn save_failure(&mut self, failure: &Value) {
        let dir = crate::settings::find_repo_root().join("examples/faulty-controllers/regressions");
        let name = format!(
            "{}-{}.json",
            failure["controller"].as_str().unwrap_or("controller"),
            failure["violated"].as_array().and_then(|a| a.first()).and_then(Value::as_str).unwrap_or("failure")
        );
        if let Some(path) = rfd::FileDialog::new().set_directory(dir).set_file_name(name).add_filter("failure", &["json"]).save_file() {
            match std::fs::write(&path, serde_json::to_string_pretty(failure).unwrap_or_default()) {
                Ok(_) => self.notify(format!("saved regression test {}", path.display())),
                Err(e) => self.error(format!("cannot save: {e}")),
            }
        }
    }

    fn load_failure_file(&mut self) {
        let dir = crate::settings::find_repo_root().join("examples/faulty-controllers/regressions");
        if let Some(path) = rfd::FileDialog::new().set_directory(dir).add_filter("failure", &["json"]).pick_file() {
            match std::fs::read_to_string(&path).ok().and_then(|t| serde_json::from_str::<Value>(&t).ok()) {
                Some(f) if f["schema"] == "robosbse.failure/1" => {
                    if let Ok(arena) = serde_json::from_value::<WorldMap>(f["arena"].clone()) {
                        self.testing.set_arena(arena);
                    }
                    self.testing.failures.push(f.clone());
                    self.testing.selected = Some(self.testing.failures.len() - 1);
                    self.testing.tab = TestTab::Failures;
                    self.reproduce_failure(f);
                }
                _ => self.error(format!("{} is not a robosbse.failure/1 record", path.display())),
            }
        }
    }

    fn testing_config(&mut self, ui: &mut egui::Ui) {
        let Some(caps) = self.caps.clone() else {
            ui.label("Waiting for the engine...");
            return;
        };
        let running = self.run(&self.testing.run_id).map(|r| r.status.active()).unwrap_or(false);
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("Failure discovery");
            ui.label(RichText::new("System under test").strong());
            let current = self.testing.controller();
            egui::ComboBox::from_id_salt("test-controller")
                .selected_text(caps.controller(&current).map(|c| c.name.clone()).unwrap_or(current.clone()))
                .width(260.0)
                .show_ui(ui, |ui| {
                    for c in caps.controllers.iter().filter(|c| c.kind == "reactive") {
                        if ui.selectable_label(current == c.id, &c.name).clicked() {
                            self.testing.problem.insert("controller".into(), json!(c.id));
                        }
                    }
                });
            if let Some(c) = caps.controller(&self.testing.controller()) {
                let text = match &c.defect {
                    Some(defect) => format!(
                        "Injected defect: {defect}\nExpected to violate: {}",
                        c.expected_property.clone().unwrap_or_default()
                    ),
                    None => format!("{} (reference implementation, no injected defect)", c.summary),
                };
                ui.add(egui::Label::new(RichText::new(text).small()).wrap());
            }
            ui.add_space(6.0);
            ui.label(RichText::new("Properties (test oracle)").strong());
            for p in &caps.properties {
                let mut on = self.testing.properties.contains(&p.id);
                if ui.checkbox(&mut on, &p.id).on_hover_text(&p.description).changed() {
                    if on {
                        self.testing.properties.insert(p.id.clone());
                    } else {
                        self.testing.properties.remove(&p.id);
                    }
                }
            }
            ui.horizontal(|ui| {
                ui.label("Fitness guide");
                let current = self.testing.problem.get("fitness").and_then(Value::as_str).unwrap_or("min").to_string();
                egui::ComboBox::from_id_salt("test-fitness").selected_text(&current).show_ui(ui, |ui| {
                    let mut choice = current.clone();
                    ui.selectable_value(&mut choice, "min".to_string(), "min (all selected)");
                    for p in &self.testing.properties {
                        ui.selectable_value(&mut choice, p.clone(), p);
                    }
                    if let Some(c) = caps.controller(&self.testing.controller()) {
                        if !c.branches.is_empty() {
                            ui.separator();
                            ui.label(RichText::new("white-box: reach a decision outcome first").small().weak());
                            for b in &c.branches {
                                for outcome in [true, false] {
                                    let value = format!("branch:{}:{}", b.id, outcome);
                                    ui.selectable_value(&mut choice, value.clone(), value)
                                        .on_hover_text(&b.description);
                                }
                            }
                        }
                    }
                    if choice != current {
                        self.testing.problem.insert("fitness".into(), json!(choice));
                    }
                });
            });
            ui.add(egui::Label::new(RichText::new(
                "Fitness = robustness (negative = violated). Guiding by one property avoids masking by others. \
                 branch:<decision>:<outcome> = approach level + branch distance until that outcome is covered, then robustness.")
                .small().weak()).wrap());
            ui.add_space(6.0);
            ui.label(RichText::new("Scenario space").strong());
            if let Some(problem) = caps.problem("controller-testing") {
                params::form(ui, "test-problem", &problem.params, &mut self.testing.problem,
                    &["controller", "properties", "fitness"]);
            }
            ui.add_space(6.0);
            ui.separator();
            ui.label(RichText::new("Test generator").strong());
            let current = caps.algorithm(&self.testing.algorithm).map(|a| a.name.clone()).unwrap_or_default();
            egui::ComboBox::from_id_salt("test-alg").selected_text(current).width(220.0).show_ui(ui, |ui| {
                for a in caps.algorithms_for("controller-testing") {
                    ui.selectable_value(&mut self.testing.algorithm, a.id.clone(), &a.name);
                }
            });
            if let Some(a) = caps.algorithm(&self.testing.algorithm) {
                let operators = caps.problem("controller-testing").map(|p| p.operators.clone());
                let values = self.testing.algo_values.entry(a.id.clone()).or_default();
                params::form_for(ui, "test-alg-params", &a.params, values, &[], operators.as_ref());
            }
            egui::Grid::new("test-budget").num_columns(2).show(ui, |ui| {
                ui.label("Fitness evaluations");
                ui.add(egui::DragValue::new(&mut self.testing.budget).range(1..=10_000_000).speed(50));
                ui.end_row();
                ui.label("Random seed");
                ui.add(egui::DragValue::new(&mut self.testing.seed));
                ui.end_row();
                ui.label("Evaluation threads").on_hover_text("Results are identical for every thread count.");
                ui.add(egui::DragValue::new(&mut self.testing.threads).range(1..=64));
                ui.end_row();
            });
            ui.checkbox(&mut self.testing.minimize, "minimise discovered failures");
            ui.checkbox(&mut self.testing.stop_on_failure, "stop at the first failure");
            ui.add_space(8.0);
            ui.horizontal_wrapped(|ui| {
                if ui.add_enabled(!running, egui::Button::new(RichText::new("Generate tests").strong())).clicked() {
                    self.start_testing_run();
                }
                let id = self.testing.run_id.clone();
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
            ui.horizontal_wrapped(|ui| {
                if ui.button("Load failure file").clicked() {
                    self.load_failure_file();
                }
                if ui.button("Load config").clicked() {
                    self.load_experiment_file();
                }
                if ui.button("Reset arena").clicked() {
                    self.request_map(json!("builtin:testing-arena"), MapTarget::Arena);
                }
            });
        });
    }

    fn testing_bottom(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.selectable_value(&mut self.testing.tab, TestTab::Progress, "Progress");
            let label = format!("Failures ({})", self.testing.failures.len());
            ui.selectable_value(&mut self.testing.tab, TestTab::Failures, label);
            ui.selectable_value(&mut self.testing.tab, TestTab::Coverage, "Coverage");
            ui.selectable_value(&mut self.testing.tab, TestTab::Log, "Log");
        });
        ui.separator();
        match self.testing.tab {
            TestTab::Progress => match self.runs.get(self.testing.run_id.as_deref().unwrap_or("")) {
                Some(run) => {
                    widgets::run_status(ui, run, "failing scenarios");
                    let h = ui.available_height().max(80.0);
                    let verdicts = self.testing.verdicts.clone();
                    ui.columns(2, |cols| {
                        widgets::convergence_plot(&mut cols[0], "test-conv", run, h - 8.0, false);
                        verdict_table(&mut cols[1], &verdicts);
                    });
                }
                None => {
                    ui.label("Choose a controller and press 'Generate tests'.");
                }
            },
            TestTab::Failures => {
                let mut replay: Option<usize> = None;
                let mut save: Option<usize> = None;
                ui.horizontal(|ui| {
                    if let Some(rep) = &self.testing.reproduction {
                        let ok = rep["reproduced"].as_bool() == Some(true);
                        let reference = &rep["referenceViolated"];
                        ui.label(RichText::new(if ok { "REPRODUCED" } else { "NOT REPRODUCED" })
                            .strong()
                            .color(if ok { canvas::START } else { canvas::DANGER }));
                        ui.label(format!("expected {}  replayed {}", rep["expected"], rep["violated"]));
                        if reference.is_array() {
                            ui.label(format!("| correct controller violates {}", reference));
                        }
                    }
                });
                egui::ScrollArea::vertical().show(ui, |ui| {
                    egui::Grid::new("failures").striped(true).num_columns(8).show(ui, |ui| {
                        for h in ["#", "violated", "outcome", "time s", "obstacles", "minimised", "found at", ""] {
                            ui.label(RichText::new(h).strong());
                        }
                        ui.end_row();
                        for (i, f) in self.testing.failures.iter().enumerate() {
                            let selected = self.testing.selected == Some(i);
                            if ui.selectable_label(selected, format!("{}", i + 1)).clicked() {
                                replay = Some(i);
                            }
                            ui.monospace(f["violated"].to_string());
                            ui.monospace(f["outcome"].as_str().unwrap_or(""));
                            ui.monospace(widgets::fmt(model::num(&f["time"])));
                            ui.monospace(format!("{}", f["scenario"]["obstacles"].as_array().map(|a| a.len()).unwrap_or(0)));
                            let m = &f["minimization"];
                            ui.monospace(if m.is_object() {
                                format!("{} -> {} obstacles, {} sims", m["originalObstacles"],
                                    f["scenario"]["obstacles"].as_array().map(|a| a.len()).unwrap_or(0), m["simulations"])
                            } else {
                                "no".into()
                            });
                            ui.monospace(format!("{}", f["provenance"]["evaluation"]));
                            ui.horizontal(|ui| {
                                if ui.small_button("replay + reproduce").clicked() {
                                    replay = Some(i);
                                }
                                if ui.small_button("save as regression test").clicked() {
                                    save = Some(i);
                                }
                            });
                            ui.end_row();
                        }
                    });
                    if self.testing.failures.is_empty() {
                        ui.label("No failures discovered (yet). Absence of failures is not proof of correctness.");
                    }
                });
                if let Some(i) = replay {
                    self.testing.selected = Some(i);
                    let f = self.testing.failures[i].clone();
                    self.reproduce_failure(f);
                }
                if let Some(i) = save {
                    let f = self.testing.failures[i].clone();
                    self.save_failure(&f);
                }
            }
            TestTab::Coverage => self.coverage_tab(ui),
            TestTab::Log => crate::pages::lab::log_view(ui, &self.log),
        }
    }

    pub fn testing_page(&mut self, ui: &mut egui::Ui) {
        egui::Panel::right("test-config").default_size(340.0).resizable(true).show(ui, |ui| self.testing_config(ui));
        egui::Panel::bottom("test-bottom").default_size(260.0).resizable(true).show(ui, |ui| self.testing_bottom(ui));
        egui::CentralPanel::default().show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                self.testing.replay.controls(ui);
                ui.separator();
                ui.checkbox(&mut self.testing.show_rays, "sensor rays");
                if ui.small_button("fit").clicked() {
                    self.testing.view.reset();
                }
            });
            if let Some(s) = &self.testing.scenario {
                ui.label(RichText::new(format!(
                    "scenario: noise {} m  |  scan dropout {}  |  battery {}  |  speed limit {} m/s  |  free distance {} m",
                    widgets::fmt(model::num(&s["sensorNoise"])),
                    widgets::fmt(model::num(&s["scanDropout"])),
                    widgets::fmt(model::num(&s["battery"])),
                    widgets::fmt(model::num(&s["speedLimit"])),
                    widgets::fmt(model::num(&s["freeDistance"])),
                )).small().monospace());
            }
            let (extra, start, goal) = self.testing.scenario_parts();
            let caption = Some(format!("{}  |  controller under test: {}", self.testing.arena.name, self.testing.controller()));
            let dark = self.dark();
            let t = &mut self.testing;
            let overlay = Overlay {
                extra_obstacles: &extra,
                start: start.as_ref(),
                goal: goal.as_ref(),
                trajectory: t.replay.trajectory.as_ref(),
                time: Some(t.replay.time),
                show_rays: t.show_rays,
                robot_radius: 0.25,
                sensor_range: 4.0,
                caption,
                ..Overlay::default()
            };
            canvas::show(ui, &mut t.view, &mut t.arena, &overlay, None, dark);
        });
    }
}

fn string_list(v: &Value) -> Vec<String> {
    v.as_array().map(|a| a.iter().filter_map(|s| s.as_str().map(str::to_string)).collect()).unwrap_or_default()
}

fn verdict_table(ui: &mut egui::Ui, verdicts: &Value) {
    let Some(list) = verdicts.as_array() else {
        ui.label("Verdicts of the displayed scenario appear here.");
        return;
    };
    egui::ScrollArea::vertical().id_salt("verdicts").show(ui, |ui| {
        egui::Grid::new("verdicts-grid").striped(true).num_columns(4).show(ui, |ui| {
            for h in ["property", "verdict", "robustness", "detail"] {
                ui.label(RichText::new(h).strong());
            }
            ui.end_row();
            for v in list {
                ui.monospace(v["property"].as_str().unwrap_or(""));
                let (text, color) = if v["applicable"].as_bool() == Some(false) {
                    ("n/a", ui.visuals().weak_text_color())
                } else if v["violated"].as_bool() == Some(true) {
                    ("VIOLATED", canvas::DANGER)
                } else {
                    ("ok", canvas::START)
                };
                ui.label(RichText::new(text).color(color).strong());
                ui.monospace(widgets::fmt(model::num(&v["robustness"])));
                ui.label(RichText::new(v["detail"].as_str().unwrap_or("")).small());
                ui.end_row();
            }
        });
    });
}
