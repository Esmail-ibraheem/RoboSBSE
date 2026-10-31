//! Simulation Laboratory: configure an optimisation experiment (route optimisation, controller
//! tuning, warehouse scheduling or multi-robot task allocation), run it in the engine, watch live
//! progress, inspect candidates and the Pareto front, and replay the robots executing the result.

use std::collections::HashMap;

use eframe::egui::{self, RichText};
use egui_plot::{Legend, Plot, PlotPoints, Points};
use serde_json::{json, Value};

use crate::app::{widgets, MapTarget, RoboApp};
use crate::canvas::{self, Conflict, FleetTrack, Marker, MarkerKind, Overlay, View};
use crate::model::{self, Capabilities, WorldMap};
use crate::params::{self, Values};
use crate::runs::{Replay, RunKind, RunStatus};

pub const NAVIGATION: &str = "navigation";
pub const TUNING: &str = "controller-tuning";
pub const WAREHOUSE: &str = "warehouse-scheduling";
pub const MULTI: &str = "multi-robot-allocation";

/// Problems with their own pages (not offered in the Laboratory's problem list).
pub fn has_own_page(problem: &str) -> bool {
    problem == "controller-testing" || problem.starts_with("arm-")
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum BottomTab {
    Progress,
    Candidates,
    Pareto,
    Result,
    Log,
}

pub struct LabState {
    pub map: WorldMap,
    pub map_source: String,
    pub problem_type: String,
    pub problem_values: HashMap<String, Values>,
    pub algorithm: String,
    pub algo_values: HashMap<String, Values>,
    pub budget: u64,
    pub seed: u64,
    /// Fitness evaluations computed in parallel (results do not depend on it).
    pub threads: u64,
    pub name: String,
    pub run_id: Option<String>,
    pub view: View,
    pub replay: Replay,
    pub reference: Option<Value>,
    pub show_reference: bool,
    pub show_candidates: bool,
    pub log_y: bool,
    pub tab: BottomTab,
    /// Map shown instead of `map` (tuning episodes, the warehouse of a task set).
    pub display_map: Option<WorldMap>,
    /// Tuning replays: one recorded episode per map.
    pub episodes: Vec<Value>,
    pub episode: usize,
    /// Warehouse stops and the visiting order of the shown schedule.
    pub markers: Vec<Marker>,
    pub schedule: Vec<Value>,
    /// Multi-robot replays: one track per robot, conflicts found after simulation, robot summaries.
    pub fleet: Vec<FleetTrack>,
    pub conflicts: Vec<Conflict>,
    pub fleet_robots: Vec<Value>,
    pub pareto_x: usize,
    pub pareto_y: usize,
    pub pareto_selected: Option<usize>,
    /// Series shown next to the convergence curve.
    pub series_choice: String,
}

impl LabState {
    pub fn new(map: WorldMap, source: &str) -> LabState {
        LabState {
            map,
            map_source: source.into(),
            problem_type: NAVIGATION.into(),
            problem_values: HashMap::new(),
            algorithm: "genetic-algorithm".into(),
            algo_values: HashMap::new(),
            budget: 2000,
            seed: 42,
            threads: 1,
            name: "lab-experiment".into(),
            run_id: None,
            view: View::default(),
            replay: Replay { speed: 2.0, ..Replay::default() },
            reference: None,
            show_reference: true,
            show_candidates: false,
            log_y: false,
            tab: BottomTab::Progress,
            display_map: None,
            episodes: Vec::new(),
            episode: 0,
            markers: Vec::new(),
            schedule: Vec::new(),
            fleet: Vec::new(),
            conflicts: Vec::new(),
            fleet_robots: Vec::new(),
            pareto_x: 0,
            pareto_y: 1,
            pareto_selected: None,
            series_choice: String::new(),
        }
    }

    pub fn on_capabilities(&mut self, caps: &Capabilities) {
        for p in &caps.problems {
            let values = self.problem_values.entry(p.id.clone()).or_default();
            for (k, v) in params::defaults(&p.params) {
                values.entry(k).or_insert(v);
            }
        }
        for a in &caps.algorithms {
            let values = self.algo_values.entry(a.id.clone()).or_default();
            for (k, v) in params::defaults(&a.params) {
                values.entry(k).or_insert(v);
            }
        }
    }

    pub fn values(&mut self) -> &mut Values {
        self.problem_values.entry(self.problem_type.clone()).or_default()
    }

    pub fn set_map(&mut self, map: WorldMap, source: &str) {
        self.map = map;
        self.display_map = None;
        self.map_source = source.into();
        self.reference = None;
        self.replay.clear();
        self.view.reset();
    }

    /// Clears everything that belongs to the previously shown result.
    pub fn clear_result_view(&mut self) {
        self.replay.clear();
        self.reference = None;
        self.episodes.clear();
        self.episode = 0;
        self.markers.clear();
        self.schedule.clear();
        self.fleet.clear();
        self.conflicts.clear();
        self.fleet_robots.clear();
        self.pareto_selected = None;
        if self.problem_type != WAREHOUSE && self.problem_type != MULTI {
            self.display_map = None;
        }
    }

    /// The `problem` section of an experiment for the current problem type.
    pub fn problem_json(&self, caps: &Capabilities) -> Value {
        let specs = caps.problem(&self.problem_type).map(|p| p.params.clone()).unwrap_or_default();
        let empty = Values::new();
        let values = self.problem_values.get(&self.problem_type).unwrap_or(&empty);
        let mut problem = json!({"type": self.problem_type, "params": params::to_json(&specs, values)});
        if self.problem_type == NAVIGATION {
            problem["map"] = self.map.to_json();
        }
        problem
    }

    pub fn experiment(&self, caps: &Capabilities) -> Value {
        let algo_specs = caps.algorithm(&self.algorithm).map(|a| a.params.clone()).unwrap_or_default();
        let operators = caps.problem(&self.problem_type).map(|p| p.operators.clone());
        let empty = Values::new();
        json!({
            "schema": "robosbse.experiment/1",
            "name": self.name,
            "seed": self.seed,
            "budget": {"maxEvaluations": self.budget},
            "problem": self.problem_json(caps),
            "algorithm": {
                "type": self.algorithm,
                "params": params::to_json_for(&algo_specs, self.algo_values.get(&self.algorithm).unwrap_or(&empty),
                    operators.as_ref()),
            },
            "options": {"archiveSize": 10, "threads": self.threads}
        })
    }

    /// Picks an algorithm that supports the current problem if the selected one does not.
    pub fn ensure_compatible_algorithm(&mut self, caps: &Capabilities) {
        let supported = caps.algorithms_for(&self.problem_type);
        if !supported.iter().any(|a| a.id == self.algorithm) {
            self.algorithm = supported
                .iter()
                .find(|a| a.id == "genetic-algorithm")
                .or(supported.first())
                .map(|a| a.id.clone())
                .unwrap_or_default();
        }
    }

    /// Applies a saved experiment configuration (navigation, tuning or scheduling).
    pub fn apply_experiment(&mut self, cfg: &Value) -> Result<Option<Value>, String> {
        let kind = cfg["problem"]["type"].as_str().unwrap_or(NAVIGATION).to_string();
        if ![NAVIGATION, TUNING, WAREHOUSE, MULTI].contains(&kind.as_str()) {
            return Err(format!("the Laboratory cannot open '{kind}' experiments"));
        }
        self.problem_type = kind.clone();
        self.clear_result_view();
        self.name = cfg["name"].as_str().unwrap_or("experiment").to_string();
        self.seed = cfg["seed"].as_u64().unwrap_or(self.seed);
        self.budget = cfg["budget"]["maxEvaluations"].as_u64().unwrap_or(self.budget);
        self.threads = cfg["options"]["threads"].as_u64().unwrap_or(1).max(1);
        if let Some(obj) = cfg["problem"]["params"].as_object() {
            let values = self.values();
            for (k, v) in obj {
                // Inlined task sets are stored as references to the bundled set when possible.
                let value = if k == "tasks" && v.is_object() {
                    json!(format!("builtin:{}", v["name"].as_str().unwrap_or("orders-8")))
                } else if k == "fleet" && v.is_object() {
                    json!(v["name"].as_str().unwrap_or("deliveries-12"))
                } else {
                    v.clone()
                };
                values.insert(k.clone(), value);
            }
        }
        if let Some(alg) = cfg["algorithm"]["type"].as_str() {
            self.algorithm = alg.to_string();
            let values = self.algo_values.entry(alg.to_string()).or_default();
            if let Some(obj) = cfg["algorithm"]["params"].as_object() {
                for (k, v) in obj {
                    values.insert(k.clone(), v.clone());
                }
            }
        }
        let map = &cfg["problem"]["map"];
        if kind == NAVIGATION {
            if map.is_object() {
                let parsed: WorldMap = serde_json::from_value(map.clone()).map_err(|e| e.to_string())?;
                self.set_map(parsed, "experiment file");
                return Ok(None);
            }
            return Ok(Some(map.clone()));
        }
        if map.is_object() {
            self.display_map = serde_json::from_value(map.clone()).ok();
            return Ok(None);
        }
        Ok(match kind.as_str() {
            WAREHOUSE => Some(json!("builtin:warehouse")),
            MULTI => Some(json!("builtin:delivery-center")),
            _ => None,
        })
    }

    /// Shows a replay produced by the engine for any of the Lab's problem types.
    pub fn load_replay(&mut self, replay: &Value, title: &str) {
        match self.problem_type.as_str() {
            TUNING => {
                self.episodes = replay["episodes"].as_array().cloned().unwrap_or_default();
                self.episode = 0;
                self.show_episode(title);
            }
            MULTI => self.load_fleet(replay, title),
            WAREHOUSE => {
                self.markers = markers(&replay["stops"]);
                self.schedule = replay["schedule"].as_array().cloned().unwrap_or_default();
                annotate_order(&mut self.markers, &self.schedule);
                self.markers = merge_colocated(std::mem::take(&mut self.markers));
                self.replay.load(&replay["trajectory"], title, replay["outcome"].as_str().unwrap_or(""),
                    Some(model::points(&replay["route"])), replay["metrics"].clone());
            }
            _ => {
                self.replay.load(&replay["trajectory"], title, replay["outcome"].as_str().unwrap_or(""),
                    Some(model::points(&replay["route"])), replay["metrics"].clone());
            }
        }
    }

    /// Multi-robot replay: one track per robot; the replay clock follows the longest one.
    fn load_fleet(&mut self, replay: &Value, title: &str) {
        self.markers = merge_colocated(markers(&replay["stops"]));
        self.fleet_robots = replay["robots"].as_array().cloned().unwrap_or_default();
        self.fleet = self
            .fleet_robots
            .iter()
            .map(|r| FleetTrack {
                label: format!("R{}", r["robot"].as_u64().unwrap_or(0) + 1),
                trajectory: serde_json::from_value(r["trajectory"].clone()).unwrap_or_default(),
                route: model::points(&r["route"]),
            })
            .collect();
        self.conflicts = conflicts(&replay["conflicts"]);
        let longest = self
            .fleet_robots
            .iter()
            .max_by(|a, b| model::num(&a["simulatedTime"]).total_cmp(&model::num(&b["simulatedTime"])))
            .map(|r| r["trajectory"].clone())
            .unwrap_or(Value::Null);
        let label = format!("{title}: {} robots, {} conflict(s)", self.fleet.len(), self.conflicts.len());
        self.replay.load(&longest, label, replay["outcome"].as_str().unwrap_or(""), None, replay["metrics"].clone());
    }

    pub fn show_episode(&mut self, title: &str) {
        let Some(e) = self.episodes.get(self.episode).cloned() else { return };
        self.display_map = serde_json::from_value(e["map"].clone()).ok();
        let label = format!("{title}: {} ({})", e["map"]["name"].as_str().unwrap_or("?"), e["split"].as_str().unwrap_or(""));
        self.replay.load(&e["trajectory"], label, e["outcome"].as_str().unwrap_or(""), Some(model::points(&e["route"])),
            e["metrics"].clone());
        self.view.reset();
    }

    pub fn on_finished(&mut self, result: &Value) {
        // Always show a result on the map the run used, even if the editable map changed meanwhile.
        if let Ok(map) = serde_json::from_value::<WorldMap>(result["experiment"]["problem"]["map"].clone()) {
            self.display_map = if self.problem_type == NAVIGATION && map == self.map { None } else { Some(map) };
        }
        self.load_replay(&result["replay"], "best solution");
        if result.get("reference").is_some() {
            self.reference = Some(result["reference"].clone());
        }
        self.pareto_selected = None;
        self.tab = BottomTab::Result;
    }

    pub fn on_plan(&mut self, reference: &Value) {
        self.reference = Some(reference.clone());
        let astar = &reference["astar"];
        if self.problem_type == NAVIGATION && astar["found"].as_bool() == Some(true) {
            let mut route = vec![[self.map.start.x, self.map.start.y]];
            route.extend(model::points(&astar["route"]));
            self.replay.load(&astar["trajectory"], "A* reference route",
                if astar["metrics"]["success"].as_f64() == Some(1.0) { "REACHED_GOAL" } else { "FAILED" },
                Some(route), astar["metrics"].clone());
        }
    }

    pub fn on_simulation(&mut self, m: &Value) {
        let title = format!("candidate (fitness {})", widgets::fmt(model::num(&m["fitness"])));
        if m.get("replay").is_some() {
            self.load_replay(&m["replay"], &title);
        }
    }

    fn candidate_routes(&self, result: Option<&Value>) -> Vec<Vec<[f64; 2]>> {
        result
            .and_then(|r| r["archive"].as_array())
            .map(|a| a.iter().map(|c| model::points(&c["solution"]["route"])).filter(|r| r.len() > 1).collect())
            .unwrap_or_default()
    }
}

/// Markers for warehouse stops (`stops` array of a scheduling view or replay).
pub fn markers(stops: &Value) -> Vec<Marker> {
    stops
        .as_array()
        .map(|a| {
            a.iter()
                .map(|s| Marker {
                    x: model::num(&s["x"]),
                    y: model::num(&s["y"]),
                    label: s["label"].as_str().unwrap_or("").to_string(),
                    kind: match s["kind"].as_str() {
                        Some("pickup") => MarkerKind::Pickup,
                        Some("delivery") => MarkerKind::Delivery,
                        Some("charger") => MarkerKind::Charger,
                        _ => MarkerKind::Depot,
                    },
                })
                .collect()
        })
        .unwrap_or_default()
}

/// Appends the visiting order ("#3") to each marker that the schedule visits.
pub fn annotate_order(markers: &mut [Marker], schedule: &[Value]) {
    let mut order = 0;
    for v in schedule {
        if matches!(v["kind"].as_str(), Some("start") | Some("end")) {
            continue;
        }
        order += 1;
        let label = v["label"].as_str().unwrap_or("");
        if let Some(m) = markers.iter_mut().find(|m| m.label.split(' ').next() == Some(label)) {
            m.label.push_str(&format!(" #{order}"));
        }
    }
}

/// Merges markers at the same place (several orders delivered to one dock) into one marker whose
/// label lists them all, so labels do not overprint each other.
pub fn merge_colocated(markers: Vec<Marker>) -> Vec<Marker> {
    let mut out: Vec<Marker> = Vec::new();
    for m in markers {
        match out.iter_mut().find(|o| (o.x - m.x).abs() < 0.05 && (o.y - m.y).abs() < 0.05 && o.kind == m.kind) {
            Some(existing) => {
                existing.label.push_str(", ");
                existing.label.push_str(&m.label);
            }
            None => out.push(m),
        }
    }
    out
}

/// Robot-robot conflicts of a multi-robot replay.
pub fn conflicts(value: &Value) -> Vec<Conflict> {
    value
        .as_array()
        .map(|a| {
            a.iter()
                .map(|c| Conflict {
                    robots: (c["robots"][0].as_u64().unwrap_or(0) as usize, c["robots"][1].as_u64().unwrap_or(0) as usize),
                    x: model::num(&c["x"]),
                    y: model::num(&c["y"]),
                    start: model::num(&c["start"]),
                    end: model::num(&c["end"]),
                })
                .collect()
        })
        .unwrap_or_default()
}

/// Visiting order of a schedule as a polyline.
fn schedule_points(schedule: &Value) -> Vec<[f64; 2]> {
    schedule
        .as_array()
        .map(|a| a.iter().map(|v| [model::num(&v["x"]), model::num(&v["y"])]).collect())
        .unwrap_or_default()
}

/// Metrics shown per candidate for each problem type.
fn candidate_metrics(problem: &str) -> &'static [&'static str] {
    match problem {
        TUNING => &["successRate", "meanTime", "meanRisk", "meanEnergy"],
        WAREHOUSE => &["makespan", "tardiness", "lateTasks", "charges"],
        MULTI => &["makespan", "totalDistance", "imbalance"],
        _ => &["success", "time", "energy", "risk"],
    }
}

/// Index of the point nearest to (x, y), with each axis normalised by its range.
pub fn nearest_point(points: &[[f64; 2]], x: f64, y: f64) -> Option<usize> {
    if points.is_empty() {
        return None;
    }
    let range = |k: usize| {
        let lo = points.iter().map(|p| p[k]).fold(f64::INFINITY, f64::min);
        let hi = points.iter().map(|p| p[k]).fold(f64::NEG_INFINITY, f64::max);
        if hi > lo { hi - lo } else { 1.0 }
    };
    let (rx, ry) = (range(0), range(1));
    points
        .iter()
        .enumerate()
        .map(|(i, p)| (i, ((p[0] - x) / rx).powi(2) + ((p[1] - y) / ry).powi(2)))
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(i, _)| i)
}

impl RoboApp {
    pub fn start_lab_run(&mut self) {
        let Some(caps) = self.caps.clone() else {
            self.error("engine capabilities not available yet");
            return;
        };
        self.lab.ensure_compatible_algorithm(&caps);
        let experiment = self.lab.experiment(&caps);
        if let Some(id) = self.start_run(RunKind::Lab, experiment) {
            self.lab.run_id = Some(id);
            self.lab.clear_result_view();
            self.lab.tab = BottomTab::Progress;
        }
    }

    /// Switches the Laboratory to another problem type.
    pub fn set_lab_problem(&mut self, kind: &str) {
        if self.lab.problem_type == kind {
            return;
        }
        self.lab.problem_type = kind.to_string();
        self.lab.clear_result_view();
        self.lab.display_map = None;
        if let Some(caps) = self.caps.clone() {
            self.lab.ensure_compatible_algorithm(&caps);
        }
        match kind {
            WAREHOUSE => self.request_map(json!("builtin:warehouse"), MapTarget::LabDisplay),
            MULTI => self.request_map(json!("builtin:delivery-center"), MapTarget::LabDisplay),
            _ => {}
        }
    }

    /// Replays a solution of the shown run, using that run's own problem configuration.
    fn simulate_genome(&mut self, genome: &Value) {
        let Some(caps) = self.caps.clone() else { return };
        let problem = self
            .run(&self.lab.run_id)
            .map(|r| r.config["problem"].clone())
            .filter(|p| p.is_object())
            .unwrap_or_else(|| self.lab.problem_json(&caps));
        self.request("simulate", json!({"problem": problem, "genome": genome}));
    }

    fn save_lab_config(&mut self) {
        let Some(caps) = self.caps.clone() else { return };
        let config = self.lab.experiment(&caps);
        let dir = self.settings.configs_dir();
        let _ = std::fs::create_dir_all(&dir);
        if let Some(path) = rfd::FileDialog::new()
            .set_directory(&dir)
            .set_file_name(format!("{}.json", self.lab.name))
            .add_filter("experiment", &["json"])
            .save_file()
        {
            match std::fs::write(&path, serde_json::to_string_pretty(&config).unwrap_or_default()) {
                Ok(_) => self.notify(format!("saved experiment to {}", path.display())),
                Err(e) => self.error(format!("cannot save: {e}")),
            }
        }
    }

    pub fn load_experiment_file(&mut self) {
        let start = crate::settings::find_repo_root().join("examples").join("experiments");
        if let Some(path) = rfd::FileDialog::new().set_directory(start).add_filter("experiment", &["json"]).pick_file() {
            let text = match std::fs::read_to_string(&path) {
                Ok(t) => t,
                Err(e) => return self.error(format!("cannot read {}: {e}", path.display())),
            };
            let cfg: Value = match serde_json::from_str(&text) {
                Ok(v) => v,
                Err(e) => return self.error(format!("invalid JSON: {e}")),
            };
            self.apply_experiment_config(&cfg, &path.display().to_string());
        }
    }

    /// Routes a configuration to the Lab or the Testing page.
    pub fn apply_experiment_config(&mut self, cfg: &Value, label: &str) {
        let problem = cfg["problem"]["type"].as_str().unwrap_or(NAVIGATION);
        if problem == "controller-testing" {
            match self.testing.apply_experiment(cfg) {
                Ok(Some(map_ref)) => self.request_map(map_ref, MapTarget::Arena),
                Ok(None) => {}
                Err(e) => return self.error(e),
            }
            self.page = crate::app::Page::Testing;
        } else {
            let target = if problem == NAVIGATION { MapTarget::Lab } else { MapTarget::LabDisplay };
            match self.lab.apply_experiment(cfg) {
                Ok(Some(map_ref)) => self.request_map(map_ref, target),
                Ok(None) => {}
                Err(e) => return self.error(e),
            }
            self.page = crate::app::Page::Lab;
        }
        self.notify(format!("loaded experiment {label}"));
    }

    fn problem_section(&mut self, ui: &mut egui::Ui, caps: &Capabilities) {
        let Some(problem) = caps.problem(&self.lab.problem_type).cloned() else { return };
        ui.add(egui::Label::new(RichText::new(&problem.summary).small().weak()).wrap());
        let kind = self.lab.problem_type.clone();
        let values = self.lab.values();
        let weighted = values.get("objective").and_then(Value::as_str) == Some("weighted");
        let mut skip: Vec<&str> = Vec::new();
        if !weighted {
            skip.extend(["weightTime", "weightEnergy", "weightLength", "weightSmoothness", "weightRisk",
                "weightTardiness", "weightDistance"]);
        }
        match kind.as_str() {
            TUNING => {
                skip.extend(["maps", "validationMaps"]);
                for (key, title) in [("maps", "Training maps"), ("validationMaps", "Validation maps (held out)")] {
                    ui.label(RichText::new(title).strong());
                    let current: Vec<String> = values
                        .get(key)
                        .and_then(Value::as_str)
                        .unwrap_or("")
                        .split(',')
                        .map(|s| s.trim().to_string())
                        .filter(|s| !s.is_empty())
                        .collect();
                    let mut selected = current.clone();
                    ui.horizontal_wrapped(|ui| {
                        for m in &caps.maps {
                            let mut on = selected.contains(m);
                            if ui.checkbox(&mut on, m).changed() {
                                if on {
                                    selected.push(m.clone());
                                } else {
                                    selected.retain(|s| s != m);
                                }
                            }
                        }
                    });
                    if selected != current {
                        values.insert(key.to_string(), json!(selected.join(",")));
                    }
                }
            }
            WAREHOUSE => {
                skip.push("tasks");
                let current = values.get("tasks").and_then(Value::as_str).unwrap_or("builtin:orders-8").to_string();
                ui.horizontal(|ui| {
                    ui.label("Order book");
                    egui::ComboBox::from_id_salt("lab-tasks").selected_text(&current).show_ui(ui, |ui| {
                        for t in &caps.task_sets {
                            let reference = format!("builtin:{t}");
                            if ui.selectable_label(current == reference, t).clicked() {
                                values.insert("tasks".into(), json!(reference));
                            }
                        }
                    });
                });
            }
            MULTI => {
                skip.push("fleet");
                let current = values.get("fleet").and_then(Value::as_str).unwrap_or("deliveries-12").to_string();
                ui.horizontal(|ui| {
                    ui.label("Fleet and jobs");
                    egui::ComboBox::from_id_salt("lab-fleet").selected_text(&current).show_ui(ui, |ui| {
                        for f in &caps.fleets {
                            if ui.selectable_label(current == *f, f).clicked() {
                                values.insert("fleet".into(), json!(f));
                            }
                        }
                    });
                });
            }
            _ => {}
        }
        params::form(ui, "lab-problem", &problem.params, values, &skip);
    }

    fn lab_config_panel(&mut self, ui: &mut egui::Ui) {
        let Some(caps) = self.caps.clone() else {
            ui.label("Waiting for the engine...");
            return;
        };
        let running = self.run(&self.lab.run_id).map(|r| r.status.active()).unwrap_or(false);
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("Experiment");
            egui::Grid::new("lab-exp").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
                ui.label("Name");
                ui.text_edit_singleline(&mut self.lab.name);
                ui.end_row();
                ui.label("Problem type");
                let current = caps.problem(&self.lab.problem_type).map(|p| p.name.clone()).unwrap_or_default();
                let mut chosen: Option<String> = None;
                ui.horizontal(|ui| {
                    egui::ComboBox::from_id_salt("lab-problem-type").selected_text(current).width(190.0).show_ui(ui, |ui| {
                        for p in caps.problems.iter().filter(|p| !has_own_page(&p.id)) {
                            if ui.selectable_label(self.lab.problem_type == p.id, &p.name).clicked() {
                                chosen = Some(p.id.clone());
                            }
                        }
                    });
                    if ui.small_button("testing...").on_hover_text("Controller failure discovery").clicked() {
                        self.page = crate::app::Page::Testing;
                    }
                    if ui.small_button("arm...").on_hover_text("Robotic arm problems").clicked() {
                        self.page = crate::app::Page::Arm;
                    }
                });
                if let Some(kind) = chosen {
                    self.set_lab_problem(&kind);
                }
                ui.end_row();
                if self.lab.problem_type == NAVIGATION {
                    ui.label("Environment");
                    ui.horizontal(|ui| {
                        let mut chosen: Option<String> = None;
                        egui::ComboBox::from_id_salt("lab-map").selected_text(&self.lab.map.name).width(150.0).show_ui(ui, |ui| {
                            for m in &caps.maps {
                                if ui.selectable_label(self.lab.map.name == *m, m).clicked() {
                                    chosen = Some(m.clone());
                                }
                            }
                        });
                        if let Some(m) = chosen {
                            self.request_map(json!(format!("builtin:{m}")), MapTarget::Lab);
                        }
                        if ui.small_button("edit").clicked() {
                            self.page = crate::app::Page::Editor;
                        }
                    });
                    ui.end_row();
                }
            });
            ui.add_space(6.0);
            ui.label(RichText::new("Problem").strong());
            self.problem_section(ui, &caps);
            ui.add_space(8.0);
            ui.separator();
            ui.label(RichText::new("Algorithm").strong());
            let supported: Vec<_> = caps.algorithms_for(&self.lab.problem_type).into_iter().cloned().collect();
            let current = caps.algorithm(&self.lab.algorithm).map(|a| a.name.clone()).unwrap_or_default();
            egui::ComboBox::from_id_salt("lab-alg").selected_text(current).width(230.0).show_ui(ui, |ui| {
                for a in &supported {
                    ui.selectable_value(&mut self.lab.algorithm, a.id.clone(), format!("{} ({})", a.name, a.family));
                }
            });
            let hidden = caps.algorithms.len() - supported.len();
            if hidden > 0 {
                ui.label(RichText::new(format!("{hidden} algorithm(s) hidden: not applicable to this representation")).small().weak());
            }
            if let Some(a) = caps.algorithm(&self.lab.algorithm) {
                ui.add(egui::Label::new(RichText::new(&a.summary).small().weak()).wrap());
                let operators = caps.problem(&self.lab.problem_type).map(|p| p.operators.clone());
                let values = self.lab.algo_values.entry(a.id.clone()).or_default();
                params::form_for(ui, "lab-alg-params", &a.params, values, &[], operators.as_ref());
            }
            ui.add_space(8.0);
            ui.separator();
            ui.label(RichText::new("Budget and reproducibility").strong());
            egui::Grid::new("lab-budget").num_columns(2).show(ui, |ui| {
                ui.label("Fitness evaluations");
                ui.add(egui::DragValue::new(&mut self.lab.budget).range(1..=10_000_000).speed(50));
                ui.end_row();
                ui.label("Random seed");
                ui.horizontal(|ui| {
                    ui.add(egui::DragValue::new(&mut self.lab.seed));
                    if ui.small_button("new").clicked() {
                        self.lab.seed = (self.elapsed().as_nanos() as u64) % 1_000_000;
                    }
                });
                ui.end_row();
                ui.label("Evaluation threads")
                    .on_hover_text("Population-based algorithms evaluate each generation in parallel. \
                        The result is identical for every thread count; only the wall-clock time changes.");
                ui.add(egui::DragValue::new(&mut self.lab.threads).range(1..=64));
                ui.end_row();
            });
            ui.add_space(10.0);
            ui.horizontal_wrapped(|ui| {
                let run_id = self.lab.run_id.clone();
                let status = self.run(&run_id).map(|r| r.status.clone());
                if ui.add_enabled(!running, egui::Button::new(RichText::new("Start").strong())).clicked() {
                    self.start_lab_run();
                }
                if let (Some(id), Some(status)) = (run_id, status) {
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
                let label = match self.lab.problem_type.as_str() {
                    NAVIGATION => "A* / Dijkstra reference",
                    WAREHOUSE | MULTI => "Greedy reference",
                    _ => "Untuned defaults",
                };
                if ui.button(label).clicked() {
                    let problem = self.lab.problem_json(&caps);
                    self.request("plan", json!({"problem": problem}));
                }
                if ui.button("Save config").clicked() {
                    self.save_lab_config();
                }
                if ui.button("Load config").clicked() {
                    self.load_experiment_file();
                }
            });
        });
    }

    fn lab_bottom(&mut self, ui: &mut egui::Ui) {
        let has_objectives = self
            .caps
            .as_ref()
            .and_then(|c| c.problem(&self.lab.problem_type))
            .map(|p| !p.objectives.is_empty())
            .unwrap_or(false);
        ui.horizontal(|ui| {
            ui.selectable_value(&mut self.lab.tab, BottomTab::Progress, "Progress");
            ui.selectable_value(&mut self.lab.tab, BottomTab::Candidates, "Candidates");
            if has_objectives {
                ui.selectable_value(&mut self.lab.tab, BottomTab::Pareto, "Pareto front");
            }
            ui.selectable_value(&mut self.lab.tab, BottomTab::Result, "Result");
            ui.selectable_value(&mut self.lab.tab, BottomTab::Log, "Log");
        });
        ui.separator();
        let run_id = self.lab.run_id.clone();
        match self.lab.tab {
            BottomTab::Progress => match self.runs.get(run_id.as_deref().unwrap_or("")) {
                Some(run) => {
                    let target = match self.lab.problem_type.as_str() {
                        TUNING => "all-maps successes",
                        WAREHOUSE => "on-time schedules",
                        MULTI => "plans no slower than greedy",
                        _ => "feasible routes",
                    };
                    widgets::run_status(ui, run, target);
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut self.lab.log_y, "log scale");
                        if let Some(sim) = self.lab.replay.trajectory.as_ref() {
                            ui.separator();
                            ui.label(format!("replay: {:.1} s simulated", sim.duration()));
                        }
                    });
                    let h = ui.available_height().max(80.0);
                    let series = run.result.as_ref().map(|r| r["series"].clone()).unwrap_or(Value::Null);
                    ui.columns(2, |cols| {
                        widgets::convergence_plot(&mut cols[0], "lab-conv", run, h - 8.0, self.lab.log_y);
                        widgets::series_plot(&mut cols[1], "lab-series", &series, h - 8.0, &mut self.lab.series_choice);
                    });
                }
                None => {
                    ui.label("No run yet. Configure the experiment on the right and press Start.");
                }
            },
            BottomTab::Candidates => {
                let result = self.run(&run_id).and_then(|r| r.result.clone());
                let Some(result) = result else {
                    ui.label("Candidates are listed when a run has finished.");
                    return;
                };
                let names = candidate_metrics(&self.lab.problem_type);
                let mut simulate: Option<Value> = None;
                egui::ScrollArea::vertical().show(ui, |ui| {
                    egui::Grid::new("lab-candidates").striped(true).num_columns(names.len() + 4).show(ui, |ui| {
                        ui.label(RichText::new("#").strong());
                        ui.label(RichText::new("fitness").strong());
                        for n in names {
                            ui.label(RichText::new(*n).strong());
                        }
                        ui.label(RichText::new("found at").strong());
                        ui.label("");
                        ui.end_row();
                        for (i, c) in result["archive"].as_array().cloned().unwrap_or_default().iter().enumerate() {
                            ui.monospace(format!("{}", i + 1));
                            ui.monospace(widgets::fmt(model::num(&c["fitness"])));
                            for n in names {
                                ui.monospace(widgets::fmt(model::num(&c["metrics"][*n])));
                            }
                            ui.monospace(format!("{}", c["evaluation"]));
                            if ui.small_button("replay").clicked() {
                                simulate = Some(c["solution"]["genome"].clone());
                            }
                            ui.end_row();
                        }
                    });
                });
                if let Some(genome) = simulate {
                    self.simulate_genome(&genome);
                }
            }
            BottomTab::Pareto => self.pareto_tab(ui),
            BottomTab::Result => self.result_tab(ui),
            BottomTab::Log => log_view(ui, &self.log),
        }
    }

    fn pareto_tab(&mut self, ui: &mut egui::Ui) {
        let result = self.run(&self.lab.run_id).and_then(|r| r.result.clone());
        let Some(pareto) = result.as_ref().map(|r| r["pareto"].clone()).filter(|p| p.is_object()) else {
            ui.label("The Pareto front of non-dominated feasible solutions appears here when a run has finished. \
                      Every algorithm gets one; NSGA-II optimises it directly.");
            return;
        };
        let objectives: Vec<String> = pareto["objectives"]
            .as_array()
            .map(|a| a.iter().filter_map(|o| o.as_str().map(String::from)).collect())
            .unwrap_or_default();
        let points: Vec<Value> = pareto["points"].as_array().cloned().unwrap_or_default();
        let n = objectives.len();
        if n < 2 {
            ui.label("This problem has fewer than two objectives.");
            return;
        }
        self.lab.pareto_x = self.lab.pareto_x.min(n - 1);
        self.lab.pareto_y = self.lab.pareto_y.min(n - 1);
        ui.horizontal_wrapped(|ui| {
            ui.label(format!("{} non-dominated feasible solutions", points.len()));
            ui.separator();
            ui.label(format!(
                "normalised hypervolume {} (reference point {})",
                widgets::fmt(model::num(&pareto["hypervolume"])),
                pareto["referencePoint"]
            ));
            ui.separator();
            for (axis, label) in [(0, "x"), (1, "y")] {
                let current = if axis == 0 { self.lab.pareto_x } else { self.lab.pareto_y };
                let mut chosen = current;
                egui::ComboBox::from_id_salt(("pareto-axis", axis))
                    .selected_text(format!("{label}: {}", objectives[current]))
                    .show_ui(ui, |ui| {
                        for (i, o) in objectives.iter().enumerate() {
                            ui.selectable_value(&mut chosen, i, o);
                        }
                    });
                if axis == 0 {
                    self.lab.pareto_x = chosen;
                } else {
                    self.lab.pareto_y = chosen;
                }
            }
        });
        let (px, py) = (self.lab.pareto_x, self.lab.pareto_y);
        let xy: Vec<[f64; 2]> = points
            .iter()
            .map(|p| [model::num(&p["objectives"][px]), model::num(&p["objectives"][py])])
            .collect();
        let selected = self.lab.pareto_selected;
        let mut clicked: Option<usize> = None;
        let h = ui.available_height().max(120.0);
        ui.columns(2, |cols| {
            let response = Plot::new("pareto-plot")
                .height(h - 8.0)
                .legend(Legend::default())
                .x_axis_label(objectives[px].clone())
                .y_axis_label(objectives[py].clone())
                .allow_scroll(false)
                .show(&mut cols[0], |plot| {
                    plot.points(Points::new("non-dominated", PlotPoints::from(xy.clone())).radius(4.0).color(canvas::ROUTE));
                    if let Some(i) = selected.filter(|i| *i < xy.len()) {
                        plot.points(Points::new("selected", PlotPoints::from(vec![xy[i]])).radius(8.0).color(canvas::GOAL));
                    }
                    plot.pointer_coordinate()
                });
            if response.response.clicked() {
                if let Some(c) = response.inner {
                    clicked = nearest_point(&xy, c.x, c.y);
                }
            }
            egui::ScrollArea::vertical().id_salt("pareto-table").show(&mut cols[1], |ui| {
                egui::Grid::new("pareto-grid").striped(true).num_columns(n + 2).show(ui, |ui| {
                    ui.label(RichText::new("#").strong());
                    for o in &objectives {
                        ui.label(RichText::new(o).strong());
                    }
                    ui.label("");
                    ui.end_row();
                    let mut order: Vec<usize> = (0..points.len()).collect();
                    order.sort_by(|a, b| xy[*a][0].partial_cmp(&xy[*b][0]).unwrap_or(std::cmp::Ordering::Equal));
                    for i in order {
                        let text = RichText::new(format!("{}", i + 1)).monospace();
                        ui.label(if selected == Some(i) { text.color(canvas::GOAL) } else { text });
                        for k in 0..n {
                            ui.monospace(widgets::fmt(model::num(&points[i]["objectives"][k])));
                        }
                        if ui.small_button("replay").clicked() {
                            clicked = Some(i);
                        }
                        ui.end_row();
                    }
                });
            });
        });
        if let Some(i) = clicked {
            self.lab.pareto_selected = Some(i);
            let genome = points[i]["solution"]["genome"].clone();
            self.simulate_genome(&genome);
        }
    }

    fn result_tab(&mut self, ui: &mut egui::Ui) {
        let run_id = self.lab.run_id.clone();
        let result = self.run(&run_id).and_then(|r| r.result.clone());
        let config = self.run(&run_id).map(|r| r.config.clone());
        let reference = self.lab.reference.clone().unwrap_or(Value::Null);
        egui::ScrollArea::vertical().show(ui, |ui| {
            if let Some(result) = &result {
                ui.horizontal_wrapped(|ui| {
                    ui.label(format!(
                        "stop: {}  |  {} evaluations in {} ms  |  best found at evaluation {}",
                        result["stopReason"].as_str().unwrap_or("?"),
                        result["evaluations"],
                        result["elapsedMs"],
                        result["best"]["evaluation"]
                    ));
                    if let Some(dir) = result["outputDir"].as_str() {
                        if ui.small_button("open output folder").clicked() {
                            widgets::open_in_file_manager(std::path::Path::new(dir));
                        }
                    }
                });
            }
            let best = result.as_ref().map(|r| r["best"]["metrics"].clone()).unwrap_or(Value::Null);
            match self.lab.problem_type.as_str() {
                TUNING => {
                    let defaults = &reference["defaults"];
                    let validation = result.as_ref().map(|r| r["replay"]["validation"].clone()).unwrap_or(Value::Null);
                    let tuned = result.as_ref().map(|r| r["best"]["solution"]["params"].clone()).unwrap_or(Value::Null);
                    ui.columns(2, |cols| {
                        cols[0].label(RichText::new("Training maps").strong());
                        widgets::metrics_grid(&mut cols[0], "tune-train", &[("tuned", &best), ("defaults", &defaults["training"])]);
                        cols[0].label(RichText::new("Validation maps (never seen during search)").strong());
                        widgets::metrics_grid(&mut cols[0], "tune-val", &[("tuned", &validation), ("defaults", &defaults["validation"])]);
                        cols[1].label(RichText::new("Parameters").strong());
                        widgets::metrics_grid(&mut cols[1], "tune-params", &[("tuned", &tuned), ("defaults", &defaults["params"])]);
                    });
                }
                WAREHOUSE => {
                    let greedy = &reference["greedy"]["metrics"];
                    ui.columns(2, |cols| {
                        widgets::metrics_grid(&mut cols[0], "wh-metrics", &[("best schedule", &best), ("greedy dispatch", greedy)]);
                        cols[1].label(RichText::new("Schedule of the shown solution (model times)").strong());
                        egui::Grid::new("wh-schedule").striped(true).num_columns(6).show(&mut cols[1], |ui| {
                            for h in ["stop", "arrival s", "departure s", "load", "battery", "late s"] {
                                ui.label(RichText::new(h).strong());
                            }
                            ui.end_row();
                            for v in &self.lab.schedule {
                                ui.monospace(v["label"].as_str().unwrap_or(""));
                                ui.monospace(format!("{:.1}", model::num(&v["arrival"])));
                                ui.monospace(format!("{:.1}", model::num(&v["departure"])));
                                ui.monospace(format!("{}", v["load"]));
                                ui.monospace(format!("{:.0}%", 100.0 * model::num(&v["battery"])));
                                let late = model::num(&v["lateness"]);
                                ui.label(RichText::new(format!("{late:.1}")).monospace().color(if late > 0.0 {
                                    canvas::DANGER
                                } else {
                                    ui.visuals().text_color()
                                }));
                                ui.end_row();
                            }
                        });
                    });
                }
                MULTI => {
                    let greedy = &reference["greedy"]["metrics"];
                    ui.columns(2, |cols| {
                        widgets::metrics_grid(&mut cols[0], "mr-metrics", &[("best plan", &best), ("greedy", greedy)]);
                        if let Some(plan) = reference["greedy"]["plan"].as_array() {
                            let text: Vec<String> = plan
                                .iter()
                                .enumerate()
                                .map(|(i, l)| format!("R{}: {}", i + 1, l.as_array().map(|t| t.iter()
                                    .filter_map(Value::as_str).collect::<Vec<_>>().join(" ")).unwrap_or_default()))
                                .collect();
                            cols[0].label(RichText::new(format!("greedy plan  {}", text.join("  |  "))).small().weak());
                        }
                        cols[1].label(RichText::new("Shown plan, simulated robot by robot").strong());
                        egui::Grid::new("mr-robots").striped(true).num_columns(4).show(&mut cols[1], |ui| {
                            for h in ["robot", "tasks in order", "model s", "simulated s"] {
                                ui.label(RichText::new(h).strong());
                            }
                            ui.end_row();
                            for r in &self.lab.fleet_robots {
                                let k = r["robot"].as_u64().unwrap_or(0) as usize;
                                ui.label(RichText::new(format!("R{}", k + 1)).color(canvas::FLEET[k % canvas::FLEET.len()]));
                                let tasks: Vec<&str> = r["tasks"].as_array().map(|a| a.iter().filter_map(Value::as_str).collect())
                                    .unwrap_or_default();
                                ui.monospace(if tasks.is_empty() { "(idle)".to_string() } else { tasks.join(" ") });
                                ui.monospace(widgets::fmt(model::num(&self.lab.replay.metrics[format!("time.robot{k}")])));
                                ui.monospace(widgets::fmt(model::num(&r["simulatedTime"])));
                                ui.end_row();
                            }
                        });
                        cols[1].add_space(4.0);
                        if self.lab.conflicts.is_empty() {
                            cols[1].label("No robot-robot conflicts in the simulated execution.");
                        } else {
                            cols[1].label(RichText::new(format!(
                                "{} conflict(s): robots closer than 2 radii + 0.1 m. Robots do not avoid each other; conflicts are measured, not resolved.", self.lab.conflicts.len())).color(canvas::DANGER));
                            for c in &self.lab.conflicts {
                                cols[1].monospace(format!("R{} - R{}  {:.1}-{:.1} s at ({:.1}, {:.1})", c.robots.0 + 1,
                                    c.robots.1 + 1, c.start, c.end, c.x, c.y));
                            }
                        }
                    });
                }
                _ => {
                    let astar = reference["astar"]["metrics"].clone();
                    ui.columns(2, |cols| {
                        widgets::metrics_grid(&mut cols[0], "lab-metrics", &[("best (search)", &best), ("A* route", &astar)]);
                        let a = &reference["astar"];
                        let d = &reference["dijkstra"];
                        if a.is_object() {
                            cols[1].label(RichText::new("Reference planners (search effort is not comparable to fitness evaluations)").strong());
                            egui::Grid::new("lab-ref").striped(true).show(&mut cols[1], |ui| {
                                for h in ["planner", "expansions", "time us", "grid m", "smoothed m", "fitness"] {
                                    ui.label(RichText::new(h).strong());
                                }
                                ui.end_row();
                                for (name, p) in [("A*", a), ("Dijkstra", d)] {
                                    ui.label(name);
                                    ui.monospace(format!("{}", p["expansions"]));
                                    ui.monospace(format!("{}", p["planningMicros"]));
                                    ui.monospace(widgets::fmt(model::num(&p["gridLength"])));
                                    ui.monospace(widgets::fmt(model::num(&p["smoothLength"])));
                                    ui.monospace(widgets::fmt(model::num(&p["fitness"])));
                                    ui.end_row();
                                }
                            });
                        } else {
                            cols[1].label("Press 'A* / Dijkstra reference' to compute the conventional baseline.");
                        }
                    });
                }
            }
            if let Some(config) = &config {
                ui.collapsing("Experiment configuration sent to the engine (reproducible)", |ui| {
                    ui.monospace(serde_json::to_string_pretty(config).unwrap_or_default());
                });
            }
        });
    }

    pub fn lab_page(&mut self, ui: &mut egui::Ui) {
        egui::Panel::right("lab-config").default_size(350.0).resizable(true).show(ui, |ui| self.lab_config_panel(ui));
        egui::Panel::bottom("lab-bottom").default_size(280.0).resizable(true).show(ui, |ui| self.lab_bottom(ui));
        egui::CentralPanel::default().show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                self.lab.replay.controls(ui);
                ui.separator();
                if self.lab.problem_type == TUNING && !self.lab.episodes.is_empty() {
                    let mut chosen = self.lab.episode;
                    let label = |e: &Value| format!("{} ({}, {})", e["map"]["name"].as_str().unwrap_or("?"),
                        e["split"].as_str().unwrap_or(""), e["outcome"].as_str().unwrap_or(""));
                    egui::ComboBox::from_id_salt("lab-episode")
                        .selected_text(label(&self.lab.episodes[chosen.min(self.lab.episodes.len() - 1)]))
                        .show_ui(ui, |ui| {
                            for (i, e) in self.lab.episodes.iter().enumerate() {
                                ui.selectable_value(&mut chosen, i, label(e));
                            }
                        });
                    if chosen != self.lab.episode {
                        self.lab.episode = chosen;
                        self.lab.show_episode("episode");
                    }
                }
                if self.lab.problem_type == NAVIGATION {
                    ui.checkbox(&mut self.lab.show_reference, "A*");
                    ui.checkbox(&mut self.lab.show_candidates, "candidates");
                }
                if ui.small_button("fit").clicked() {
                    self.lab.view.reset();
                }
            });
            let run = self.run(&self.lab.run_id);
            let active = run.map(|r| r.status.active()).unwrap_or(false);
            let kind = self.lab.problem_type.clone();
            let mut live_markers: Vec<Marker> = Vec::new();
            let mut sequence: Option<Vec<[f64; 2]>> = None;
            let mut live_route: Option<Vec<[f64; 2]>> = None;
            if let Some(r) = run.filter(|_| active) {
                match kind.as_str() {
                    WAREHOUSE => {
                        live_markers = markers(&r.best_solution["stops"]);
                        let schedule = r.best_solution["schedule"].as_array().cloned().unwrap_or_default();
                        annotate_order(&mut live_markers, &schedule);
                        live_markers = merge_colocated(live_markers);
                        sequence = Some(schedule_points(&r.best_solution["schedule"])).filter(|s| s.len() > 1);
                    }
                    NAVIGATION => live_route = Some(model::points(&r.best_solution["route"])).filter(|p| p.len() > 1),
                    MULTI => live_markers = merge_colocated(markers(&r.best_solution["stops"])),
                    _ => {}
                }
            }
            let candidates = if self.lab.show_candidates && kind == NAVIGATION {
                self.lab.candidate_routes(run.and_then(|r| r.result.as_ref()))
            } else {
                vec![]
            };
            let reference: Option<Vec<[f64; 2]>> = self
                .lab
                .reference
                .as_ref()
                .filter(|_| self.lab.show_reference && kind == NAVIGATION)
                .map(|r| {
                    let mut v = vec![[self.lab.map.start.x, self.lab.map.start.y]];
                    v.extend(model::points(&r["astar"]["route"]));
                    v
                })
                .filter(|r| r.len() > 1);
            let caption = match run {
                Some(r) if r.status.active() => Some(format!(
                    "{}: evaluation {} / {}   best {}",
                    self.lab.algorithm,
                    r.evaluations,
                    r.max_evaluations,
                    widgets::fmt(r.best_fitness)
                )),
                _ => {
                    let map = self.lab.display_map.as_ref().unwrap_or(&self.lab.map);
                    let dynamic = if map.is_dynamic() || !map.zones.is_empty() {
                        format!(", {} moving, {} timed, {} zones", map.moving_obstacles.len(), map.timed_obstacles.len(),
                            map.zones.len())
                    } else {
                        String::new()
                    };
                    Some(format!("{}  ({} obstacles{dynamic})", map.name, map.obstacles.len()))
                }
            };
            let route = live_route.or_else(|| self.lab.replay.route.clone());
            let marker_list = if live_markers.is_empty() { self.lab.markers.clone() } else { live_markers };
            let dark = self.dark();
            let lab = &mut self.lab;
            let overlay = Overlay {
                markers: &marker_list,
                sequence: sequence.as_deref(),
                reference: reference.as_deref(),
                candidates: &candidates,
                route: route.as_deref(),
                trajectory: if active || kind == MULTI { None } else { lab.replay.trajectory.as_ref() },
                time: Some(lab.replay.time),
                show_rays: true,
                number_waypoints: kind == NAVIGATION,
                hide_start_goal: kind == WAREHOUSE || kind == MULTI,
                fleet: if active { &[] } else { &lab.fleet },
                conflicts: if active { &[] } else { &lab.conflicts },
                robot_radius: 0.25,
                sensor_range: 4.0,
                caption,
                ..Overlay::default()
            };
            let map = match lab.display_map.as_mut() {
                Some(m) => m,
                None => &mut lab.map,
            };
            canvas::show(ui, &mut lab.view, map, &overlay, None, dark);
        });
    }
}

pub fn log_view(ui: &mut egui::Ui, log: &[crate::app::LogLine]) {
    egui::ScrollArea::vertical().stick_to_bottom(true).show(ui, |ui| {
        for line in log.iter().rev().take(400).collect::<Vec<_>>().into_iter().rev() {
            let color = match line.level {
                crate::app::Level::Info => ui.visuals().text_color(),
                crate::app::Level::Warn => egui::Color32::from_rgb(230, 190, 90),
                crate::app::Level::Error => crate::canvas::DANGER,
                crate::app::Level::Engine => ui.visuals().weak_text_color(),
            };
            ui.label(RichText::new(format!("{:8.1}s  {}", line.secs, line.text)).monospace().small().color(color));
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nearest_point_uses_normalised_axes() {
        let pts = vec![[0.0, 0.0], [100.0, 1.0], [50.0, 0.5]];
        assert_eq!(nearest_point(&pts, 60.0, 0.1), Some(2));
        assert_eq!(nearest_point(&pts, 0.0, 0.3), Some(0));
        assert_eq!(nearest_point(&pts, 30.0, 0.9), Some(2), "y differences matter as much as x after normalising");
        assert_eq!(nearest_point(&[], 0.0, 0.0), None);
    }

    #[test]
    fn markers_are_labelled_with_their_visiting_order() {
        let mut m = markers(&json!([{"label": "P:T1", "kind": "pickup", "x": 0, "y": 0},
                                     {"label": "charger", "kind": "charger", "x": 1, "y": 1}]));
        let schedule = vec![json!({"label": "depot", "kind": "start"}), json!({"label": "P:T1", "kind": "pickup"}),
                            json!({"label": "charger", "kind": "charge"}), json!({"label": "charger", "kind": "charge"})];
        annotate_order(&mut m, &schedule);
        assert_eq!(m[0].label, "P:T1 #1");
        assert_eq!(m[1].label, "charger #2 #3");
    }

    #[test]
    fn colocated_markers_share_one_label() {
        let m = merge_colocated(markers(&json!([
            {"label": "D:T1 #3", "kind": "delivery", "x": 6.5, "y": 12.8},
            {"label": "D:T7 #5", "kind": "delivery", "x": 6.5, "y": 12.8},
            {"label": "P:T2 #1", "kind": "pickup", "x": 6.5, "y": 12.8}])));
        assert_eq!(m.len(), 2, "different kinds stay separate");
        assert_eq!(m[0].label, "D:T1 #3, D:T7 #5");
    }

    #[test]
    fn warehouse_stops_become_markers() {
        let stops = json!([{"label": "depot", "kind": "depot", "x": 1.0, "y": 2.0},
                           {"label": "P:T1", "kind": "pickup", "x": 3.0, "y": 4.0}]);
        let m = markers(&stops);
        assert_eq!(m.len(), 2);
        assert_eq!(m[1].kind, MarkerKind::Pickup);
        assert_eq!(m[1].label, "P:T1");
    }

    #[test]
    fn experiment_json_depends_on_the_problem_type() {
        let caps: Capabilities = serde_json::from_value(json!({
            "problems": [{"id": "warehouse-scheduling", "name": "S", "summary": "", "representation": "permutation",
                          "params": [{"name": "tasks", "type": "STRING", "defaultValue": "builtin:orders-8"}]}],
            "algorithms": [{"id": "genetic-algorithm", "name": "GA", "family": "e", "summary": "",
                            "representations": ["any"],
                            "params": [{"name": "mutation", "type": "CHOICE", "defaultValue": "default",
                                        "options": ["default", "gaussian", "swap"]}]}]
        }))
        .unwrap();
        let mut lab = LabState::new(WorldMap::empty("m", 10.0, 10.0), "t");
        lab.on_capabilities(&caps);
        lab.problem_type = WAREHOUSE.into();
        lab.algo_values.get_mut("genetic-algorithm").unwrap().insert("mutation".into(), json!("gaussian"));
        let e = lab.experiment(&caps);
        assert!(e["problem"].get("map").is_none(), "scheduling takes its map from the task set");
        assert_eq!(e["problem"]["params"]["tasks"], "builtin:orders-8");
        assert_eq!(e["algorithm"]["params"]["mutation"], "gaussian", "no operator list declared -> unchanged");
    }
}
