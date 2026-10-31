//! Robotic Arm: inverse kinematics, trajectory optimisation and pick-and-place planning for planar
//! arms. Choose a scene, click the workspace to move the target, run a search, and replay the motion
//! next to the analytical, numerical and configuration-space references computed by the engine.

use std::collections::HashMap;

use eframe::egui::{self, Align2, Color32, FontId, Pos2, Rect, RichText, Stroke, Vec2};
use serde_json::{json, Value};

use crate::app::{widgets, RoboApp};
use crate::canvas::{self, Overlay, View, Xf};
use crate::model::{self, Capabilities, Point, Pose, Shape, WorldMap};
use crate::params::{self, Values};
use crate::runs::{RunKind, RunStatus};

pub const IK: &str = "arm-ik";
pub const TRAJECTORY: &str = "arm-trajectory";
pub const PICK_PLACE: &str = "arm-pick-place";
pub const PROBLEMS: [&str; 3] = [IK, TRAJECTORY, PICK_PLACE];

const OBJECT: Color32 = Color32::from_rgb(90, 160, 250);

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ArmTab {
    Progress,
    Compare,
    Log,
}

/// One recorded configuration of a replay (engine `ArmMotion.frames`).
#[derive(Clone, Debug)]
pub struct ArmFrame {
    pub t: f64,
    pub points: Vec<[f64; 2]>,
    pub clearance: f64,
    pub segment: usize,
}

/// A motion being replayed.
#[derive(Clone, Debug, Default)]
pub struct Motion {
    pub title: String,
    pub frames: Vec<ArmFrame>,
    /// Object carried on the move into each keyframe (pick and place); empty otherwise.
    pub carrying: Vec<Option<String>>,
    pub plan: Vec<String>,
    pub metrics: Value,
}

impl Motion {
    pub fn from_frames(frames: &Value, title: impl Into<String>) -> Motion {
        let frames = frames
            .as_array()
            .map(|a| {
                a.iter()
                    .map(|f| ArmFrame {
                        t: model::num(&f["t"]),
                        points: model::points(&f["points"]),
                        clearance: model::num(&f["clearance"]),
                        segment: f["segment"].as_u64().unwrap_or(0) as usize,
                    })
                    .collect()
            })
            .unwrap_or_default();
        Motion { title: title.into(), frames, ..Motion::default() }
    }

    pub fn from_replay(replay: &Value, title: impl Into<String>) -> Motion {
        let mut m = Motion::from_frames(&replay["frames"], title);
        m.carrying = replay["carrying"]
            .as_array()
            .map(|a| a.iter().map(|c| c.as_str().map(String::from)).collect())
            .unwrap_or_default();
        m.plan = replay["plan"]
            .as_array()
            .map(|a| a.iter().filter_map(|p| p.as_str().map(String::from)).collect())
            .unwrap_or_default();
        m.metrics = replay["metrics"].clone();
        m
    }

    pub fn duration(&self) -> f64 {
        self.frames.last().map(|f| f.t).unwrap_or(0.0)
    }

    /// Index of the last frame at or before `t`.
    pub fn index_at(&self, t: f64) -> Option<usize> {
        if self.frames.is_empty() {
            return None;
        }
        Some(self.frames.partition_point(|f| f.t <= t).saturating_sub(1))
    }

    /// Where every task object is at frame `index`: at its pick point, in the gripper, or placed.
    pub fn objects(&self, tasks: &[Task], index: usize) -> Vec<(String, f64, f64, bool)> {
        let Some(frame) = self.frames.get(index) else { return vec![] };
        // Frames of segment s move towards keyframe s + 1 (or pause at keyframe s, where the gripper is
        // at the pick point anyway), so the object carried into s + 1 is in the gripper.
        let s = frame.segment;
        let carried = self.carrying.get(s + 1).cloned().flatten();
        let end = frame.points.last().copied().unwrap_or([f64::NAN, f64::NAN]);
        tasks
            .iter()
            .map(|t| {
                if carried.as_deref() == Some(t.label.as_str()) {
                    (t.label.clone(), end[0], end[1], true)
                } else if self.carrying.iter().take(s + 1).any(|c| c.as_deref() == Some(t.label.as_str())) {
                    (t.label.clone(), t.place[0], t.place[1], false)
                } else {
                    (t.label.clone(), t.pick[0], t.pick[1], false)
                }
            })
            .collect()
    }
}

/// A pick-and-place job of a scene.
#[derive(Clone, Debug)]
pub struct Task {
    pub label: String,
    pub pick: [f64; 2],
    pub place: [f64; 2],
}

pub fn tasks(scene: &Value) -> Vec<Task> {
    scene["tasks"]
        .as_array()
        .map(|a| {
            a.iter()
                .map(|t| Task {
                    label: t["label"].as_str().unwrap_or("?").to_string(),
                    pick: [model::num(&t["pick"]["x"]), model::num(&t["pick"]["y"])],
                    place: [model::num(&t["place"]["x"]), model::num(&t["place"]["y"])],
                })
                .collect()
        })
        .unwrap_or_default()
}

/// Joint positions for joint angles (relative angles, like the engine's `ArmKinematics.points`).
pub fn forward(scene: &Value, q: &[f64]) -> Vec<[f64; 2]> {
    let (mut x, mut y) = (model::num(&scene["base"]["x"]), model::num(&scene["base"]["y"]));
    let mut out = vec![[x, y]];
    let mut phi = 0.0;
    for (i, l) in scene["links"].as_array().cloned().unwrap_or_default().iter().enumerate() {
        phi += q.get(i).copied().unwrap_or(0.0);
        x += model::num(l) * phi.cos();
        y += model::num(l) * phi.sin();
        out.push([x, y]);
    }
    out
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array().map(|a| a.iter().map(model::num).collect()).unwrap_or_default()
}

/// The scene's workspace as a map, so the shared canvas draws floor, obstacles, zoom and pan.
pub fn workspace(scene: &Value) -> WorldMap {
    let mut map = WorldMap::empty(
        scene["name"].as_str().unwrap_or("arm"),
        model::num(&scene["width"]).max(0.5),
        model::num(&scene["height"]).max(0.5),
    );
    map.obstacles = serde_json::from_value::<Vec<Shape>>(scene["obstacles"].clone()).unwrap_or_default();
    map.start = Pose { x: model::num(&scene["base"]["x"]), y: model::num(&scene["base"]["y"]), theta: 0.0 };
    map.goal = Point { x: model::num(&scene["target"]["x"]), y: model::num(&scene["target"]["y"]) };
    map
}

pub fn default_scene(problem: &str) -> &'static str {
    match problem {
        TRAJECTORY => "arm-cluttered",
        PICK_PLACE => "arm-pick-place",
        _ => "arm-reach",
    }
}

pub struct ArmState {
    pub problem: String,
    pub scene_name: String,
    /// The scene being worked on (its target may have been moved by clicking).
    pub scene: Value,
    pub target_moved: bool,
    pub problem_values: HashMap<String, Values>,
    pub algorithm: String,
    pub algo_values: HashMap<String, Values>,
    pub budget: u64,
    pub seed: u64,
    pub threads: u64,
    pub run_id: Option<String>,
    pub result: Option<Value>,
    pub reference: Option<Value>,
    /// Id of an outstanding `plan` or `simulate` request sent from this page.
    pub pending: Option<String>,
    pub motion: Option<Motion>,
    pub time: f64,
    pub playing: bool,
    pub speed: f32,
    pub show_trace: bool,
    pub view: View,
    pub tab: ArmTab,
    pub series_choice: String,
    pub log_y: bool,
}

impl ArmState {
    pub fn new() -> ArmState {
        ArmState {
            problem: IK.into(),
            scene_name: "arm-reach".into(),
            scene: Value::Null,
            target_moved: false,
            problem_values: HashMap::new(),
            algorithm: "particle-swarm".into(),
            algo_values: HashMap::new(),
            budget: 3000,
            seed: 1,
            threads: 2,
            run_id: None,
            result: None,
            reference: None,
            pending: None,
            motion: None,
            time: 0.0,
            playing: false,
            speed: 1.0,
            show_trace: true,
            view: View::default(),
            tab: ArmTab::Progress,
            series_choice: String::new(),
            log_y: true,
        }
    }

    pub fn on_capabilities(&mut self, caps: &Capabilities) {
        for p in PROBLEMS {
            if let Some(info) = caps.problem(p) {
                let values = self.problem_values.entry(p.to_string()).or_default();
                for (k, v) in params::defaults(&info.params) {
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
        if self.scene.is_null() {
            let name = self.scene_name.clone();
            self.set_scene(caps, &name);
        }
    }

    pub fn set_scene(&mut self, caps: &Capabilities, name: &str) {
        if let Some(scene) = caps.arm_scenes.iter().find(|s| s["name"] == name) {
            self.scene = scene.clone();
            self.scene_name = name.to_string();
            self.target_moved = false;
            self.clear_view();
        }
    }

    pub fn set_problem(&mut self, caps: &Capabilities, kind: &str) {
        self.problem = kind.to_string();
        if kind == PICK_PLACE && tasks(&self.scene).is_empty() || kind != PICK_PLACE && self.scene_name == "arm-pick-place" {
            self.set_scene(caps, default_scene(kind));
        }
        self.clear_view();
        let supported = caps.algorithms_for(kind);
        if !supported.iter().any(|a| a.id == self.algorithm) {
            self.algorithm = supported.first().map(|a| a.id.clone()).unwrap_or_default();
        }
    }

    pub fn clear_view(&mut self) {
        self.result = None;
        self.reference = None;
        self.motion = None;
        self.time = 0.0;
        self.playing = false;
        self.view.reset();
    }

    /// Moves the target (inverse kinematics and trajectory problems); the scene is then sent inline.
    pub fn set_target(&mut self, x: f64, y: f64) {
        self.scene["target"] = json!({"x": x, "y": y});
        self.target_moved = true;
        self.reference = None;
        self.motion = None;
    }

    pub fn problem_json(&self, caps: &Capabilities) -> Value {
        let specs = caps.problem(&self.problem).map(|p| p.params.clone()).unwrap_or_default();
        let empty = Values::new();
        let values = self.problem_values.get(&self.problem).unwrap_or(&empty);
        let scene = if self.target_moved { self.scene.clone() } else { json!(self.scene_name) };
        json!({"type": self.problem, "scene": scene, "params": params::to_json(&specs, values)})
    }

    pub fn experiment(&self, caps: &Capabilities) -> Value {
        let algo_specs = caps.algorithm(&self.algorithm).map(|a| a.params.clone()).unwrap_or_default();
        let operators = caps.problem(&self.problem).map(|p| p.operators.clone());
        let empty = Values::new();
        json!({
            "schema": "robosbse.experiment/1",
            "name": format!("{}-{}", self.problem, self.scene_name),
            "seed": self.seed,
            "budget": {"maxEvaluations": self.budget},
            "problem": self.problem_json(caps),
            "algorithm": {
                "type": self.algorithm,
                "params": params::to_json_for(&algo_specs, self.algo_values.get(&self.algorithm).unwrap_or(&empty),
                    operators.as_ref()),
            },
            "options": {"archiveSize": 5, "threads": self.threads}
        })
    }

    pub fn show(&mut self, motion: Motion) {
        self.motion = Some(motion);
        self.time = 0.0;
        self.playing = true;
    }

    pub fn on_finished(&mut self, result: &Value) {
        if result["reference"].is_object() {
            self.reference = Some(result["reference"].clone());
        }
        self.show(Motion::from_replay(&result["replay"], "best solution"));
        self.result = Some(result.clone());
        self.tab = ArmTab::Compare;
    }

    pub fn tick(&mut self, dt: f64) -> bool {
        let Some(m) = &self.motion else { return false };
        if !self.playing {
            return false;
        }
        self.time += dt * self.speed as f64;
        if self.time >= m.duration() {
            self.time = m.duration();
            self.playing = false;
        }
        true
    }

    /// Reference rows: (name, fitness, metrics, how to show it).
    pub fn reference_rows(&self) -> Vec<(String, f64, Value, RefAction)> {
        let Some(r) = &self.reference else { return vec![] };
        let mut rows = Vec::new();
        let mut add = |entry: &Value, name: String| {
            let action = if entry["frames"].is_array() {
                RefAction::Frames(entry["frames"].clone())
            } else if entry["genome"].is_array() {
                RefAction::Simulate(entry["genome"].clone())
            } else if entry["q"].is_array() {
                RefAction::Simulate(entry["q"].clone())
            } else {
                RefAction::None
            };
            rows.push((name, model::num(&entry["fitness"]), entry["metrics"].clone(), action));
        };
        match self.problem.as_str() {
            IK => {
                for e in r["analytical"].as_array().cloned().unwrap_or_default() {
                    add(&e, format!("analytical IK, {}", e["name"].as_str().unwrap_or("")));
                }
                for e in r["numerical"].as_array().cloned().unwrap_or_default() {
                    add(&e, format!("numerical IK, {} ({} iterations)", e["name"].as_str().unwrap_or(""), e["iterations"]));
                }
            }
            TRAJECTORY => {
                for e in r["straight"].as_array().cloned().unwrap_or_default() {
                    add(&e, e["name"].as_str().unwrap_or("").to_string());
                }
            }
            _ => {
                for (key, label) in [("greedy", "greedy plan"), ("optimum", "exhaustive optimum")] {
                    if r[key].is_object() {
                        let e = r[key].clone();
                        let extra = if key == "optimum" { format!(" ({} plans enumerated)", e["plansEnumerated"]) } else { String::new() };
                        add(&e, format!("{label}{extra}"));
                    }
                }
            }
        }
        rows
    }
}

pub enum RefAction {
    Frames(Value),
    Simulate(Value),
    None,
}

fn key_metrics(problem: &str) -> &'static [&'static str] {
    match problem {
        IK => &["distance", "clearance", "limitViolation", "reached"],
        TRAJECTORY => &["reach", "time", "effort", "roughness", "minClearance", "penetration", "limitViolation", "feasible"],
        _ => &["time", "effort", "minClearance", "unplannedMoves", "feasible"],
    }
}

/// Draws the arm scene on top of the canvas: reach, targets or jobs, objects, the trace of the end
/// effector, and the arm itself (red where it penetrates an obstacle).
fn draw_arm(painter: &egui::Painter, xf: &Xf, state: &ArmState, live: Option<&[Vec<[f64; 2]>]>, dark: bool) {
    let scene = &state.scene;
    let text = if dark { Color32::from_rgb(210, 216, 226) } else { Color32::from_rgb(40, 46, 56) };
    let base = [model::num(&scene["base"]["x"]), model::num(&scene["base"]["y"])];
    let reach: f64 = floats(&scene["links"]).iter().sum();
    let centre = xf.to_screen(base[0], base[1]);
    let ring: Vec<Pos2> = (0..=96)
        .map(|k| {
            let a = std::f64::consts::TAU * k as f64 / 96.0;
            xf.to_screen(base[0] + reach * a.cos(), base[1] + reach * a.sin())
        })
        .collect();
    painter.extend(egui::Shape::dashed_line(&ring, Stroke::new(1.0, text.gamma_multiply(0.25)), 5.0, 5.0));
    let task_list = tasks(scene);
    if state.problem == PICK_PLACE {
        for t in &task_list {
            let p = xf.to_screen(t.pick[0], t.pick[1]);
            painter.rect_stroke(Rect::from_center_size(p, Vec2::splat(14.0)), 2.0, Stroke::new(1.0, OBJECT.gamma_multiply(0.6)),
                egui::StrokeKind::Middle);
            painter.text(p + Vec2::new(-9.0, 0.0), Align2::RIGHT_CENTER, format!("{} pick", t.label), FontId::proportional(10.0),
                OBJECT.gamma_multiply(0.8));
            let q = xf.to_screen(t.place[0], t.place[1]);
            painter.add(egui::Shape::convex_polygon(
                vec![q + Vec2::new(0.0, -8.0), q + Vec2::new(8.0, 6.0), q + Vec2::new(-8.0, 6.0)],
                Color32::TRANSPARENT,
                Stroke::new(1.0, canvas::GOAL.gamma_multiply(0.8)),
            ));
            painter.text(q + Vec2::new(10.0, 0.0), Align2::LEFT_CENTER, format!("{} place", t.label), FontId::proportional(10.0),
                canvas::GOAL.gamma_multiply(0.9));
        }
    } else {
        let g = xf.to_screen(model::num(&scene["target"]["x"]), model::num(&scene["target"]["y"]));
        painter.circle_stroke(g, 9.0, Stroke::new(2.0, canvas::GOAL));
        painter.line_segment([g - Vec2::new(13.0, 0.0), g + Vec2::new(13.0, 0.0)], Stroke::new(1.5, canvas::GOAL));
        painter.line_segment([g - Vec2::new(0.0, 13.0), g + Vec2::new(0.0, 13.0)], Stroke::new(1.5, canvas::GOAL));
    }
    // Ghost poses of a running search (best so far).
    for pose in live.unwrap_or(&[]) {
        let pts: Vec<Pos2> = pose.iter().map(|p| xf.to_screen(p[0], p[1])).collect();
        painter.line(pts, Stroke::new(xf.len(0.05).max(2.0), canvas::ROUTE.gamma_multiply(0.45)));
    }
    let width = xf.len(2.0 * model::num(&scene["linkRadius"]).max(0.01)).max(3.0);
    let draw_pose = |points: &[[f64; 2]], colliding: bool, alpha: f32| {
        let color = if colliding { canvas::DANGER } else { text };
        let pts: Vec<Pos2> = points.iter().map(|p| xf.to_screen(p[0], p[1])).collect();
        for w in pts.windows(2) {
            painter.line_segment([w[0], w[1]], Stroke::new(width, color.gamma_multiply(0.55 * alpha)));
            painter.line_segment([w[0], w[1]], Stroke::new(1.5, color.gamma_multiply(alpha)));
        }
        for (i, p) in pts.iter().enumerate() {
            let r = if i == 0 { width * 0.8 } else { width * 0.55 };
            painter.circle(*p, r.max(3.0), Color32::TRANSPARENT, Stroke::new(1.5, color.gamma_multiply(alpha)));
        }
        if let Some(end) = pts.last() {
            painter.circle_filled(*end, 3.5, canvas::START.gamma_multiply(alpha));
        }
    };
    let motion = state.motion.as_ref();
    let index = motion.and_then(|m| m.index_at(state.time));
    match (motion, index) {
        (Some(m), Some(i)) => {
            if state.show_trace {
                let trace: Vec<Pos2> = m.frames[..=i]
                    .iter()
                    .filter_map(|f| f.points.last())
                    .map(|p| xf.to_screen(p[0], p[1]))
                    .collect();
                if trace.len() > 1 {
                    painter.line(trace, Stroke::new(2.0, canvas::TRAIL));
                }
                for f in m.frames[..=i].iter().filter(|f| f.clearance < 0.0) {
                    if let Some(p) = f.points.last() {
                        painter.circle_filled(xf.to_screen(p[0], p[1]), 2.5, canvas::DANGER);
                    }
                }
            }
            for (label, x, y, carried) in m.objects(&task_list, i) {
                let p = xf.to_screen(x, y);
                painter.rect_filled(Rect::from_center_size(p, Vec2::splat(if carried { 11.0 } else { 10.0 })), 2.0, OBJECT);
                painter.text(p, Align2::CENTER_CENTER, &label, FontId::proportional(9.0), Color32::WHITE);
            }
            if state.problem == IK {
                draw_pose(&forward(scene, &floats(&scene["home"])), false, 0.25);
            }
            let f = &m.frames[i];
            draw_pose(&f.points, f.clearance < 0.0, 1.0);
        }
        _ => {
            for t in &task_list {
                let p = xf.to_screen(t.pick[0], t.pick[1]);
                if state.problem == PICK_PLACE {
                    painter.rect_filled(Rect::from_center_size(p, Vec2::splat(10.0)), 2.0, OBJECT);
                    painter.text(p, Align2::CENTER_CENTER, &t.label, FontId::proportional(9.0), Color32::WHITE);
                }
            }
            draw_pose(&forward(scene, &floats(&scene["home"])), false, 0.8);
        }
    }
    painter.rect_filled(Rect::from_center_size(centre + Vec2::new(0.0, 6.0), Vec2::new(26.0, 10.0)), 2.0,
        text.gamma_multiply(0.6));
}

impl RoboApp {
    pub fn start_arm_run(&mut self) {
        let Some(caps) = self.caps.clone() else { return };
        self.arm.ensure_algorithm(&caps);
        let experiment = self.arm.experiment(&caps);
        if let Some(id) = self.start_run(RunKind::Arm, experiment) {
            self.arm.run_id = Some(id);
            self.arm.result = None;
            self.arm.motion = None;
            self.arm.tab = ArmTab::Progress;
        }
    }

    pub fn request_arm_reference(&mut self) {
        let Some(caps) = self.caps.clone() else { return };
        let problem = self.arm.problem_json(&caps);
        self.arm.pending = self.request("plan", json!({"problem": problem}));
    }

    fn simulate_arm(&mut self, genome: &Value) {
        let Some(caps) = self.caps.clone() else { return };
        let problem = self
            .run(&self.arm.run_id)
            .filter(|_| self.arm.result.is_some())
            .map(|r| r.config["problem"].clone())
            .filter(|p| p.is_object())
            .unwrap_or_else(|| self.arm.problem_json(&caps));
        self.arm.pending = self.request("simulate", json!({"problem": problem, "genome": genome}));
    }

    fn arm_config(&mut self, ui: &mut egui::Ui) {
        let Some(caps) = self.caps.clone() else {
            ui.label("Waiting for the engine...");
            return;
        };
        let running = self.run(&self.arm.run_id).map(|r| r.status.active()).unwrap_or(false);
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("Robotic arm");
            egui::Grid::new("arm-select").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
                ui.label("Problem");
                let current = caps.problem(&self.arm.problem).map(|p| p.name.clone()).unwrap_or_default();
                let mut chosen: Option<String> = None;
                egui::ComboBox::from_id_salt("arm-problem").selected_text(current).width(220.0).show_ui(ui, |ui| {
                    for p in PROBLEMS {
                        if let Some(info) = caps.problem(p) {
                            if ui.selectable_label(self.arm.problem == p, &info.name).clicked() {
                                chosen = Some(p.to_string());
                            }
                        }
                    }
                });
                if let Some(kind) = chosen {
                    self.arm.set_problem(&caps, &kind);
                }
                ui.end_row();
                ui.label("Scene");
                let mut chosen: Option<String> = None;
                egui::ComboBox::from_id_salt("arm-scene").selected_text(&self.arm.scene_name).width(220.0).show_ui(ui, |ui| {
                    for s in &caps.arm_scenes {
                        let name = s["name"].as_str().unwrap_or("?");
                        let usable = self.arm.problem != PICK_PLACE || !tasks(s).is_empty();
                        if ui.add_enabled(usable, egui::Button::selectable(self.arm.scene_name == name, name)).clicked() {
                            chosen = Some(name.to_string());
                        }
                    }
                });
                if let Some(name) = chosen {
                    self.arm.set_scene(&caps, &name);
                }
                ui.end_row();
            });
            if let Some(p) = caps.problem(&self.arm.problem) {
                ui.add(egui::Label::new(RichText::new(&p.summary).small().weak()).wrap());
            }
            ui.add(egui::Label::new(RichText::new(self.arm.scene["description"].as_str().unwrap_or("")).small()).wrap());
            let links = floats(&self.arm.scene["links"]);
            ui.label(RichText::new(format!(
                "{} links ({} m), joint speed {} rad/s, link radius {} m",
                links.len(),
                links.iter().map(|l| format!("{l}")).collect::<Vec<_>>().join(" + "),
                self.arm.scene["maxJointSpeed"],
                self.arm.scene["linkRadius"]
            )).small().weak());
            if self.arm.problem != PICK_PLACE {
                ui.horizontal(|ui| {
                    let t = &self.arm.scene["target"];
                    ui.label(format!("Target ({:.2}, {:.2})", model::num(&t["x"]), model::num(&t["y"])));
                    if self.arm.target_moved && ui.small_button("reset").clicked() {
                        let name = self.arm.scene_name.clone();
                        self.arm.set_scene(&caps, &name);
                    }
                });
                ui.label(RichText::new("Click the workspace to move the target.").small().weak());
            }
            ui.add_space(6.0);
            ui.label(RichText::new("Fitness").strong());
            if let Some(p) = caps.problem(&self.arm.problem) {
                let values = self.arm.problem_values.entry(self.arm.problem.clone()).or_default();
                params::form(ui, "arm-problem", &p.params, values, &[]);
            }
            ui.add_space(6.0);
            ui.separator();
            ui.label(RichText::new("Search").strong());
            let supported: Vec<_> = caps.algorithms_for(&self.arm.problem).into_iter().cloned().collect();
            let current = caps.algorithm(&self.arm.algorithm).map(|a| a.name.clone()).unwrap_or_default();
            egui::ComboBox::from_id_salt("arm-alg").selected_text(current).width(230.0).show_ui(ui, |ui| {
                for a in &supported {
                    ui.selectable_value(&mut self.arm.algorithm, a.id.clone(), format!("{} ({})", a.name, a.family));
                }
            });
            if let Some(a) = caps.algorithm(&self.arm.algorithm) {
                let operators = caps.problem(&self.arm.problem).map(|p| p.operators.clone());
                let values = self.arm.algo_values.entry(a.id.clone()).or_default();
                params::form_for(ui, "arm-alg-params", &a.params, values, &[], operators.as_ref());
            }
            egui::Grid::new("arm-budget").num_columns(2).show(ui, |ui| {
                ui.label("Fitness evaluations");
                ui.add(egui::DragValue::new(&mut self.arm.budget).range(1..=10_000_000).speed(50));
                ui.end_row();
                ui.label("Random seed");
                ui.add(egui::DragValue::new(&mut self.arm.seed));
                ui.end_row();
                ui.label("Evaluation threads");
                ui.add(egui::DragValue::new(&mut self.arm.threads).range(1..=64));
                ui.end_row();
            });
            ui.add_space(8.0);
            ui.horizontal_wrapped(|ui| {
                if ui.add_enabled(!running, egui::Button::new(RichText::new("Start").strong())).clicked() {
                    self.start_arm_run();
                }
                let id = self.arm.run_id.clone();
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
                let label = match self.arm.problem.as_str() {
                    IK => "Analytical / numerical IK",
                    TRAJECTORY => "Straight move / C-space planner",
                    _ => "Greedy / exhaustive plan",
                };
                if ui.button(label).on_hover_text("Conventional references, evaluated with the same fitness").clicked() {
                    self.request_arm_reference();
                    self.arm.tab = ArmTab::Compare;
                }
            });
        });
    }

    fn arm_bottom(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.selectable_value(&mut self.arm.tab, ArmTab::Progress, "Progress");
            ui.selectable_value(&mut self.arm.tab, ArmTab::Compare, "Solution and references");
            ui.selectable_value(&mut self.arm.tab, ArmTab::Log, "Log");
        });
        ui.separator();
        match self.arm.tab {
            ArmTab::Progress => match self.runs.get(self.arm.run_id.as_deref().unwrap_or("")) {
                Some(run) => {
                    let target = match self.arm.problem.as_str() {
                        IK => "configurations reaching the target",
                        TRAJECTORY => "collision-free motions reaching the target",
                        _ => "collision-free plans",
                    };
                    widgets::run_status(ui, run, target);
                    ui.checkbox(&mut self.arm.log_y, "log scale");
                    let h = ui.available_height().max(80.0);
                    let series = run.result.as_ref().map(|r| r["series"].clone()).unwrap_or(Value::Null);
                    let (log_y, choice) = (self.arm.log_y, &mut self.arm.series_choice);
                    ui.columns(2, |cols| {
                        widgets::convergence_plot(&mut cols[0], "arm-conv", run, h - 8.0, log_y);
                        widgets::series_plot(&mut cols[1], "arm-series", &series, h - 8.0, choice);
                    });
                }
                None => {
                    ui.label("Choose a problem and a scene on the right and press Start.");
                }
            },
            ArmTab::Compare => self.arm_compare(ui),
            ArmTab::Log => crate::pages::lab::log_view(ui, &self.log),
        }
    }

    fn arm_compare(&mut self, ui: &mut egui::Ui) {
        let names = key_metrics(&self.arm.problem);
        let result = self.arm.result.clone();
        let rows = self.arm.reference_rows();
        let mut action: Option<(String, RefAction)> = None;
        egui::ScrollArea::vertical().id_salt("arm-compare").show(ui, |ui| {
            if let Some(r) = &result {
                ui.label(format!(
                    "{} evaluations in {} ms, best found at evaluation {}",
                    r["evaluations"], r["elapsedMs"], r["best"]["evaluation"]
                ));
            }
            egui::Grid::new("arm-rows").striped(true).num_columns(names.len() + 3).show(ui, |ui| {
                ui.label(RichText::new("solution").strong());
                ui.label(RichText::new("fitness").strong());
                for n in names {
                    ui.label(RichText::new(*n).strong());
                }
                ui.label("");
                ui.end_row();
                if let Some(r) = &result {
                    ui.label(RichText::new(format!("search: {}", r["experiment"]["algorithm"]["type"].as_str().unwrap_or(""))).strong());
                    ui.monospace(widgets::fmt(model::num(&r["best"]["fitness"])));
                    for n in names {
                        ui.monospace(widgets::fmt(model::num(&r["best"]["metrics"][*n])));
                    }
                    if ui.small_button("show").clicked() {
                        action = Some(("best solution".into(), RefAction::Frames(r["replay"].clone())));
                    }
                    ui.end_row();
                }
                for (name, fitness, metrics, how) in &rows {
                    ui.label(name);
                    ui.monospace(widgets::fmt(*fitness));
                    for n in names {
                        ui.monospace(widgets::fmt(model::num(&metrics[*n])));
                    }
                    let enabled = !matches!(how, RefAction::None);
                    if ui.add_enabled(enabled, egui::Button::new("show").small()).clicked() {
                        action = Some((name.clone(), match how {
                            RefAction::Frames(f) => RefAction::Frames(json!({"frames": f})),
                            RefAction::Simulate(g) => RefAction::Simulate(g.clone()),
                            RefAction::None => RefAction::None,
                        }));
                    }
                    ui.end_row();
                }
            });
            if self.arm.problem == IK {
                ui.label(RichText::new("Inverse kinematics scores the final configuration only; 'show' displays that pose                     (no motion is planned).").small().weak());
            }
            if rows.is_empty() {
                ui.label(RichText::new("Press the reference button on the right to compute the conventional baselines \
                    (they are also part of every finished run).").weak());
            }
            if let Some(reference) = &self.arm.reference {
                if let Some(note) = reference["note"].as_str() {
                    ui.label(RichText::new(note).small().weak());
                }
                let expansions = model::num(&reference["plannerExpansions"]);
                if expansions > 0.0 {
                    ui.label(RichText::new(format!("configuration-space planner: {expansions} node expansions (not fitness evaluations)"))
                        .small().weak());
                }
            }
            if let Some(m) = &self.arm.motion {
                if !m.plan.is_empty() {
                    ui.label(RichText::new(format!("{}: {}", m.title, m.plan.join("  ->  "))).monospace().small());
                }
            }
        });
        match action {
            Some((title, RefAction::Frames(replay))) => {
                let motion = Motion::from_replay(&replay, title);
                self.arm.show(motion);
            }
            Some((_, RefAction::Simulate(genome))) => self.simulate_arm(&genome),
            _ => {}
        }
    }

    pub fn arm_page(&mut self, ui: &mut egui::Ui) {
        egui::Panel::right("arm-config").default_size(360.0).resizable(true).show(ui, |ui| self.arm_config(ui));
        egui::Panel::bottom("arm-bottom").default_size(270.0).resizable(true).show(ui, |ui| self.arm_bottom(ui));
        egui::CentralPanel::default().show(ui, |ui| {
            let duration = self.arm.motion.as_ref().map(|m| m.duration()).unwrap_or(0.0);
            ui.horizontal_wrapped(|ui| {
                let a = &mut self.arm;
                ui.add_enabled_ui(a.motion.is_some(), |ui| {
                    if ui.button(if a.playing { "Pause" } else { "Play" }).clicked() {
                        if !a.playing && a.time >= duration {
                            a.time = 0.0;
                        }
                        a.playing = !a.playing;
                    }
                    if ui.button("Restart").clicked() {
                        a.time = 0.0;
                        a.playing = true;
                    }
                    egui::ComboBox::from_id_salt("arm-speed").selected_text(format!("{}x", a.speed)).width(56.0).show_ui(ui, |ui| {
                        for s in [0.25f32, 0.5, 1.0, 2.0, 4.0] {
                            ui.selectable_value(&mut a.speed, s, format!("{s}x"));
                        }
                    });
                    let mut t = a.time;
                    if ui.add(egui::Slider::new(&mut t, 0.0..=duration.max(0.01)).show_value(false)).changed() {
                        a.time = t;
                        a.playing = false;
                    }
                    ui.monospace(format!("{:5.2} / {:.2} s", a.time, duration));
                });
                ui.checkbox(&mut a.show_trace, "trace");
                if let Some(m) = &a.motion {
                    ui.separator();
                    ui.label(RichText::new(&m.title).strong());
                    if let Some(f) = m.index_at(a.time).map(|i| &m.frames[i]) {
                        let color = if f.clearance < 0.0 { canvas::DANGER } else { ui.visuals().text_color() };
                        ui.label(RichText::new(format!("clearance {:.3} m", f.clearance)).monospace().color(color));
                    }
                }
                if ui.small_button("fit").clicked() {
                    a.view.reset();
                }
            });
            let dt = ui.input(|i| i.stable_dt).min(0.1) as f64;
            if self.arm.tick(dt) {
                ui.ctx().request_repaint();
            }
            let run = self.run(&self.arm.run_id).filter(|r| r.status.active());
            let live: Vec<Vec<[f64; 2]>> = run
                .map(|r| {
                    let s = &r.best_solution;
                    if s["points"].is_array() {
                        vec![model::points(&s["points"])]
                    } else if let Some(keys) = s["keyframes"].as_array() {
                        keys.iter().map(|q| forward(&self.arm.scene, &floats(q))).collect()
                    } else {
                        vec![]
                    }
                })
                .unwrap_or_default();
            let caption = match run {
                Some(r) => format!("{}: evaluation {} / {}   best {}", self.arm.algorithm, r.evaluations, r.max_evaluations,
                    widgets::fmt(r.best_fitness)),
                None => {
                    let name = self.caps.as_ref().and_then(|c| c.problem(&self.arm.problem)).map(|p| p.name.clone());
                    format!("{}  |  {}", self.arm.scene_name, name.unwrap_or_else(|| self.arm.problem.clone()))
                }
            };
            let running = run.is_some();
            let dark = self.dark();
            if self.arm.scene.is_null() {
                ui.label("Waiting for the engine's arm scenes...");
                return;
            }
            let mut map = workspace(&self.arm.scene);
            let overlay = Overlay { hide_start_goal: true, caption: Some(caption), ..Overlay::default() };
            let out = canvas::show(ui, &mut self.arm.view, &mut map, &overlay, None, dark);
            let painter = ui.painter_at(out.rect);
            draw_arm(&painter, &out.xf, &self.arm, (!live.is_empty()).then_some(&live[..]), dark);
            if let Some((x, y)) = out.clicked {
                let inside = x >= 0.0 && y >= 0.0 && x <= map.width && y <= map.height;
                if inside && self.arm.problem != PICK_PLACE && !running {
                    self.arm.set_target((x * 100.0).round() / 100.0, (y * 100.0).round() / 100.0);
                }
            }
        });
    }
}

impl ArmState {
    pub fn ensure_algorithm(&mut self, caps: &Capabilities) {
        let supported = caps.algorithms_for(&self.problem);
        if !supported.iter().any(|a| a.id == self.algorithm) {
            self.algorithm = supported.first().map(|a| a.id.clone()).unwrap_or_default();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scene() -> Value {
        json!({"name": "s", "width": 3.0, "height": 2.0, "base": {"x": 1.5, "y": 0.1}, "links": [0.9, 0.7],
               "linkRadius": 0.04, "home": [1.5708, 1.5], "target": {"x": 2.2, "y": 0.8},
               "obstacles": [{"type": "rect", "x": 0.3, "y": 1.3, "w": 0.55, "h": 0.08}],
               "tasks": [{"label": "A", "pick": {"x": 0.5, "y": 0.7}, "place": {"x": 2.4, "y": 0.7}}]})
    }

    #[test]
    fn forward_kinematics_matches_the_engine_convention() {
        let p = forward(&scene(), &[std::f64::consts::FRAC_PI_2, -std::f64::consts::FRAC_PI_2]);
        assert!((p[1][0] - 1.5).abs() < 1e-9 && (p[1][1] - 1.0).abs() < 1e-9);
        assert!((p[2][0] - 2.2).abs() < 1e-9 && (p[2][1] - 1.0).abs() < 1e-9, "relative joint angles: {:?}", p[2]);
        let map = workspace(&scene());
        assert_eq!(map.obstacles.len(), 1);
        assert!((map.goal.x - 2.2).abs() < 1e-12);
    }

    #[test]
    fn objects_follow_the_gripper_while_carried() {
        let frame = |t: f64, segment: usize, end: [f64; 2]| json!({"t": t, "segment": segment, "clearance": 0.1,
            "points": [[1.5, 0.1], [1.0, 0.5], end]});
        // keyframes: home, pick, place; segment 0 = home -> pick, segment 1 = pick -> place (carrying A).
        let replay = json!({
            "frames": [frame(0.0, 0, [1.5, 1.6]), frame(0.5, 0, [1.0, 1.0]), frame(1.0, 0, [0.5, 0.7]),
                       frame(2.0, 1, [0.5, 0.7]), frame(2.5, 1, [1.5, 0.9]), frame(3.0, 1, [2.4, 0.7]),
                       frame(4.0, 2, [2.4, 0.7])],
            "carrying": [null, null, "A"], "plan": ["A (pick elbow +, place elbow -)"], "metrics": {}
        });
        let m = Motion::from_replay(&replay, "t");
        let t = tasks(&scene());
        assert_eq!(m.index_at(2.7), Some(4));
        let at = |i: usize| m.objects(&t, i)[0].clone();
        assert_eq!((at(1).1, at(1).2, at(1).3), (0.5, 0.7, false), "waiting at the pick point");
        assert_eq!((at(4).1, at(4).2, at(4).3), (1.5, 0.9, true), "in the gripper");
        assert_eq!((at(6).1, at(6).2, at(6).3), (2.4, 0.7, false), "placed");
        assert_eq!(m.plan.len(), 1);
    }
}
