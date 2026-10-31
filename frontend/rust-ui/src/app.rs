//! Application state, engine message dispatch and the overall window layout.

use std::collections::HashMap;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use eframe::egui::{self, Color32, RichText};
use serde_json::{json, Value};

use crate::capture::{self, CliArgs};
use crate::engine::{Engine, EngineEvent, EngineStatus};
use crate::model::{self, Capabilities, WorldMap};
use crate::pages::{benchmark::BenchState, editor::EditorPage, history::HistoryState, lab::LabState, testing::TestingState};
use crate::runs::{RunKind, RunState, RunStatus};
use crate::settings::{self, Settings};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Page {
    Dashboard,
    Lab,
    Editor,
    Algorithms,
    Testing,
    GeneticProgramming,
    ProgramRepair,
    Arm,
    Benchmark,
    History,
    Settings,
}

impl Page {
    pub const ALL: [Page; 11] = [
        Page::Dashboard,
        Page::Lab,
        Page::Editor,
        Page::Algorithms,
        Page::Testing,
        Page::GeneticProgramming,
        Page::ProgramRepair,
        Page::Arm,
        Page::Benchmark,
        Page::History,
        Page::Settings,
    ];

    pub fn title(self) -> &'static str {
        match self {
            Page::Dashboard => "Dashboard",
            Page::Lab => "Simulation Laboratory",
            Page::Editor => "Environment Editor",
            Page::Algorithms => "Algorithms",
            Page::Testing => "Search-Based Testing",
            Page::GeneticProgramming => "Genetic Programming",
            Page::ProgramRepair => "Program Repair",
            Page::Arm => "Robotic Arm",
            Page::Benchmark => "Benchmark Suite",
            Page::History => "Experiment History",
            Page::Settings => "Settings",
        }
    }

    pub fn from_name(name: &str) -> Option<Page> {
        Some(match name {
            "dashboard" => Page::Dashboard,
            "lab" => Page::Lab,
            "editor" => Page::Editor,
            "algorithms" => Page::Algorithms,
            "testing" => Page::Testing,
            "gp" => Page::GeneticProgramming,
            "repair" => Page::ProgramRepair,
            "arm" => Page::Arm,
            "benchmark" => Page::Benchmark,
            "history" => Page::History,
            "settings" => Page::Settings,
            _ => return None,
        })
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Level {
    Info,
    Warn,
    Error,
    Engine,
}

pub struct LogLine {
    /// Seconds since the UI started.
    pub secs: f64,
    pub level: Level,
    pub text: String,
}

/// Where a requested map should go when it arrives.
#[derive(Clone, Copy)]
pub enum MapTarget {
    /// The editable Laboratory map (route optimisation).
    Lab,
    /// The map shown for other Laboratory problems (e.g. the warehouse of a task set).
    LabDisplay,
    Arena,
}

pub struct RoboApp {
    pub settings: Settings,
    pub engine: Engine,
    pub caps: Option<Capabilities>,
    pub page: Page,
    pub log: Vec<LogLine>,
    pub engine_stderr: Vec<String>,
    pub runs: HashMap<String, RunState>,
    pub lab: LabState,
    pub editor: EditorPage,
    pub testing: TestingState,
    pub gp: crate::pages::gp::GpState,
    pub repair: crate::pages::repair::RepairState,
    pub arm: crate::pages::arm::ArmState,
    pub bench: BenchState,
    pub history: HistoryState,
    pub pending_maps: HashMap<String, MapTarget>,
    pub capture: Option<capture::Script>,
    pub toast: Option<(String, Instant)>,
    started: Instant,
}

impl RoboApp {
    pub fn new(cc: &eframe::CreationContext<'_>, args: CliArgs) -> Self {
        let settings = Settings::load();
        apply_style(&cc.egui_ctx, &settings);
        let engine = spawn_engine(&settings, &cc.egui_ctx);
        let maps_dir = PathBuf::from(&settings.maps_dir);
        let lab_map = load_map_file(&maps_dir.join("warehouse.json")).unwrap_or_else(|| WorldMap::empty("untitled", 20.0, 20.0));
        let arena = load_map_file(&maps_dir.join("testing-arena.json")).unwrap_or_else(|| WorldMap::empty("arena", 12.0, 12.0));
        let page = args.page.as_deref().and_then(Page::from_name).unwrap_or(Page::Dashboard);
        let capture = (args.capture.is_some() || args.record.is_some())
            .then(|| capture::Script::new(args.capture.clone(), args.record.clone(), page, args.demo.clone()));
        let mut app = RoboApp {
            history: HistoryState::new(),
            settings,
            engine,
            caps: None,
            page,
            log: Vec::new(),
            engine_stderr: Vec::new(),
            runs: HashMap::new(),
            lab: LabState::new(lab_map, "builtin:warehouse"),
            editor: EditorPage::default(),
            testing: TestingState::new(arena),
            gp: crate::pages::gp::GpState::new(),
            repair: crate::pages::repair::RepairState::new(),
            arm: crate::pages::arm::ArmState::new(),
            bench: BenchState::default(),
            pending_maps: HashMap::new(),
            capture,
            toast: None,
            started: Instant::now(),
        };
        app.info("RoboSBSE UI started; launching the engine.");
        app.history.rescan(&app.settings);
        app
    }

    pub fn dark(&self) -> bool {
        self.settings.dark_mode
    }

    pub fn info(&mut self, text: impl Into<String>) {
        self.push_log(Level::Info, text.into());
    }

    pub fn warn(&mut self, text: impl Into<String>) {
        self.push_log(Level::Warn, text.into());
    }

    pub fn error(&mut self, text: impl Into<String>) {
        let text = text.into();
        self.toast = Some((text.clone(), Instant::now()));
        self.push_log(Level::Error, text);
    }

    pub fn notify(&mut self, text: impl Into<String>) {
        let text = text.into();
        self.toast = Some((text.clone(), Instant::now()));
        self.push_log(Level::Info, text);
    }

    fn push_log(&mut self, level: Level, text: String) {
        self.log.push(LogLine { secs: self.started.elapsed().as_secs_f64(), level, text });
        if self.log.len() > 3000 {
            self.log.drain(0..1000);
        }
    }

    pub fn elapsed(&self) -> Duration {
        self.started.elapsed()
    }

    pub fn restart_engine(&mut self, ctx: &egui::Context) {
        for run in self.runs.values_mut() {
            if run.status.active() {
                run.status = RunStatus::Failed("engine restarted".into());
            }
        }
        self.engine.shutdown();
        self.caps = None;
        self.engine = spawn_engine(&self.settings, ctx);
        self.info(format!("engine restarted: {}", self.engine.command_line));
    }

    /// Sends a request and logs failures. Returns the request id.
    pub fn request(&mut self, kind: &str, body: Value) -> Option<String> {
        if !self.engine.is_ready() {
            self.error("the engine is not ready (see Settings)");
            return None;
        }
        match self.engine.request(kind, body) {
            Ok(id) => Some(id),
            Err(e) => {
                self.error(format!("request '{kind}' failed: {e}"));
                None
            }
        }
    }

    /// Starts a search run and registers its client-side state.
    pub fn start_run(&mut self, kind: RunKind, experiment: Value) -> Option<String> {
        let budget = experiment["budget"]["maxEvaluations"].as_u64().unwrap_or(0);
        let output = self.settings.results_dir().display().to_string();
        let id = self.request("start", json!({"experiment": experiment.clone(), "output": output}))?;
        self.info(format!("started {} ({} evaluations)", id, budget));
        self.runs.insert(id.clone(), RunState::new(id.clone(), kind, experiment, budget));
        Some(id)
    }

    pub fn control_run(&mut self, id: &str, action: &str) {
        if self.engine.request_with_id(action, id, json!({})).is_err() {
            self.error(format!("could not {action} {id}"));
        }
    }

    pub fn request_map(&mut self, reference: Value, target: MapTarget) {
        if let Some(id) = self.request("get_map", json!({"map": reference})) {
            self.pending_maps.insert(id, target);
        }
    }

    fn handle_events(&mut self) {
        for event in self.engine.poll() {
            match event {
                EngineEvent::Message(m) => self.handle_message(m),
                EngineEvent::Stderr(line) => {
                    self.engine_stderr.push(line.clone());
                    if self.engine_stderr.len() > 2000 {
                        self.engine_stderr.drain(0..500);
                    }
                    self.push_log(Level::Engine, line);
                }
                EngineEvent::Exited(reason) => self.error(format!("engine stopped: {reason}")),
            }
        }
    }

    fn handle_message(&mut self, m: Value) {
        let kind = m["type"].as_str().unwrap_or("").to_string();
        let id = m["id"].as_str().map(str::to_string);
        match kind.as_str() {
            "ready" | "hello" => match serde_json::from_value::<Capabilities>(m.clone()) {
                Ok(caps) => {
                    self.info(format!(
                        "engine {} ready (protocol {}, Java {}): {} algorithms, {} controllers",
                        caps.version,
                        caps.protocol,
                        caps.java,
                        caps.algorithms.len(),
                        caps.controllers.len()
                    ));
                    self.lab.on_capabilities(&caps);
                    self.testing.on_capabilities(&caps);
                    self.gp.on_capabilities(&caps);
                    self.repair.on_capabilities(&caps);
                    self.arm.on_capabilities(&caps);
                    self.bench.on_capabilities(&caps);
                    self.caps = Some(caps);
                }
                Err(e) => self.error(format!("cannot read engine capabilities: {e}")),
            },
            "started" => {
                if let Some(run) = id.as_ref().and_then(|i| self.runs.get_mut(i)) {
                    run.status = RunStatus::Running;
                    if let Some(max) = m["maxEvaluations"].as_u64() {
                        run.max_evaluations = max;
                    }
                }
            }
            "progress" => {
                if let Some(run) = id.as_ref().and_then(|i| self.runs.get_mut(i)) {
                    run.apply_progress(&m);
                }
            }
            "best" => {
                if let Some(run) = id.as_ref().and_then(|i| self.runs.get_mut(i)) {
                    run.apply_best(&m);
                }
            }
            "state" => {
                if let Some(run) = id.as_ref().and_then(|i| self.runs.get_mut(i)) {
                    run.status = match m["state"].as_str() {
                        Some("paused") => RunStatus::Paused,
                        Some("running") => RunStatus::Running,
                        _ => RunStatus::Cancelling,
                    };
                }
            }
            "finished" => {
                let Some(id) = id else { return };
                let result = m["result"].clone();
                let kind = match self.runs.get_mut(&id) {
                    Some(run) => {
                        run.apply_finished(result.clone());
                        run.kind
                    }
                    None => return,
                };
                self.info(format!(
                    "{id} finished: {} after {} evaluations, best fitness {:.4}",
                    result["stopReason"].as_str().unwrap_or("?"),
                    result["evaluations"],
                    model::num(&result["best"]["fitness"])
                ));
                match kind {
                    RunKind::Lab => self.lab.on_finished(&result),
                    RunKind::Testing => self.testing.on_finished(&result),
                    RunKind::Gp => self.gp.on_finished(&result),
                    RunKind::Repair => self.repair.on_finished(&result),
                    RunKind::Arm => self.arm.on_finished(&result),
                    RunKind::Benchmark => {}
                }
                self.history.rescan(&self.settings);
            }
            "benchmark_progress" => {
                if let Some(run) = id.as_ref().and_then(|i| self.runs.get_mut(i)) {
                    run.status = RunStatus::Running;
                    // Worker threads may deliver progress out of order: keep the maximum.
                    let done = m["finished"].as_u64().unwrap_or(0).max(run.bench_done.0);
                    run.bench_done = (done, m["total"].as_u64().unwrap_or(run.bench_done.1));
                    run.bench_rows.push(m["run"].clone());
                }
            }
            "benchmark_finished" => {
                if let Some(run) = id.as_ref().and_then(|i| self.runs.get_mut(i)) {
                    let result = m["result"].clone();
                    run.status = RunStatus::Finished(if result["cancelled"].as_bool() == Some(true) {
                        "CANCELLED".into()
                    } else {
                        "COMPLETED".into()
                    });
                    let executed = result["runs"].as_array().map(|a| a.len() as u64).unwrap_or(0);
                    run.bench_done = (executed, run.bench_done.1.max(executed));
                    run.result = Some(result.clone());
                    self.bench.result = Some(result);
                }
                self.info("benchmark finished");
                self.history.rescan(&self.settings);
            }
            "map" => {
                let target = id.as_ref().and_then(|i| self.pending_maps.remove(i));
                match serde_json::from_value::<WorldMap>(m["map"].clone()) {
                    Ok(map) => match target {
                        Some(MapTarget::Arena) => self.testing.set_arena(map),
                        Some(MapTarget::LabDisplay) => {
                            self.lab.display_map = Some(map);
                            self.lab.view.reset();
                        }
                        _ => {
                            let name = map.name.clone();
                            self.lab.set_map(map, &format!("builtin:{name}"));
                        }
                    },
                    Err(e) => self.error(format!("cannot read map: {e}")),
                }
            }
            "plan_result" => {
                if id.is_some() && id == self.arm.pending {
                    self.arm.pending = None;
                    self.arm.reference = Some(m["reference"].clone());
                } else {
                    self.lab.on_plan(&m["reference"]);
                }
            }
            "simulation" => {
                if id.is_some() && id == self.arm.pending {
                    self.arm.pending = None;
                    let title = format!("replayed solution (fitness {})", widgets::fmt(model::num(&m["fitness"])));
                    self.arm.show(crate::pages::arm::Motion::from_replay(&m["replay"], title));
                } else if id.is_some() && id == self.testing.pending_simulation {
                    self.testing.pending_simulation = None;
                    self.testing.on_simulation(&m);
                } else {
                    self.lab.on_simulation(&m);
                }
            }
            "coverage_progress" => self.testing.on_coverage_progress(&m),
            "coverage_finished" => {
                self.testing.on_coverage_finished(&m["result"]);
                let s = &m["result"]["summary"];
                self.info(format!("coverage campaign finished: {} of {} decision outcomes covered",
                    s["covered"], s["outcomes"]));
                self.history.rescan(&self.settings);
            }
            "reproduction" => self.testing.on_reproduction(&m),
            "error" => {
                let message = m["message"].as_str().unwrap_or("unknown error").to_string();
                let code = m["code"].as_str().unwrap_or("error").to_string();
                if let Some(run) = id.as_ref().and_then(|i| self.runs.get_mut(i)) {
                    run.status = RunStatus::Failed(message.clone());
                }
                if id.is_some() && id == self.testing.coverage_id {
                    self.testing.coverage_status = format!("failed: {message}");
                    self.testing.coverage_id = None;
                }
                if let Some(i) = &id {
                    self.pending_maps.remove(i);
                }
                self.error(format!("engine error [{code}] {}: {message}", id.unwrap_or_default()));
            }
            "bye" => {}
            other => self.warn(format!("unhandled engine message '{other}'")),
        }
    }

    pub fn run(&self, id: &Option<String>) -> Option<&RunState> {
        id.as_ref().and_then(|i| self.runs.get(i))
    }

    fn sidebar(&mut self, ui: &mut egui::Ui) {
        ui.add_space(10.0);
        ui.label(RichText::new("RoboSBSE").size(20.0).strong());
        ui.label(RichText::new("search-based robotics lab").small().weak());
        ui.add_space(12.0);
        for page in Page::ALL {
            let selected = self.page == page;
            let label = RichText::new(page.title());
            if ui.add_sized([ui.available_width(), 26.0], egui::Button::selectable(selected, label)).clicked() {
                self.page = page;
            }
        }
        ui.with_layout(egui::Layout::bottom_up(egui::Align::Min), |ui| {
            ui.add_space(8.0);
            let (text, color) = match &self.engine.status {
                EngineStatus::Ready => ("engine ready".to_string(), crate::canvas::START),
                EngineStatus::Starting => ("engine starting...".to_string(), crate::canvas::GOAL),
                EngineStatus::Stopped => ("engine stopped".to_string(), crate::canvas::DANGER),
                EngineStatus::Failed(e) => (format!("engine error: {e}"), crate::canvas::DANGER),
            };
            ui.add(egui::Label::new(RichText::new(text).color(color).small()).wrap());
            if let Some(caps) = &self.caps {
                ui.label(RichText::new(format!("engine {} / protocol {}", caps.version, caps.protocol)).small().weak());
            }
        });
    }

    fn status_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label(RichText::new(self.page.title()).strong());
            ui.separator();
            let active: Vec<String> = self
                .runs
                .values()
                .filter(|r| r.status.active())
                .map(|r| format!("{} {} {}/{}", r.id, r.status.label(), r.evaluations, r.max_evaluations))
                .collect();
            if active.is_empty() {
                ui.label(RichText::new("no active runs").weak());
            } else {
                ui.label(active.join("   |   "));
            }
            if let Some((text, at)) = &self.toast {
                if at.elapsed() < Duration::from_secs(6) {
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(RichText::new(text).color(Color32::from_rgb(235, 200, 120)));
                    });
                }
            }
        });
    }
}

impl eframe::App for RoboApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.handle_events();
        if self.capture.is_some() {
            capture::step(self, &ctx);
        }
        let dt = ctx.input(|i| i.stable_dt).min(0.1) as f64;
        let animating = self.lab.replay.tick(dt) | self.testing.replay.tick(dt);
        let busy = self.runs.values().any(|r| r.status.active()) || self.engine.status == EngineStatus::Starting;
        if animating {
            ctx.request_repaint();
        } else if busy {
            ctx.request_repaint_after(Duration::from_millis(100));
        }

        egui::Panel::left("nav").exact_size(206.0).show(ui, |ui| self.sidebar(ui));
        egui::Panel::top("status").show(ui, |ui| self.status_bar(ui));
        match self.page {
            Page::Dashboard => self.dashboard_page(ui),
            Page::Lab => self.lab_page(ui),
            Page::Editor => self.editor_page(ui),
            Page::Algorithms => self.algorithms_page(ui),
            Page::Testing => self.testing_page(ui),
            Page::GeneticProgramming => self.gp_page(ui),
            Page::ProgramRepair => self.repair_page(ui),
            Page::Arm => self.arm_page(ui),
            Page::Benchmark => self.benchmark_page(ui),
            Page::History => self.history_page(ui),
            Page::Settings => self.settings_page(ui),
        }
    }

    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        self.engine.shutdown();
    }
}

pub fn apply_style(ctx: &egui::Context, settings: &Settings) {
    ctx.set_visuals(if settings.dark_mode { egui::Visuals::dark() } else { egui::Visuals::light() });
    ctx.set_zoom_factor(settings.ui_scale);
}

fn spawn_engine(settings: &Settings, ctx: &egui::Context) -> Engine {
    let root = settings::find_repo_root();
    Engine::spawn(&settings.java, &PathBuf::from(&settings.engine_jar), &root, ctx.clone())
}

pub fn load_map_file(path: &std::path::Path) -> Option<WorldMap> {
    std::fs::read_to_string(path).ok().and_then(|t| serde_json::from_str(&t).ok())
}

/// Shared widgets.
pub mod widgets {
    use eframe::egui::{self, RichText};
    use egui_plot::{Legend, Line, Plot, PlotPoints};
    use serde_json::Value;

    use crate::runs::RunState;

    pub fn fmt(v: f64) -> String {
        if !v.is_finite() {
            "-".into()
        } else if v.abs() >= 1000.0 {
            format!("{v:.1}")
        } else {
            format!("{v:.4}")
        }
    }

    /// Status line, progress bar and algorithm statistics of a run.
    pub fn run_status(ui: &mut egui::Ui, run: &RunState, target_label: &str) {
        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new(run.status.label()).strong());
            ui.separator();
            ui.monospace(format!("{} / {} evaluations", run.evaluations, run.max_evaluations));
            ui.separator();
            ui.monospace(format!("{:.1} s", run.elapsed_ms as f64 / 1000.0));
            ui.separator();
            ui.label("best fitness");
            ui.monospace(fmt(run.best_fitness));
            ui.separator();
            ui.label(target_label);
            if run.targets > 0 {
                ui.monospace(format!("{} (first at evaluation {})", run.targets, run.first_target));
            } else {
                ui.monospace("0");
            }
        });
        ui.add(egui::ProgressBar::new(run.progress_fraction()).show_percentage());
        if let Some(stats) = run.stats.as_object() {
            if !stats.is_empty() {
                ui.horizontal_wrapped(|ui| {
                    for (k, v) in stats {
                        ui.label(RichText::new(format!("{k}: {}", fmt(v.as_f64().unwrap_or(f64::NAN)))).small().monospace());
                    }
                });
            }
        }
    }

    /// Best-so-far fitness against evaluations (exact step curve from the engine).
    pub fn convergence_plot(ui: &mut egui::Ui, id: &str, run: &RunState, height: f32, log_y: bool) {
        let mut pts: Vec<[f64; 2]> = Vec::new();
        for (i, p) in run.convergence.iter().enumerate() {
            if i > 0 {
                pts.push([p[0], run.convergence[i - 1][1]]);
            }
            pts.push(*p);
        }
        if let Some(last) = run.convergence.last() {
            pts.push([run.evaluations.max(last[0] as u64) as f64, last[1]]);
        }
        let transform = |y: f64| if log_y { y.max(1e-9).log10() } else { y };
        let shown: Vec<[f64; 2]> = pts.iter().map(|p| [p[0], transform(p[1])]).collect();
        Plot::new(id)
            .height(height)
            .legend(Legend::default())
            .x_axis_label("fitness evaluations")
            .y_axis_label(if log_y { "log10 best fitness" } else { "best fitness" })
            .allow_scroll(false)
            .show(ui, |plot| {
                plot.line(Line::new("best so far", PlotPoints::from(shown)).color(crate::canvas::ROUTE).width(2.0));
            });
    }

    /// Algorithm-specific series recorded in a result (diversity, temperature, behaviour coverage, ...).
    /// Series live on very different scales, so one is shown at a time; the population best/mean
    /// curves are drawn together because they share the fitness scale.
    pub fn series_plot(ui: &mut egui::Ui, id: &str, series: &Value, height: f32, choice: &mut String) {
        let Some(map) = series.as_object() else { return };
        let names: Vec<String> = map
            .iter()
            .filter(|(k, v)| crate::model::points(v).len() > 1 && !k.starts_with("population"))
            .map(|(k, _)| k.clone())
            .chain(map.contains_key("populationBest").then(|| "population fitness".to_string()))
            .collect();
        if names.is_empty() {
            return;
        }
        if !names.contains(choice) {
            *choice = names[0].clone();
        }
        egui::ComboBox::from_id_salt((id, "choice")).selected_text(choice.as_str()).show_ui(ui, |ui| {
            for n in &names {
                ui.selectable_value(choice, n.clone(), n);
            }
        });
        let shown: Vec<&str> = if choice == "population fitness" {
            vec!["populationBest", "populationMean"]
        } else {
            vec![choice.as_str()]
        };
        let mut plot = Plot::new(id).height(height - 24.0);
        if shown.len() > 1 {
            plot = plot.legend(Legend::default());
        }
        plot.x_axis_label("fitness evaluations")
            .allow_scroll(false)
            .show(ui, |plot| {
                for name in shown {
                    let pts = crate::model::points(&map[name]);
                    if pts.len() > 1 {
                        plot.line(Line::new(name.to_string(), PlotPoints::from(pts)));
                    }
                }
            });
    }

    pub fn metrics_grid(ui: &mut egui::Ui, id: &str, columns: &[(&str, &Value)]) {
        let mut keys: Vec<String> = Vec::new();
        for (_, m) in columns {
            if let Some(obj) = m.as_object() {
                for k in obj.keys() {
                    if !keys.contains(k) {
                        keys.push(k.clone());
                    }
                }
            }
        }
        egui::Grid::new(id).striped(true).num_columns(columns.len() + 1).show(ui, |ui| {
            ui.label(RichText::new("metric").strong());
            for (title, _) in columns {
                ui.label(RichText::new(*title).strong());
            }
            ui.end_row();
            for k in keys {
                ui.label(&k);
                for (_, m) in columns {
                    ui.monospace(m.get(&k).and_then(Value::as_f64).map(fmt).unwrap_or_else(|| "-".into()));
                }
                ui.end_row();
            }
        });
    }

    pub fn open_in_file_manager(path: &std::path::Path) {
        #[cfg(windows)]
        let program = "explorer";
        #[cfg(target_os = "macos")]
        let program = "open";
        #[cfg(all(unix, not(target_os = "macos")))]
        let program = "xdg-open";
        let _ = std::process::Command::new(program).arg(path).spawn();
    }
}
