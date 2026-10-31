//! Benchmark Suite: compare algorithms on the same problem with the same seeds and evaluation
//! budget. All tables and plots are computed by the engine from the executed runs.

use std::collections::BTreeMap;

use eframe::egui::{self, Color32, RichText};
use egui_plot::{BoxElem, BoxPlot, BoxSpread, HLine, Legend, Line, Plot, PlotPoints};
use serde_json::{json, Value};

use crate::app::{widgets, RoboApp};
use crate::model::{self, Capabilities};
use crate::runs::{RunKind, RunState};

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Source {
    Lab,
    Testing,
}

pub struct BenchState {
    pub source: Source,
    pub selected: BTreeMap<String, bool>,
    pub runs: u32,
    pub base_seed: u64,
    pub budget: u64,
    pub threads: u32,
    pub name: String,
    pub run_id: Option<String>,
    pub result: Option<Value>,
    pub log_y: bool,
    /// Per-run metric for the statistical comparison: bestFitness, hypervolume, behaviourCoverage or
    /// timeToTarget (evaluations to the first target hit; runs without one ranked last).
    pub compare_by: String,
}

impl Default for BenchState {
    fn default() -> Self {
        BenchState {
            source: Source::Lab,
            selected: BTreeMap::new(),
            runs: 10,
            base_seed: 1,
            budget: 1500,
            threads: std::thread::available_parallelism().map(|n| n.get().min(8) as u32).unwrap_or(2),
            name: "comparison".into(),
            run_id: None,
            result: None,
            log_y: true,
            compare_by: "bestFitness".into(),
        }
    }
}

const PALETTE: [Color32; 8] = [
    Color32::from_rgb(56, 189, 222),
    Color32::from_rgb(240, 150, 60),
    Color32::from_rgb(120, 200, 100),
    Color32::from_rgb(220, 90, 120),
    Color32::from_rgb(170, 130, 230),
    Color32::from_rgb(230, 210, 90),
    Color32::from_rgb(90, 140, 240),
    Color32::from_rgb(200, 200, 200),
];

impl BenchState {
    pub fn on_capabilities(&mut self, caps: &Capabilities) {
        for a in &caps.algorithms {
            let default_on = matches!(a.id.as_str(), "random-search" | "hill-climbing" | "simulated-annealing" | "genetic-algorithm");
            self.selected.entry(a.id.clone()).or_insert(default_on);
        }
    }
}

impl RoboApp {
    pub fn start_benchmark_now(&mut self) {
        let Some(caps) = self.caps.clone() else { return };
        let (experiment, values, problem_id) = match self.bench.source {
            Source::Lab => (self.lab.experiment(&caps), self.lab.algo_values.clone(), self.lab.problem_type.clone()),
            Source::Testing => (self.testing.experiment(&caps), self.testing.algo_values.clone(), "controller-testing".to_string()),
        };
        let problem = caps.problem(&problem_id).cloned();
        let operators = problem.as_ref().map(|p| p.operators.clone());
        let mut algorithms = Vec::new();
        for a in caps.algorithms_for(&problem_id) {
            if self.bench.selected.get(&a.id).copied().unwrap_or(false) {
                let params = values
                    .get(&a.id)
                    .map(|v| crate::params::to_json_for(&a.params, v, operators.as_ref()))
                    .unwrap_or(json!({}));
                algorithms.push(json!({"type": a.id, "label": a.name, "params": params}));
            }
        }
        let has_objectives = problem.map(|p| !p.objectives.is_empty()).unwrap_or(false);
        let compare_by = if has_objectives || self.bench.compare_by != "hypervolume" {
            self.bench.compare_by.clone()
        } else {
            "bestFitness".to_string()
        };
        if algorithms.is_empty() {
            return self.error("select at least one algorithm");
        }
        let benchmark = json!({
            "schema": "robosbse.benchmark/1",
            "name": self.bench.name,
            "runs": self.bench.runs,
            "baseSeed": self.bench.base_seed,
            "threads": self.bench.threads,
            "budget": {"maxEvaluations": self.bench.budget},
            "problem": experiment["problem"],
            "algorithms": algorithms,
            "options": {"archiveSize": 0, "minimizeFailures": false},
            "compareBy": compare_by,
            "reference": true
        });
        let output = self.settings.benchmarks_dir().display().to_string();
        if let Some(id) = self.request("benchmark", json!({"benchmark": benchmark.clone(), "output": output})) {
            let total = self.bench.runs as u64 * benchmark["algorithms"].as_array().map(|a| a.len()).unwrap_or(0) as u64;
            let mut run = RunState::new(id.clone(), RunKind::Benchmark, benchmark, self.bench.budget);
            run.bench_done = (0, total);
            self.runs.insert(id.clone(), run);
            self.bench.run_id = Some(id);
            self.bench.result = None;
        }
    }

    fn benchmark_config(&mut self, ui: &mut egui::Ui) {
        let Some(caps) = self.caps.clone() else {
            ui.label("Waiting for the engine...");
            return;
        };
        let active = self.run(&self.bench.run_id).map(|r| r.status.active()).unwrap_or(false);
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("Comparison");
            ui.label("Problem (taken from the page's current configuration)");
            ui.radio_value(&mut self.bench.source, Source::Lab, format!("Laboratory: route optimisation on '{}'", self.lab.map.name));
            ui.radio_value(&mut self.bench.source, Source::Testing,
                format!("Testing: failure discovery for '{}'", self.testing.controller()));
            ui.add_space(6.0);
            let problem_id = match self.bench.source {
                Source::Lab => self.lab.problem_type.clone(),
                Source::Testing => "controller-testing".to_string(),
            };
            let problem = caps.problem(&problem_id).cloned();
            ui.label(RichText::new("Algorithms").strong());
            ui.label(RichText::new("Each uses the parameters currently set on that page.").small().weak());
            for a in &caps.algorithms {
                let supported = problem.as_ref().map(|p| a.supports(p)).unwrap_or(true);
                let on = self.bench.selected.entry(a.id.clone()).or_insert(false);
                ui.add_enabled(supported, egui::Checkbox::new(on, &a.name))
                    .on_disabled_hover_text("not applicable to this problem's representation or objectives");
            }
            let has_objectives = problem.as_ref().map(|p| !p.objectives.is_empty()).unwrap_or(false);
            ui.horizontal_wrapped(|ui| {
                ui.label("Compare by");
                ui.selectable_value(&mut self.bench.compare_by, "bestFitness".to_string(), "best fitness");
                if has_objectives {
                    ui.selectable_value(&mut self.bench.compare_by, "hypervolume".to_string(), "hypervolume");
                }
                ui.selectable_value(&mut self.bench.compare_by, "timeToTarget".to_string(), "time to target")
                    .on_hover_text("Evaluations until the first target hit (e.g. failure); runs without one rank last");
                ui.selectable_value(&mut self.bench.compare_by, "behaviourCoverage".to_string(), "behaviour coverage")
                    .on_hover_text("Share of a 10 x 10 grid over the first two behaviour components that the run visited");
            });
            ui.add_space(6.0);
            egui::Grid::new("bench-cfg").num_columns(2).show(ui, |ui| {
                ui.label("Name");
                ui.text_edit_singleline(&mut self.bench.name);
                ui.end_row();
                ui.label("Runs (seeds) per algorithm");
                ui.add(egui::DragValue::new(&mut self.bench.runs).range(1..=1000));
                ui.end_row();
                ui.label("First seed");
                ui.add(egui::DragValue::new(&mut self.bench.base_seed));
                ui.end_row();
                ui.label("Evaluation budget (equal)");
                ui.add(egui::DragValue::new(&mut self.bench.budget).range(1..=10_000_000).speed(50));
                ui.end_row();
                ui.label("Worker threads");
                ui.add(egui::DragValue::new(&mut self.bench.threads).range(1..=64));
                ui.end_row();
            });
            ui.add_space(8.0);
            ui.horizontal_wrapped(|ui| {
                if ui.add_enabled(!active, egui::Button::new(RichText::new("Run benchmark").strong())).clicked() {
                    self.start_benchmark_now();
                }
                if active {
                    if let Some(id) = self.bench.run_id.clone() {
                        if ui.button("Cancel").clicked() {
                            self.control_run(&id, "cancel");
                        }
                    }
                }
                if ui.button("Load benchmark.json").clicked() {
                    if let Some(path) = rfd::FileDialog::new()
                        .set_directory(self.settings.benchmarks_dir())
                        .add_filter("benchmark", &["json"])
                        .pick_file()
                    {
                        match std::fs::read_to_string(&path).ok().and_then(|t| serde_json::from_str::<Value>(&t).ok()) {
                            Some(v) if v["schema"] == "robosbse.benchmark-result/1" => self.bench.result = Some(v),
                            _ => self.error("not a robosbse.benchmark-result/1 file"),
                        }
                    }
                }
            });
            if let Some(run) = self.run(&self.bench.run_id) {
                ui.add(egui::ProgressBar::new(if run.bench_done.1 > 0 {
                    run.bench_done.0 as f32 / run.bench_done.1 as f32
                } else {
                    0.0
                })
                .text(format!("{} / {} runs  ({})", run.bench_done.0, run.bench_done.1, run.status.label())));
            }
        });
    }

    pub fn benchmark_page(&mut self, ui: &mut egui::Ui) {
        egui::Panel::right("bench-config").default_size(320.0).resizable(true).show(ui, |ui| self.benchmark_config(ui));
        egui::CentralPanel::default().show(ui, |ui| {
            let live_rows = self.run(&self.bench.run_id).map(|r| r.bench_rows.clone()).unwrap_or_default();
            let Some(result) = self.bench.result.clone() else {
                if live_rows.is_empty() {
                    ui.label("Select algorithms and press 'Run benchmark'. Every algorithm gets the same seeds and evaluation budget.");
                } else {
                    ui.label(RichText::new("Finished runs").strong());
                    runs_table(ui, &live_rows);
                }
                return;
            };
            let mut log_y = self.bench.log_y;
            egui::ScrollArea::vertical().show(ui, |ui| benchmark_report(ui, &result, &mut log_y));
            self.bench.log_y = log_y;
        });
    }
}

fn runs_table(ui: &mut egui::Ui, rows: &[Value]) {
    egui::ScrollArea::vertical().id_salt("bench-runs").max_height(260.0).show(ui, |ui| {
        egui::Grid::new("bench-runs-grid").striped(true).num_columns(8).show(ui, |ui| {
            for h in ["algorithm", "seed", "best fitness", "success", "first target", "ms", "CPU ms", "stop"] {
                ui.label(RichText::new(h).strong());
            }
            ui.end_row();
            for r in rows {
                ui.label(r["algorithm"].as_str().unwrap_or(""));
                ui.monospace(format!("{}", r["seed"]));
                ui.monospace(widgets::fmt(model::num(&r["bestFitness"])));
                ui.monospace(if r["success"].as_bool() == Some(true) { "yes" } else { "no" });
                ui.monospace(format!("{}", r["firstTargetEvaluation"]));
                ui.monospace(format!("{}", r["elapsedMs"]));
                ui.monospace(r["cpuMs"].as_u64().map(|c| c.to_string()).unwrap_or_default());
                ui.monospace(r["stopReason"].as_str().unwrap_or(""));
                ui.end_row();
            }
        });
    });
}

/// log10 for plotting fitness values spanning several orders of magnitude (penalties vs. successes).
fn scale(v: f64, log_y: bool) -> f64 {
    if log_y {
        v.max(1e-6).log10()
    } else {
        v
    }
}

pub fn benchmark_report(ui: &mut egui::Ui, result: &Value, log_y: &mut bool) {
    let bench = &result["benchmark"];
    ui.horizontal_wrapped(|ui| {
        ui.heading(bench["name"].as_str().unwrap_or("benchmark"));
        ui.label(format!(
            "{} | budget {} evaluations | {} runs | {} threads",
            bench["problem"]["type"].as_str().unwrap_or("?"),
            bench["budget"]["maxEvaluations"],
            result["runs"].as_array().map(|a| a.len()).unwrap_or(0),
            result["threads"]
        ));
        if let Some(dir) = result["outputDir"].as_str() {
            if ui.small_button("open output folder (CSV / JSON)").clicked() {
                widgets::open_in_file_manager(std::path::Path::new(dir));
            }
        }
    });
    let summary = result["summary"].as_array().cloned().unwrap_or_default();
    ui.label(RichText::new("Final best fitness per algorithm (lower is better)").strong());
    let has_validation = summary.iter().any(|s| s["validation"].is_object());
    let has_coverage = summary.iter().any(|s| s["behaviourCoverage"].is_object());
    let has_minimised = summary.iter().any(|s| s["minimisedFailures"].is_object());
    egui::Grid::new("bench-summary").striped(true).num_columns(14).show(ui, |ui| {
        for h in ["algorithm", "runs", "median", "mean", "sd", "min", "max", "success rate", "median first target",
            "median ms", "median CPU ms"] {
            ui.label(RichText::new(h).strong());
        }
        if has_coverage {
            ui.label(RichText::new("behaviour coverage").strong())
                .on_hover_text("Median share of a 10 x 10 grid over the first two behaviour components (e.g. where routes end)");
        }
        if has_minimised {
            ui.label(RichText::new("minimised obstacles").strong())
                .on_hover_text("Median obstacle count of each run's first failure before -> after minimisation");
        }
        if has_validation {
            ui.label(RichText::new("validation median").strong())
                .on_hover_text("Fitness of each run's best solution on the held-out validation maps");
        }
        ui.end_row();
        for (i, s) in summary.iter().enumerate() {
            ui.label(RichText::new(s["algorithm"].as_str().unwrap_or("")).color(PALETTE[i % PALETTE.len()]));
            ui.monospace(format!("{}", s["runs"]));
            for k in ["median", "mean", "sd", "min", "max"] {
                ui.monospace(widgets::fmt(model::num(&s[k])));
            }
            ui.monospace(format!("{:.0}%", 100.0 * model::num(&s["successRate"])));
            ui.monospace(widgets::fmt(model::num(&s["medianFirstTargetEvaluation"])));
            ui.monospace(format!("{:.0}", model::num(if s["medianMillis"].is_number() { &s["medianMillis"] } else { &s["meanMillis"] })));
            ui.monospace(if s["medianCpuMillis"].is_number() { format!("{:.0}", model::num(&s["medianCpuMillis"])) } else { "-".into() });
            if has_coverage {
                ui.monospace(widgets::fmt(model::num(&s["behaviourCoverage"]["median"])));
            }
            if has_minimised {
                let m = &s["minimisedFailures"];
                ui.monospace(if m.is_object() {
                    format!("{} -> {}", widgets::fmt(model::num(&m["medianObstaclesBefore"])),
                        widgets::fmt(model::num(&m["medianObstaclesAfter"])))
                } else {
                    "-".into()
                });
            }
            if has_validation {
                ui.monospace(widgets::fmt(model::num(&s["validation"]["median"])));
            }
            ui.end_row();
        }
    });
    let reference_block = &result["reference"];
    let (reference_name, reference_fitness) = if reference_block["astar"].is_object() {
        ("A* route", model::num(&reference_block["astar"]["fitness"]))
    } else if reference_block["greedy"].is_object() {
        ("greedy dispatching", model::num(&reference_block["greedy"]["fitness"]))
    } else if reference_block["defaults"].is_object() {
        ("untuned defaults", model::num(&reference_block["defaults"]["fitness"]))
    } else {
        ("", f64::NAN)
    };
    if reference_block["astar"].is_object() {
        ui.label(format!(
            "A* reference (not a metaheuristic): fitness {} with {} node expansions — reported separately, not in fitness evaluations.",
            widgets::fmt(reference_fitness),
            reference_block["astar"]["expansions"]
        ));
    } else if reference_fitness.is_finite() {
        ui.label(format!("Conventional reference ({reference_name}): fitness {}.", widgets::fmt(reference_fitness)));
    }
    if summary.iter().any(|s| s["hypervolume"].is_object()) {
        ui.add_space(6.0);
        ui.label(RichText::new("Normalised hypervolume of each run's Pareto archive (higher is better)").strong());
        egui::Grid::new("bench-hv").striped(true).num_columns(6).show(ui, |ui| {
            for h in ["algorithm", "median", "mean", "sd", "min", "max"] {
                ui.label(RichText::new(h).strong());
            }
            ui.end_row();
            for (i, s) in summary.iter().enumerate() {
                let hv = &s["hypervolume"];
                ui.label(RichText::new(s["algorithm"].as_str().unwrap_or("")).color(PALETTE[i % PALETTE.len()]));
                for k in ["median", "mean", "sd", "min", "max"] {
                    ui.monospace(widgets::fmt(model::num(&hv[k])));
                }
                ui.end_row();
            }
        });
    }
    ui.add_space(8.0);
    ui.checkbox(log_y, "log10 fitness axis (failed runs carry a 10 000 penalty in navigation)");
    let log_y = *log_y;
    ui.columns(2, |cols| {
        cols[0].label(RichText::new("Distribution of final best fitness").strong());
        let boxes: Vec<BoxPlot> = summary
            .iter()
            .enumerate()
            .map(|(i, s)| {
                let v = |k: &str| scale(model::num(&s[k]), log_y);
                let spread = BoxSpread::new(v("min"), v("q1"), v("median"), v("q3"), v("max"));
                BoxPlot::new(s["algorithm"].as_str().unwrap_or("").to_string(),
                    vec![BoxElem::new(i as f64, spread).box_width(0.6).name(s["algorithm"].as_str().unwrap_or(""))])
                .color(PALETTE[i % PALETTE.len()])
            })
            .collect();
        let ref_fitness = scale(reference_fitness, log_y);
        Plot::new("bench-box").height(300.0).legend(Legend::default()).allow_scroll(false).show(&mut cols[0], |plot| {
            for b in boxes {
                plot.box_plot(b);
            }
            if ref_fitness.is_finite() {
                plot.hline(HLine::new(reference_name, ref_fitness).color(crate::canvas::REFERENCE));
            }
        });
        cols[1].label(RichText::new("Median best-so-far (dotted: quartiles)").strong());
        let convergence = result["convergence"].clone();
        Plot::new("bench-conv").height(300.0).legend(Legend::default()).allow_scroll(false)
            .x_axis_label("fitness evaluations").show(&mut cols[1], |plot| {
                for (i, s) in summary.iter().enumerate() {
                    let name = s["algorithm"].as_str().unwrap_or("").to_string();
                    let rows = convergence[&name].as_array().cloned().unwrap_or_default();
                    let col = |k: usize| -> Vec<[f64; 2]> {
                        rows.iter().filter_map(|r| Some([r.get(0)?.as_f64()?, scale(r.get(k)?.as_f64()?, log_y)])).collect()
                    };
                    let color = PALETTE[i % PALETTE.len()];
                    plot.line(Line::new(name.clone(), PlotPoints::from(col(1))).color(color).width(2.0));
                    for k in [2, 3] {
                        plot.line(Line::new(name.clone(), PlotPoints::from(col(k))).color(color.gamma_multiply(0.5))
                            .style(egui_plot::LineStyle::dotted_dense()));
                    }
                }
            });
    });
    ui.add_space(8.0);
    ui.label(RichText::new(format!(
        "Pairwise comparison on {}: two-sided Mann-Whitney U, Holm-corrected; A12 = P(A better than B)",
        result["compareBy"].as_str().unwrap_or("bestFitness")
    )).strong());
    egui::Grid::new("bench-pairs").striped(true).num_columns(6).show(ui, |ui| {
        for h in ["A", "B", "p", "p (Holm)", "A12", "effect"] {
            ui.label(RichText::new(h).strong());
        }
        ui.end_row();
        for p in result["pairwise"].as_array().cloned().unwrap_or_default() {
            ui.label(p["a"].as_str().unwrap_or(""));
            ui.label(p["b"].as_str().unwrap_or(""));
            ui.monospace(format!("{:.4}", model::num(&p["pValue"])));
            let holm = model::num(&p["pHolm"]);
            ui.label(RichText::new(format!("{holm:.4}")).monospace().strong().color(if holm < 0.05 {
                crate::canvas::START
            } else {
                ui.visuals().text_color()
            }));
            ui.monospace(format!("{:.3}", model::num(&p["a12"])));
            ui.label(p["effect"].as_str().unwrap_or(""));
            ui.end_row();
        }
    });
    ui.add_space(8.0);
    ui.collapsing("All runs", |ui| runs_table(ui, result["runs"].as_array().map(|a| a.as_slice()).unwrap_or(&[])));
}
