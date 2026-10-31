//! Command-line options and the scripted capture mode used for documentation screenshots and
//! smoke tests: `robosbse --page lab --demo lab --capture shot.png` waits for the engine, runs a
//! real experiment, positions the replay, saves a PNG of the window and exits.

use std::path::PathBuf;
use std::time::{Duration, Instant};

use eframe::egui;
use serde_json::json;

use crate::app::{MapTarget, Page, RoboApp};

#[derive(Clone, Debug, Default, PartialEq)]
pub struct CliArgs {
    pub capture: Option<PathBuf>,
    pub page: Option<String>,
    pub demo: Option<String>,
}

impl CliArgs {
    pub fn parse(args: impl Iterator<Item = String>) -> CliArgs {
        let mut out = CliArgs::default();
        let mut it = args.peekable();
        while let Some(a) = it.next() {
            match a.as_str() {
                "--capture" => out.capture = it.next().map(PathBuf::from),
                "--page" => out.page = it.next(),
                "--demo" => out.demo = it.next(),
                "--help" | "-h" => {
                    println!(
                        "robosbse [--page dashboard|lab|editor|algorithms|testing|benchmark|history|settings]\n\
                         \x20        [--demo lab|astar|testing|benchmark|tuning|warehouse|pareto|novelty|coverage|gp|repair|improve|\n\
                         \x20               arm|arm-ik|arm-trajectory|fleet|dynamic] [--capture out.png]"
                    );
                    std::process::exit(0);
                }
                _ => {}
            }
        }
        out
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Phase {
    WaitEngine,
    Act,
    WaitResult,
    Settle(u32),
    Shoot,
    WaitShot,
    Done,
}

pub struct Script {
    out: PathBuf,
    page: Page,
    demo: Option<String>,
    phase: Phase,
    started: Instant,
}

impl Script {
    pub fn new(out: PathBuf, page: Page, demo: Option<String>) -> Script {
        Script { out, page, demo, phase: Phase::WaitEngine, started: Instant::now() }
    }
}

fn result_ready(app: &RoboApp, demo: &str) -> bool {
    match demo {
        "lab" | "tuning" | "warehouse" | "pareto" | "novelty" | "fleet" | "dynamic" => {
            app.run(&app.lab.run_id).map(|r| r.result.is_some()).unwrap_or(false)
        }
        "coverage" => app.testing.covering.is_some() && app.testing.pending_simulation.is_none(),
        "gp" => app.gp.result.is_some(),
        "arm" | "arm-ik" | "arm-trajectory" => app.arm.result.is_some() && app.arm.pending.is_none(),
        "repair" | "improve" => app.repair.result.is_some(),
        "astar" => app.lab.replay.trajectory.is_some(),
        "testing" => app.run(&app.testing.run_id).map(|r| r.result.is_some()).unwrap_or(false),
        "benchmark" => app.bench.result.is_some(),
        _ => true,
    }
}

/// Advances the capture script by one frame.
pub fn step(app: &mut RoboApp, ctx: &egui::Context) {
    let Some(mut script) = app.capture.take() else { return };
    let timeout = script.started.elapsed() > Duration::from_secs(240);
    let demo = script.demo.clone().unwrap_or_default();
    script.phase = match script.phase {
        Phase::WaitEngine if app.caps.is_some() => {
            app.page = script.page;
            Phase::Act
        }
        Phase::WaitEngine if timeout => Phase::Shoot,
        Phase::Act => {
            match demo.as_str() {
                "lab" => app.start_lab_run(),
                "tuning" => {
                    app.set_lab_problem(crate::pages::lab::TUNING);
                    app.lab.values().insert("controller".into(), json!("reactive"));
                    app.lab.values().insert("validationMaps".into(), json!("testing-arena"));
                    app.lab.algorithm = "particle-swarm".into();
                    app.lab.budget = 300;
                    app.start_lab_run();
                }
                "warehouse" => {
                    app.set_lab_problem(crate::pages::lab::WAREHOUSE);
                    app.lab.algorithm = "genetic-algorithm".into();
                    app.lab.algo_values.entry("genetic-algorithm".into()).or_default().insert("crossover".into(), json!("order"));
                    app.lab.budget = 3000;
                    app.start_lab_run();
                }
                "pareto" => {
                    let file = std::path::PathBuf::from(&app.settings.maps_dir).join("obstacle-course.json");
                    if let Some(map) = crate::app::load_map_file(&file) {
                        app.lab.set_map(map, "builtin:obstacle-course");
                    }
                    app.lab.values().insert("waypoints".into(), json!(4));
                    app.lab.values().insert("referenceTime".into(), json!(60.0));
                    app.lab.values().insert("referenceEnergy".into(), json!(800.0));
                    app.lab.algorithm = "nsga2".into();
                    app.lab.budget = 3000;
                    app.start_lab_run();
                }
                "novelty" => {
                    let file = std::path::PathBuf::from(&app.settings.maps_dir).join("maze.json");
                    if let Some(map) = crate::app::load_map_file(&file) {
                        app.lab.set_map(map, "builtin:maze");
                    }
                    for (k, v) in [("waypoints", json!(10)), ("controller", json!("pure-pursuit")), ("maxSteps", json!(2400)),
                        ("repair", json!(true))] {
                        app.lab.values().insert(k.into(), v);
                    }
                    app.lab.algorithm = "novelty-search".into();
                    app.lab.algo_values.entry("novelty-search".into()).or_default().insert("populationSize".into(), json!(60));
                    app.lab.algo_values.entry("novelty-search".into()).or_default().insert("offspring".into(), json!(60));
                    app.lab.budget = 4000;
                    app.lab.threads = 4;
                    app.start_lab_run();
                }
                "coverage" => {
                    app.testing.problem.insert("controller".into(), json!("reactive-bug3-boost"));
                    app.testing.problem.insert("minBattery".into(), json!(0.2));
                    app.testing.coverage_budget = 3000;
                    app.testing.threads = 4;
                    app.start_coverage_campaign();
                }
                "gp" => {
                    app.gp.budget = 3000;
                    app.gp.threads = 4;
                    app.start_gp_run();
                }
                "repair" | "improve" => {
                    if demo == "improve" {
                        app.repair.mode = crate::pages::repair::IMPROVEMENT.into();
                        app.repair.algorithm = "nsga2".into();
                    } else {
                        app.repair.problems.entry(crate::pages::repair::REPAIR.into()).or_default()
                            .insert("subject".into(), json!("rb2-comparison"));
                    }
                    app.repair.budget = 200;
                    app.repair.threads = 2;
                    app.start_repair_run();
                }
                "arm" | "arm-ik" | "arm-trajectory" => {
                    if let Some(caps) = app.caps.clone() {
                        let (problem, scene, algorithm, budget) = match demo.as_str() {
                            "arm-ik" => (crate::pages::arm::IK, "arm-cluttered", "particle-swarm", 3000),
                            "arm-trajectory" => (crate::pages::arm::TRAJECTORY, "arm-cluttered", "genetic-algorithm", 3000),
                            _ => (crate::pages::arm::PICK_PLACE, "arm-pick-place", "genetic-algorithm", 1500),
                        };
                        app.arm.set_problem(&caps, problem);
                        app.arm.set_scene(&caps, scene);
                        app.arm.algorithm = algorithm.into();
                        app.arm.budget = budget;
                        app.start_arm_run();
                    }
                }
                "fleet" => {
                    app.set_lab_problem(crate::pages::lab::MULTI);
                    app.lab.algorithm = "genetic-algorithm".into();
                    app.lab.algo_values.entry("genetic-algorithm".into()).or_default().insert("populationSize".into(), json!(50));
                    app.lab.budget = 3000;
                    app.lab.seed = 1;
                    app.start_lab_run();
                }
                "dynamic" => {
                    let file = std::path::PathBuf::from(&app.settings.maps_dir).join("dynamic-obstacle-course.json");
                    if let Some(map) = crate::app::load_map_file(&file) {
                        app.lab.set_map(map, "builtin:dynamic-obstacle-course");
                    }
                    app.lab.values().insert("waypoints".into(), json!(6));
                    app.lab.algorithm = "genetic-algorithm".into();
                    app.lab.budget = 3000;
                    app.lab.seed = 3;
                    app.lab.threads = 2;
                    app.start_lab_run();
                }
                "astar" => {
                    let problem = json!({"type": "navigation", "map": app.lab.map.to_json()});
                    app.request("plan", json!({"problem": problem}));
                }
                "testing" => {
                    app.testing.problem.insert("controller".into(), json!("reactive-bug1-dropout"));
                    app.testing.problem.insert("fitness".into(), json!("collision"));
                    app.start_testing_run();
                }
                "benchmark" => {
                    app.bench.runs = 6;
                    app.bench.budget = 800;
                    app.request_map(json!("builtin:obstacle-course"), MapTarget::Lab);
                    app.lab.values().insert("waypoints".into(), json!(4));
                    app.start_benchmark_now();
                }
                _ => {}
            }
            Phase::WaitResult
        }
        Phase::WaitResult if demo == "coverage" && app.testing.coverage.is_some() && app.testing.covering.is_none() => {
            // Replay the scenario that reached the faulty branch.
            let target = app.testing.coverage_targets.iter()
                .find(|t| t["label"] == "low-battery-boost:true" && t["scenario"].is_array())
                .or_else(|| app.testing.coverage_targets.iter().find(|t| t["scenario"].is_array()))
                .cloned();
            match target {
                Some(t) => app.replay_covering(&t),
                None => app.testing.covering = Some(String::new()),
            }
            Phase::WaitResult
        }
        Phase::WaitResult if result_ready(app, &demo) || timeout => {
            for replay in [&mut app.lab.replay, &mut app.testing.replay, &mut app.gp.replay] {
                if replay.trajectory.is_some() {
                    replay.time = replay.duration() * 0.62;
                    replay.playing = false;
                }
            }
            if demo == "testing" && !app.testing.failures.is_empty() {
                app.testing.tab = crate::pages::testing::TestTab::Failures;
            }
            if demo == "pareto" {
                app.lab.tab = crate::pages::lab::BottomTab::Pareto;
            }
            if demo == "novelty" {
                app.lab.tab = crate::pages::lab::BottomTab::Progress;
                app.lab.series_choice = "behaviourCoverage".into();
            }
            if demo == "coverage" {
                app.testing.show_collateral = false;
            }
            if demo == "gp" {
                app.gp.tab = crate::pages::gp::GpTab::Generalisation;
                if let Some(i) = app.gp.episodes.iter().position(|e| e["split"] == "test" && e["outcome"] == "REACHED_GOAL") {
                    app.gp.select(i);
                }
            }
            if demo == "repair" || demo == "improve" {
                app.repair.tab = crate::pages::repair::RepairTab::Tests;
            }
            if demo.starts_with("arm") {
                if let Some(m) = &app.arm.motion {
                    app.arm.time = m.duration() * 0.45;
                    app.arm.playing = false;
                }
            }
            if demo == "fleet" {
                // Show the moment of the first conflict if there is one.
                if let Some(c) = app.lab.conflicts.first() {
                    app.lab.replay.time = c.start;
                }
            }
            Phase::Settle(0)
        }
        Phase::Settle(n) if n >= 30 => Phase::Shoot,
        Phase::Settle(n) => Phase::Settle(n + 1),
        Phase::Shoot => {
            ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::UserData::default()));
            Phase::WaitShot
        }
        Phase::WaitShot => {
            let image = ctx.input(|i| {
                i.raw.events.iter().find_map(|e| match e {
                    egui::Event::Screenshot { image, .. } => Some(image.clone()),
                    _ => None,
                })
            });
            match image {
                Some(image) => {
                    let mut rgba = Vec::with_capacity(image.pixels.len() * 4);
                    for p in &image.pixels {
                        rgba.extend_from_slice(&p.to_srgba_unmultiplied());
                    }
                    let [w, h] = image.size;
                    match image::save_buffer(&script.out, &rgba, w as u32, h as u32, image::ColorType::Rgba8) {
                        Ok(_) => println!("captured {}", script.out.display()),
                        Err(e) => eprintln!("capture failed: {e}"),
                    }
                    ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    Phase::Done
                }
                None if timeout => {
                    ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    Phase::Done
                }
                None => Phase::WaitShot,
            }
        }
        other => other,
    };
    ctx.request_repaint();
    app.capture = Some(script);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_capture_options() {
        let args = ["--page", "lab", "--demo", "lab", "--capture", "x.png"].iter().map(|s| s.to_string());
        let parsed = CliArgs::parse(args);
        assert_eq!(parsed.page.as_deref(), Some("lab"));
        assert_eq!(parsed.demo.as_deref(), Some("lab"));
        assert_eq!(parsed.capture, Some(PathBuf::from("x.png")));
        assert_eq!(CliArgs::parse(std::iter::empty()), CliArgs::default());
    }
}
