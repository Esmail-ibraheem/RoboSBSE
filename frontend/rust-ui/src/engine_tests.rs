//! End-to-end tests of the UI's engine client against the real Java engine (the full vertical
//! slice without a window). They need the engine jar (`gradlew engineJar`) and Java 21 via
//! `ROBOSBSE_JAVA` or `JAVA_HOME`; set `ROBOSBSE_SKIP_ENGINE_TESTS=1` to skip them.

use std::time::{Duration, Instant};

use eframe::egui;
use serde_json::{json, Value};

use crate::engine::{self, Engine, EngineEvent, EngineStatus};
use crate::model::{Capabilities, Trajectory};
use crate::settings::find_repo_root;

fn engine_or_skip() -> Option<Engine> {
    if std::env::var("ROBOSBSE_SKIP_ENGINE_TESTS").is_ok() {
        eprintln!("engine tests skipped by ROBOSBSE_SKIP_ENGINE_TESTS");
        return None;
    }
    let root = find_repo_root();
    let jar = root.join("backend/java-engine/build/libs/robosbse-engine.jar");
    assert!(jar.exists(), "build the engine first: {}", jar.display());
    let mut engine = Engine::spawn(&engine::default_java(), &jar, &root, egui::Context::default());
    let deadline = Instant::now() + Duration::from_secs(60);
    while Instant::now() < deadline {
        engine.poll();
        match &engine.status {
            EngineStatus::Ready => return Some(engine),
            EngineStatus::Failed(e) => panic!("engine failed to start: {e}"),
            _ => std::thread::sleep(Duration::from_millis(20)),
        }
    }
    panic!("engine did not become ready");
}

fn wait_for(engine: &mut Engine, id: &str, kind: &str, seen: &mut Vec<Value>) -> Value {
    let deadline = Instant::now() + Duration::from_secs(120);
    while Instant::now() < deadline {
        for event in engine.poll() {
            if let EngineEvent::Message(m) = event {
                let done = m["id"] == id && m["type"] == kind;
                if m["type"] == "error" && m["id"] == id {
                    panic!("engine error: {m}");
                }
                seen.push(m.clone());
                if done {
                    return m;
                }
            }
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    panic!("timed out waiting for {kind} {id}");
}

#[test]
fn ui_client_runs_a_real_experiment_and_reproduces_a_saved_failure() {
    let Some(mut engine) = engine_or_skip() else { return };
    let mut seen = Vec::new();

    let hello = engine.request("hello", json!({})).unwrap();
    let caps: Capabilities = serde_json::from_value(wait_for(&mut engine, &hello, "hello", &mut seen)).unwrap();
    assert!(caps.algorithm("genetic-algorithm").is_some());
    assert!(caps.controllers.iter().filter(|c| c.faulty_of.is_some()).count() >= 4);

    let experiment = json!({
        "name": "ui-e2e", "seed": 5, "budget": {"maxEvaluations": 300},
        "problem": {"type": "navigation", "map": "builtin:warehouse", "params": {"waypoints": 3}},
        "algorithm": {"type": "genetic-algorithm", "params": {"populationSize": 20}}
    });
    let id = engine.request("start", json!({"experiment": experiment})).unwrap();
    let finished = wait_for(&mut engine, &id, "finished", &mut seen);
    let result = &finished["result"];
    assert_eq!(result["evaluations"], 300);
    assert!(result["best"]["fitness"].as_f64().unwrap().is_finite());
    assert!(seen.iter().any(|m| m["type"] == "progress" && m["id"] == id.as_str()));
    let trajectory: Trajectory = serde_json::from_value(result["replay"]["trajectory"].clone()).unwrap();
    assert!(trajectory.len() > 10, "the best route must be replayable");
    assert!(trajectory.frame_at(trajectory.duration() / 2.0).is_some());

    let record: Value = serde_json::from_str(
        &std::fs::read_to_string(find_repo_root().join("examples/faulty-controllers/regressions/bug1-dropout-collision.json"))
            .unwrap(),
    )
    .unwrap();
    let rep = engine.request("reproduce", json!({"failure": record})).unwrap();
    let reproduction = wait_for(&mut engine, &rep, "reproduction", &mut seen);
    assert_eq!(reproduction["reproduced"], true);
    assert_eq!(reproduction["referenceViolated"], json!([]));
    let replay: Trajectory = serde_json::from_value(reproduction["trajectory"].clone()).unwrap();
    assert!(replay.rays.is_some(), "testing replays carry sensor rays");
    engine.shutdown();
    assert_eq!(engine.status, EngineStatus::Stopped);
}

#[test]
fn ui_client_runs_a_coverage_campaign_and_replays_a_covering_scenario() {
    let Some(mut engine) = engine_or_skip() else { return };
    let mut seen = Vec::new();
    let hello = engine.request("hello", json!({})).unwrap();
    let caps: Capabilities = serde_json::from_value(wait_for(&mut engine, &hello, "hello", &mut seen)).unwrap();
    let bug3 = caps.controller("reactive-bug3-boost").expect("bug3 controller");
    assert!(bug3.branches.iter().any(|b| b.id == "low-battery-boost"));
    assert!(caps.controller("waypoint-follower").unwrap().branches.is_empty(), "no instrumentation, no coverage");

    let problem = json!({"type": "controller-testing", "map": "builtin:testing-arena",
        "params": {"controller": "reactive", "maxSteps": 300}});
    let campaign = json!({"name": "ui-coverage", "seed": 3, "budget": {"maxEvaluations": 60},
        "problem": problem, "algorithm": {"type": "random-search"}, "randomBaseline": false, "threads": 2});
    let id = engine.request("coverage", json!({"campaign": campaign})).unwrap();
    let finished = wait_for(&mut engine, &id, "coverage_finished", &mut seen);
    let result = &finished["result"];
    let targets = result["targets"].as_array().unwrap();
    assert_eq!(targets.len() as u64, result["summary"]["outcomes"].as_u64().unwrap());
    let progress = seen.iter().filter(|m| m["type"] == "coverage_progress" && m["id"] == id.as_str()).count();
    assert_eq!(progress, targets.len(), "one progress event per target");

    let covered = targets.iter().find(|t| t["scenario"].is_array()).expect("some outcome is covered");
    let sim = engine.request("simulate", json!({"problem": result["problem"], "genome": covered["scenario"]})).unwrap();
    let reply = wait_for(&mut engine, &sim, "simulation", &mut seen);
    let outcomes: Vec<String> = serde_json::from_value(reply["replay"]["coveredOutcomes"].clone()).unwrap();
    assert!(outcomes.contains(&covered["label"].as_str().unwrap().to_string()),
        "the replayed scenario covers its target {}", covered["label"]);
    engine.shutdown();
}

#[test]
fn ui_client_evolves_a_program_and_knows_the_repair_subjects() {
    let Some(mut engine) = engine_or_skip() else { return };
    let mut seen = Vec::new();
    let hello = engine.request("hello", json!({})).unwrap();
    let caps: Capabilities = serde_json::from_value(wait_for(&mut engine, &hello, "hello", &mut seen)).unwrap();
    assert!(caps.subjects.iter().filter(|s| s.purpose == "program-repair").count() >= 5);
    assert!(caps.subjects.iter().all(|s| s.purpose != "program-repair" || s.defect.is_some()));
    assert_eq!(caps.problem("program-repair").unwrap().representation, "patch");
    assert!(caps.algorithms_for("controller-synthesis").iter().any(|a| a.id == "genetic-algorithm"));
    assert!(!caps.algorithms_for("controller-synthesis").iter().any(|a| a.id == "particle-swarm"),
        "PSO needs real vectors, not programs");

    let mut gp = crate::pages::gp::GpState::new();
    gp.on_capabilities(&caps);
    gp.problem.insert("maps".into(), json!("empty-field"));
    gp.problem.insert("testMaps".into(), json!("obstacle-course"));
    gp.budget = 80;
    gp.threads = 2;
    let id = engine.request("start", json!({"experiment": gp.experiment(&caps)})).unwrap();
    let finished = wait_for(&mut engine, &id, "finished", &mut seen);
    gp.on_finished(&finished["result"]);
    assert_eq!(gp.episodes.len(), 2, "one training and one test episode");
    assert!(gp.replay.trajectory.is_some());
    assert!(gp.result.as_ref().unwrap()["best"]["solution"]["pretty"].is_string());

    let mut repair = crate::pages::repair::RepairState::new();
    repair.on_capabilities(&caps);
    let cfg = repair.experiment(&caps);
    assert_eq!(cfg["problem"]["type"], "program-repair");
    assert_eq!(cfg["problem"]["params"]["workers"], json!(repair.threads));
    engine.shutdown();
}

#[test]
fn missing_jar_is_reported_not_panicked() {
    let engine = Engine::spawn("java", std::path::Path::new("/nonexistent/robosbse-engine.jar"),
        &find_repo_root(), egui::Context::default());
    assert!(matches!(engine.status, EngineStatus::Failed(ref m) if m.contains("not found")));
}
