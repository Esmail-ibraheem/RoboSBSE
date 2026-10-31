//! Data types shared with the engine (map format, trajectories, capability descriptions).
//! Field names follow the engine's JSON (camelCase); unknown fields are ignored so that newer
//! engines can add information without breaking the UI.

use std::collections::BTreeMap;

use serde::{Deserialize, Deserializer, Serialize};
use serde_json::Value;

/// Obstacle geometry, tagged like the engine's `Shape` (`{"type":"circle",...}`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum Shape {
    Circle { x: f64, y: f64, r: f64 },
    Rect { x: f64, y: f64, w: f64, h: f64 },
}

impl Shape {
    /// Signed distance from a point to the shape boundary (negative inside).
    pub fn signed_distance(&self, px: f64, py: f64) -> f64 {
        match *self {
            Shape::Circle { x, y, r } => (px - x).hypot(py - y) - r,
            Shape::Rect { x, y, w, h } => {
                let qx = (px - (x + w / 2.0)).abs() - w / 2.0;
                let qy = (py - (y + h / 2.0)).abs() - h / 2.0;
                qx.max(0.0).hypot(qy.max(0.0)) + qx.max(qy).min(0.0)
            }
        }
    }

    pub fn contains(&self, px: f64, py: f64) -> bool {
        self.signed_distance(px, py) <= 0.0
    }

    pub fn translate(&mut self, dx: f64, dy: f64) {
        match self {
            Shape::Circle { x, y, .. } | Shape::Rect { x, y, .. } => {
                *x += dx;
                *y += dy;
            }
        }
    }

    pub fn describe(&self) -> String {
        match *self {
            Shape::Circle { x, y, r } => format!("circle ({x:.2}, {y:.2}) r={r:.2}"),
            Shape::Rect { x, y, w, h } => format!("rect ({x:.2}, {y:.2}) {w:.2}x{h:.2}"),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    #[serde(default)]
    pub theta: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

fn default_schema() -> String {
    "robosbse.map/1".into()
}

fn default_tolerance() -> f64 {
    0.3
}

fn default_mode() -> String {
    "bounce".into()
}

fn default_zone_kind() -> String {
    "restricted".into()
}

/// A circular obstacle moving along a polyline at constant speed (engine `MovingObstacle`); its
/// position is a pure function of time.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MovingObstacle {
    pub radius: f64,
    pub path: Vec<Point>,
    #[serde(default)]
    pub speed: f64,
    /// `bounce` (back and forth) or `loop` (around the closed polygon).
    #[serde(default = "default_mode")]
    pub mode: String,
    #[serde(default)]
    pub offset: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
}

impl MovingObstacle {
    fn looped(&self) -> bool {
        self.mode == "loop"
    }

    /// Length of one traversal: the open polyline (bounce) or the closed polygon (loop).
    pub fn length(&self) -> f64 {
        let p = &self.path;
        let mut l: f64 = p.windows(2).map(|w| (w[1].x - w[0].x).hypot(w[1].y - w[0].y)).sum();
        if self.looped() && p.len() > 1 {
            l += (p[0].x - p[p.len() - 1].x).hypot(p[0].y - p[p.len() - 1].y);
        }
        l
    }

    /// Centre at time `t`, computed exactly like the engine.
    pub fn position(&self, t: f64) -> (f64, f64) {
        let length = self.length();
        if self.path.len() == 1 || length <= 0.0 || self.speed == 0.0 {
            return self.at(self.offset);
        }
        let s = self.speed * t + self.offset;
        if self.looped() {
            self.at(s.rem_euclid(length))
        } else {
            let s = s.rem_euclid(2.0 * length);
            self.at(if s > length { 2.0 * length - s } else { s })
        }
    }

    fn at(&self, distance: f64) -> (f64, f64) {
        let n = self.path.len();
        if n == 0 {
            return (f64::NAN, f64::NAN);
        }
        let mut remaining = distance.max(0.0);
        let segments = if self.looped() { n } else { n - 1 };
        for i in 0..segments {
            let (a, b) = (&self.path[i], &self.path[(i + 1) % n]);
            let l = (b.x - a.x).hypot(b.y - a.y);
            if remaining <= l || i == segments - 1 {
                let f = if l <= 0.0 { 0.0 } else { (remaining / l).min(1.0) };
                return (a.x + f * (b.x - a.x), a.y + f * (b.y - a.y));
            }
            remaining -= l;
        }
        (self.path[0].x, self.path[0].y)
    }
}

/// An obstacle present only during `[from, until)` (engine `TimedObstacle`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TimedObstacle {
    pub shape: Shape,
    #[serde(default)]
    pub from: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub until: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
}

impl TimedObstacle {
    pub fn active(&self, t: f64) -> bool {
        t >= self.from && self.until.map_or(true, |u| t < u)
    }
}

/// A restricted area or rough terrain (engine `Zone`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Zone {
    #[serde(default = "default_zone_kind")]
    pub kind: String,
    pub shape: Shape,
    #[serde(default)]
    pub speed_factor: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
}

/// Environment in the `robosbse.map/1` format.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct WorldMap {
    #[serde(default = "default_schema")]
    pub schema: String,
    pub name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    pub width: f64,
    pub height: f64,
    pub start: Pose,
    pub goal: Point,
    #[serde(default = "default_tolerance")]
    pub goal_tolerance: f64,
    #[serde(default)]
    pub obstacles: Vec<Shape>,
    /// Dynamic elements; the Environment Editor keeps them unchanged.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub moving_obstacles: Vec<MovingObstacle>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub timed_obstacles: Vec<TimedObstacle>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub zones: Vec<Zone>,
}

impl WorldMap {
    pub fn empty(name: &str, width: f64, height: f64) -> Self {
        WorldMap {
            schema: default_schema(),
            name: name.into(),
            description: Some("Created in the RoboSBSE editor.".into()),
            width,
            height,
            start: Pose { x: 1.5, y: 1.5, theta: 0.0 },
            goal: Point { x: width - 1.5, y: height - 1.5 },
            goal_tolerance: 0.3,
            obstacles: vec![],
            moving_obstacles: vec![],
            timed_obstacles: vec![],
            zones: vec![],
        }
    }

    pub fn is_dynamic(&self) -> bool {
        !self.moving_obstacles.is_empty() || !self.timed_obstacles.is_empty()
    }

    pub fn to_json(&self) -> Value {
        serde_json::to_value(self).unwrap_or(Value::Null)
    }

    /// Index of the top-most obstacle containing the point.
    pub fn obstacle_at(&self, x: f64, y: f64) -> Option<usize> {
        self.obstacles.iter().rposition(|s| s.contains(x, y))
    }
}

fn nullable_f64s<'de, D: Deserializer<'de>>(d: D) -> Result<Vec<f64>, D::Error> {
    let v: Vec<Option<f64>> = Vec::deserialize(d)?;
    Ok(v.into_iter().map(|x| x.unwrap_or(f64::NAN)).collect())
}

fn nullable_rays<'de, D: Deserializer<'de>>(d: D) -> Result<Option<Vec<Option<Vec<f32>>>>, D::Error> {
    let v: Option<Vec<Option<Vec<Option<f32>>>>> = Option::deserialize(d)?;
    Ok(v.map(|frames| {
        frames
            .into_iter()
            .map(|f| f.map(|r| r.into_iter().map(|x| x.unwrap_or(f32::NAN)).collect()))
            .collect()
    }))
}

/// Column-oriented recorded episode (see the engine's `Trajectory`). Mirrors the full format even
/// where the UI does not display a column yet.
#[allow(dead_code)]
#[derive(Clone, Debug, Default, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Trajectory {
    #[serde(default)]
    pub dt: f64,
    #[serde(deserialize_with = "nullable_f64s", default)]
    pub t: Vec<f64>,
    #[serde(deserialize_with = "nullable_f64s", default)]
    pub x: Vec<f64>,
    #[serde(deserialize_with = "nullable_f64s", default)]
    pub y: Vec<f64>,
    #[serde(deserialize_with = "nullable_f64s", default)]
    pub theta: Vec<f64>,
    #[serde(deserialize_with = "nullable_f64s", default)]
    pub v: Vec<f64>,
    #[serde(deserialize_with = "nullable_f64s", default)]
    pub omega: Vec<f64>,
    #[serde(deserialize_with = "nullable_f64s", default)]
    pub battery: Vec<f64>,
    #[serde(deserialize_with = "nullable_f64s", default)]
    pub clearance: Vec<f64>,
    #[serde(default)]
    pub scan_valid: Vec<bool>,
    #[serde(deserialize_with = "nullable_rays", default)]
    pub rays: Option<Vec<Option<Vec<f32>>>>,
}

/// Interpolated robot state at a replay time.
#[derive(Clone, Copy, Debug)]
pub struct Frame {
    pub index: usize,
    pub x: f64,
    pub y: f64,
    pub theta: f64,
    pub v: f64,
    pub battery: f64,
    pub clearance: f64,
    pub scan_valid: bool,
}

impl Trajectory {
    pub fn len(&self) -> usize {
        self.x.len().min(self.y.len()).min(self.t.len())
    }

    pub fn duration(&self) -> f64 {
        self.t.last().copied().unwrap_or(0.0)
    }

    pub fn frame_at(&self, time: f64) -> Option<Frame> {
        let n = self.len();
        if n == 0 {
            return None;
        }
        let i = match self.t.binary_search_by(|t| t.partial_cmp(&time).unwrap_or(std::cmp::Ordering::Less)) {
            Ok(i) => i,
            Err(i) => i.saturating_sub(1),
        }
        .min(n - 1);
        let j = (i + 1).min(n - 1);
        let span = self.t[j] - self.t[i];
        let a = if span > 0.0 { ((time - self.t[i]) / span).clamp(0.0, 1.0) } else { 0.0 };
        let lerp = |v: &Vec<f64>| -> f64 {
            match (v.get(i), v.get(j)) {
                (Some(p), Some(q)) => p + (q - p) * a,
                (Some(p), None) => *p,
                _ => f64::NAN,
            }
        };
        let th0 = self.theta.get(i).copied().unwrap_or(0.0);
        let th1 = self.theta.get(j).copied().unwrap_or(th0);
        let mut d = th1 - th0;
        while d > std::f64::consts::PI {
            d -= 2.0 * std::f64::consts::PI;
        }
        while d < -std::f64::consts::PI {
            d += 2.0 * std::f64::consts::PI;
        }
        Some(Frame {
            index: i,
            x: lerp(&self.x),
            y: lerp(&self.y),
            theta: th0 + d * a,
            v: lerp(&self.v),
            battery: lerp(&self.battery),
            clearance: self.clearance.get(i).copied().unwrap_or(f64::NAN),
            scan_valid: self.scan_valid.get(i).copied().unwrap_or(true),
        })
    }

    pub fn rays_at(&self, index: usize) -> Option<&Vec<f32>> {
        self.rays.as_ref().and_then(|r| r.get(index)).and_then(|f| f.as_ref())
    }
}

/// Declared parameter of an algorithm, problem or controller.
#[derive(Clone, Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ParamSpec {
    pub name: String,
    #[serde(rename = "type")]
    pub kind: String,
    #[serde(default)]
    pub default_value: Value,
    #[serde(default)]
    pub min: Option<f64>,
    #[serde(default)]
    pub max: Option<f64>,
    #[serde(default)]
    pub options: Vec<String>,
    #[serde(default)]
    pub description: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AlgorithmInfo {
    pub id: String,
    pub name: String,
    pub family: String,
    pub summary: String,
    #[serde(default)]
    pub params: Vec<ParamSpec>,
    #[serde(default)]
    pub representations: Vec<String>,
    #[serde(default)]
    pub multi_objective: bool,
}

impl AlgorithmInfo {
    /// Whether the algorithm can run on the problem (representation and objectives).
    pub fn supports(&self, problem: &ProblemInfo) -> bool {
        let representation_ok = self.representations.is_empty()
            || self.representations.iter().any(|r| r == "any" || *r == problem.representation);
        representation_ok && (!self.multi_objective || !problem.objectives.is_empty())
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ProblemInfo {
    pub id: String,
    pub name: String,
    pub summary: String,
    #[serde(default)]
    pub default_map: String,
    #[serde(default)]
    pub params: Vec<ParamSpec>,
    #[serde(default)]
    pub representation: String,
    /// Operator names per kind (mutation, crossover, neighbourhood) valid for this problem.
    #[serde(default)]
    pub operators: BTreeMap<String, Vec<String>>,
    /// Objective names; empty for single-objective problems.
    #[serde(default)]
    pub objectives: Vec<String>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ControllerInfo {
    pub id: String,
    pub name: String,
    pub kind: String,
    pub summary: String,
    #[serde(default)]
    pub faulty_of: Option<String>,
    #[serde(default)]
    pub defect: Option<String>,
    #[serde(default)]
    pub expected_property: Option<String>,
    /// Instrumented decisions (empty when the controller has no coverage instrumentation).
    #[serde(default)]
    pub branches: Vec<BranchInfo>,
    #[serde(default)]
    pub instrumentation: Option<String>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct BranchInfo {
    pub id: String,
    pub description: String,
    #[serde(default)]
    pub parent: Option<String>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct PropertyInfo {
    pub id: String,
    pub safety: bool,
    pub description: String,
}

/// The engine's self-description (`ready` / `hello` message).
#[derive(Clone, Debug, Default, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Capabilities {
    #[serde(default)]
    pub engine: String,
    #[serde(default)]
    pub version: String,
    #[serde(default)]
    pub protocol: i64,
    #[serde(default)]
    pub java: String,
    #[serde(default)]
    pub algorithms: Vec<AlgorithmInfo>,
    #[serde(default)]
    pub problems: Vec<ProblemInfo>,
    #[serde(default)]
    pub controllers: Vec<ControllerInfo>,
    #[serde(default)]
    pub properties: Vec<PropertyInfo>,
    #[serde(default)]
    pub maps: Vec<String>,
    #[serde(default)]
    pub task_sets: Vec<String>,
    /// Program-repair and genetic-improvement subjects.
    #[serde(default)]
    pub subjects: Vec<SubjectInfo>,
    /// Built-in robotic-arm scenes (full `robosbse.arm/1` objects).
    #[serde(default)]
    pub arm_scenes: Vec<Value>,
    /// Built-in multi-robot fleet instances.
    #[serde(default)]
    pub fleets: Vec<String>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SubjectInfo {
    pub id: String,
    pub name: String,
    pub purpose: String,
    #[serde(default)]
    pub defect_kind: Option<String>,
    #[serde(default)]
    pub defect: Option<String>,
    #[serde(default)]
    pub description: String,
}

impl Capabilities {
    pub fn algorithm(&self, id: &str) -> Option<&AlgorithmInfo> {
        self.algorithms.iter().find(|a| a.id == id)
    }

    pub fn problem(&self, id: &str) -> Option<&ProblemInfo> {
        self.problems.iter().find(|p| p.id == id)
    }

    pub fn controller(&self, id: &str) -> Option<&ControllerInfo> {
        self.controllers.iter().find(|c| c.id == id)
    }

    /// Algorithms applicable to a problem, in catalogue order.
    pub fn algorithms_for(&self, problem: &str) -> Vec<&AlgorithmInfo> {
        match self.problem(problem) {
            Some(p) => self.algorithms.iter().filter(|a| a.supports(p)).collect(),
            None => self.algorithms.iter().collect(),
        }
    }
}

/// Reads `[[x, y], ...]` arrays.
pub fn points(value: &Value) -> Vec<[f64; 2]> {
    value
        .as_array()
        .map(|a| {
            a.iter()
                .filter_map(|p| Some([p.get(0)?.as_f64()?, p.get(1)?.as_f64()?]))
                .collect()
        })
        .unwrap_or_default()
}

pub fn num(value: &Value) -> f64 {
    value.as_f64().unwrap_or(f64::NAN)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn map_round_trips_with_engine_format() {
        let text = r#"{"schema":"robosbse.map/1","name":"m","width":10,"height":5,
            "start":{"x":1,"y":1,"theta":0.5},"goal":{"x":9,"y":4},"goalTolerance":0.3,
            "obstacles":[{"type":"circle","x":3,"y":2,"r":0.5},{"type":"rect","x":5,"y":1,"w":1,"h":2}],
            "futureField":42}"#;
        let map: WorldMap = serde_json::from_str(text).unwrap();
        assert_eq!(map.obstacles.len(), 2);
        assert_eq!(map.obstacles[1], Shape::Rect { x: 5.0, y: 1.0, w: 1.0, h: 2.0 });
        let back = map.to_json();
        assert_eq!(back["obstacles"][0]["type"], "circle");
        let again: WorldMap = serde_json::from_value(back).unwrap();
        assert_eq!(map, again);
    }

    #[test]
    fn dynamic_map_elements_round_trip_and_move_like_the_engine() {
        let text = r#"{"name":"d","width":10,"height":10,"start":{"x":1,"y":1},"goal":{"x":9,"y":9},
            "movingObstacles":[{"radius":0.5,"path":[{"x":0,"y":0},{"x":4,"y":0}],"speed":1,"mode":"bounce"},
                               {"radius":0.4,"path":[{"x":0,"y":0},{"x":2,"y":0},{"x":2,"y":2}],"speed":1,"mode":"loop","offset":1}],
            "timedObstacles":[{"shape":{"type":"rect","x":1,"y":1,"w":1,"h":1},"from":15,"until":40}],
            "zones":[{"kind":"terrain","shape":{"type":"circle","x":5,"y":5,"r":1},"speedFactor":0.5,"label":"gravel"}]}"#;
        let map: WorldMap = serde_json::from_str(text).unwrap();
        assert!(map.is_dynamic());
        let bounce = &map.moving_obstacles[0];
        assert_eq!(bounce.position(3.0), (3.0, 0.0));
        assert_eq!(bounce.position(5.0), (3.0, 0.0), "bounces back after the end of the path");
        let looped = &map.moving_obstacles[1];
        // Closed polygon of length 2 + 2 + 2.83; offset 1 m, so after 2 s it is 1 m up the second edge.
        let (x, y) = looped.position(2.0);
        assert!((x - 2.0).abs() < 1e-12 && (y - 1.0).abs() < 1e-12, "({x}, {y})");
        assert!(!map.timed_obstacles[0].active(14.9) && map.timed_obstacles[0].active(15.0));
        assert!(!map.timed_obstacles[0].active(40.0));
        let again: WorldMap = serde_json::from_value(map.to_json()).unwrap();
        assert_eq!(map, again);
        assert!(WorldMap::empty("e", 5.0, 5.0).to_json().get("zones").is_none(), "static maps keep the old format");
    }

    #[test]
    fn signed_distance_matches_engine_definition() {
        let r = Shape::Rect { x: 2.0, y: 2.0, w: 4.0, h: 2.0 };
        assert!((r.signed_distance(1.0, 3.0) - 1.0).abs() < 1e-12);
        assert!((r.signed_distance(4.0, 2.5) + 0.5).abs() < 1e-12);
        let c = Shape::Circle { x: 5.0, y: 5.0, r: 1.0 };
        assert!((c.signed_distance(8.0, 5.0) - 2.0).abs() < 1e-12);
        let map = WorldMap { obstacles: vec![r, c], ..WorldMap::empty("t", 10.0, 10.0) };
        assert_eq!(map.obstacle_at(5.0, 5.2), Some(1));
        assert_eq!(map.obstacle_at(0.5, 0.5), None);
    }

    #[test]
    fn trajectory_tolerates_nulls_and_interpolates() {
        let traj: Trajectory = serde_json::from_value(json!({
            "dt": 0.1, "t": [0.0, 0.1, 0.2], "x": [0.0, 1.0, 2.0], "y": [0.0, 0.0, null],
            "theta": [3.1, -3.1, -3.1], "v": [0, 1, 1], "omega": [0, 0, 0], "battery": [1, 1, 1],
            "clearance": [null, 0.5, 0.4], "scanValid": [true, false, true],
            "rays": [[1.0, null], null, [2.0, 2.0]]
        }))
        .unwrap();
        let f = traj.frame_at(0.05).unwrap();
        assert!((f.x - 0.5).abs() < 1e-9);
        // Heading interpolation takes the short way across +-pi.
        assert!(f.theta.abs() > 3.1);
        assert!(traj.y[2].is_nan());
        assert!(traj.rays_at(1).is_none());
        assert!(traj.rays_at(0).unwrap()[1].is_nan());
        assert_eq!(traj.frame_at(99.0).unwrap().index, 2);
    }

    #[test]
    fn algorithms_are_filtered_by_representation_and_objectives() {
        let caps: Capabilities = serde_json::from_value(json!({
            "algorithms": [
                {"id": "ga", "name": "GA", "family": "e", "summary": "", "representations": ["any"]},
                {"id": "pso", "name": "PSO", "family": "s", "summary": "", "representations": ["real-vector"]},
                {"id": "nsga2", "name": "NSGA-II", "family": "m", "summary": "", "representations": ["any"],
                 "multiObjective": true}],
            "problems": [
                {"id": "sched", "name": "S", "summary": "", "representation": "permutation",
                 "objectives": ["makespan", "tardiness"]},
                {"id": "test", "name": "T", "summary": "", "representation": "real-vector", "objectives": []}]
        }))
        .unwrap();
        let ids = |p: &str| caps.algorithms_for(p).iter().map(|a| a.id.clone()).collect::<Vec<_>>();
        assert_eq!(ids("sched"), vec!["ga", "nsga2"]);
        assert_eq!(ids("test"), vec!["ga", "pso"]);
    }

    #[test]
    fn capabilities_parse_engine_param_specs() {
        let caps: Capabilities = serde_json::from_value(json!({
            "engine": "robosbse-engine", "version": "0.1.0", "protocol": 1,
            "algorithms": [{"id": "hill-climbing", "name": "Hill Climbing", "family": "local", "summary": "s",
                "params": [{"name": "variant", "type": "CHOICE", "defaultValue": "first", "min": null, "max": null,
                            "options": ["first", "best"], "description": "d"}], "requires": ["neighbourhood"]}],
            "controllers": [{"id": "reactive-bug1-dropout", "name": "n", "kind": "reactive", "summary": "s",
                "faultyOf": "reactive", "defect": "d", "expectedProperty": "collision", "params": []}]
        }))
        .unwrap();
        assert_eq!(caps.algorithm("hill-climbing").unwrap().params[0].options.len(), 2);
        assert_eq!(caps.controller("reactive-bug1-dropout").unwrap().faulty_of.as_deref(), Some("reactive"));
    }
}
