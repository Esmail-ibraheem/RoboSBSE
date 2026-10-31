//! Environment Editor: create and modify maps (obstacles, start pose, goal), generate maps
//! procedurally, check solvability with A*, and save/load the shared map format. The edited map is
//! the Laboratory's map, so it can be optimised immediately.

use eframe::egui::{self, RichText};
use serde_json::json;

use crate::app::{MapTarget, RoboApp};
use crate::canvas::{self, Editor, Overlay, Tool};
use crate::model::{self, Shape, WorldMap};

pub struct EditorPage {
    pub editor: Editor,
    pub gen_kind: String,
    pub gen_seed: u64,
    pub gen_count: u32,
    pub new_width: f64,
    pub new_height: f64,
}

impl Default for EditorPage {
    fn default() -> Self {
        EditorPage {
            editor: Editor::default(),
            gen_kind: "scatter".into(),
            gen_seed: 1,
            gen_count: 15,
            new_width: 20.0,
            new_height: 20.0,
        }
    }
}

/// Problems that would make a map unusable (reported, not silently fixed).
pub fn validate(map: &WorldMap) -> Vec<String> {
    let mut issues = Vec::new();
    let r = 0.25;
    let inside = |x: f64, y: f64| x >= r && y >= r && x <= map.width - r && y <= map.height - r;
    if !inside(map.start.x, map.start.y) {
        issues.push("the robot start is outside the world".into());
    }
    if !inside(map.goal.x, map.goal.y) {
        issues.push("the goal is outside the world".into());
    }
    for (i, s) in map.obstacles.iter().enumerate() {
        if s.signed_distance(map.start.x, map.start.y) < r {
            issues.push(format!("obstacle {} overlaps the robot start", i + 1));
        }
        if s.signed_distance(map.goal.x, map.goal.y) < 0.0 {
            issues.push(format!("obstacle {} covers the goal", i + 1));
        }
    }
    issues
}

impl RoboApp {
    fn save_map_as(&mut self) {
        let dir = std::path::PathBuf::from(&self.settings.maps_dir);
        if let Some(path) = rfd::FileDialog::new()
            .set_directory(&dir)
            .set_file_name(format!("{}.json", self.lab.map.name))
            .add_filter("map", &["json"])
            .save_file()
        {
            match std::fs::write(&path, serde_json::to_string_pretty(&self.lab.map.to_json()).unwrap_or_default()) {
                Ok(_) => {
                    self.lab.map_source = path.display().to_string();
                    self.notify(format!("saved map {}", path.display()));
                }
                Err(e) => self.error(format!("cannot save map: {e}")),
            }
        }
    }

    fn open_map(&mut self) {
        let dir = std::path::PathBuf::from(&self.settings.maps_dir);
        if let Some(path) = rfd::FileDialog::new().set_directory(&dir).add_filter("map", &["json"]).pick_file() {
            match crate::app::load_map_file(&path) {
                Some(map) => {
                    self.lab.set_map(map, &path.display().to_string());
                    self.notify(format!("loaded map {}", path.display()));
                }
                None => self.error(format!("{} is not a robosbse.map/1 file", path.display())),
            }
        }
    }

    fn editor_panel(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("Tools");
            for tool in Tool::ALL {
                ui.radio_value(&mut self.editor.editor.tool, tool, tool.label());
            }
            ui.label(RichText::new(self.editor.editor.tool.hint()).small().weak());
            ui.checkbox(&mut self.editor.editor.snap, "snap to 0.1 m");
            ui.label(RichText::new("Right-drag pans, scroll zooms.").small().weak());

            ui.separator();
            ui.heading("Selection");
            match self.editor.editor.selected.filter(|i| *i < self.lab.map.obstacles.len()) {
                Some(i) => {
                    ui.label(RichText::new(format!("#{}: {}", i + 1, self.lab.map.obstacles[i].describe())).monospace().small());
                    let shape = &mut self.lab.map.obstacles[i];
                    egui::Grid::new("sel").num_columns(2).show(ui, |ui| match shape {
                        Shape::Circle { x, y, r } => {
                            ui.label("centre x");
                            ui.add(egui::DragValue::new(x).speed(0.05));
                            ui.end_row();
                            ui.label("centre y");
                            ui.add(egui::DragValue::new(y).speed(0.05));
                            ui.end_row();
                            ui.label("radius");
                            ui.add(egui::DragValue::new(r).speed(0.02).range(0.05..=100.0));
                            ui.end_row();
                        }
                        Shape::Rect { x, y, w, h } => {
                            ui.label("x");
                            ui.add(egui::DragValue::new(x).speed(0.05));
                            ui.end_row();
                            ui.label("y");
                            ui.add(egui::DragValue::new(y).speed(0.05));
                            ui.end_row();
                            ui.label("width");
                            ui.add(egui::DragValue::new(w).speed(0.05).range(0.05..=1000.0));
                            ui.end_row();
                            ui.label("height");
                            ui.add(egui::DragValue::new(h).speed(0.05).range(0.05..=1000.0));
                            ui.end_row();
                        }
                    });
                    if ui.button("Delete obstacle").clicked() {
                        self.lab.map.obstacles.remove(i);
                        self.editor.editor.selected = None;
                    }
                }
                None => {
                    ui.label(RichText::new("Nothing selected.").weak());
                }
            }

            ui.separator();
            ui.heading("Map");
            let map = &mut self.lab.map;
            egui::Grid::new("map-props").num_columns(2).show(ui, |ui| {
                ui.label("name");
                ui.text_edit_singleline(&mut map.name);
                ui.end_row();
                ui.label("width m");
                ui.add(egui::DragValue::new(&mut map.width).range(2.0..=500.0).speed(0.1));
                ui.end_row();
                ui.label("height m");
                ui.add(egui::DragValue::new(&mut map.height).range(2.0..=500.0).speed(0.1));
                ui.end_row();
                ui.label("start x / y");
                ui.horizontal(|ui| {
                    ui.add(egui::DragValue::new(&mut map.start.x).speed(0.05));
                    ui.add(egui::DragValue::new(&mut map.start.y).speed(0.05));
                });
                ui.end_row();
                ui.label("start heading deg");
                let mut deg = map.start.theta.to_degrees();
                if ui.add(egui::DragValue::new(&mut deg).range(-180.0..=180.0).speed(1.0)).changed() {
                    map.start.theta = deg.to_radians();
                }
                ui.end_row();
                ui.label("goal x / y");
                ui.horizontal(|ui| {
                    ui.add(egui::DragValue::new(&mut map.goal.x).speed(0.05));
                    ui.add(egui::DragValue::new(&mut map.goal.y).speed(0.05));
                });
                ui.end_row();
                ui.label("goal tolerance m");
                ui.add(egui::DragValue::new(&mut map.goal_tolerance).range(0.05..=5.0).speed(0.01));
                ui.end_row();
                ui.label("obstacles");
                ui.label(format!("{}", map.obstacles.len()));
                ui.end_row();
            });
            for issue in validate(&self.lab.map) {
                ui.label(RichText::new(format!("warning: {issue}")).color(canvas::DANGER).small());
            }

            ui.separator();
            ui.heading("Files");
            ui.horizontal_wrapped(|ui| {
                if ui.button("New").clicked() {
                    let (w, h) = (self.editor.new_width, self.editor.new_height);
                    self.lab.set_map(WorldMap::empty("untitled", w, h), "new");
                }
                ui.add(egui::DragValue::new(&mut self.editor.new_width).range(4.0..=200.0).prefix("w "));
                ui.add(egui::DragValue::new(&mut self.editor.new_height).range(4.0..=200.0).prefix("h "));
            });
            ui.horizontal_wrapped(|ui| {
                if ui.button("Open...").clicked() {
                    self.open_map();
                }
                if ui.button("Save as...").clicked() {
                    self.save_map_as();
                }
                let maps = self.caps.as_ref().map(|c| c.maps.clone()).unwrap_or_default();
                let mut chosen = None;
                egui::ComboBox::from_id_salt("editor-builtin").selected_text("built-in...").show_ui(ui, |ui| {
                    for m in &maps {
                        if ui.selectable_label(false, m).clicked() {
                            chosen = Some(m.clone());
                        }
                    }
                });
                if let Some(m) = chosen {
                    self.request_map(json!(format!("builtin:{m}")), MapTarget::Lab);
                }
            });
            ui.label(RichText::new(format!("source: {}", self.lab.map_source)).small().weak());

            ui.separator();
            ui.heading("Procedural generation");
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.editor.gen_kind, "scatter".to_string(), "obstacles");
                ui.selectable_value(&mut self.editor.gen_kind, "maze".to_string(), "maze");
            });
            ui.horizontal(|ui| {
                ui.label("seed");
                ui.add(egui::DragValue::new(&mut self.editor.gen_seed));
                if self.editor.gen_kind == "scatter" {
                    ui.label("count");
                    ui.add(egui::DragValue::new(&mut self.editor.gen_count).range(1..=200));
                }
            });
            if ui.button("Generate (engine)").clicked() {
                let params = json!({"kind": self.editor.gen_kind, "seed": self.editor.gen_seed,
                    "count": self.editor.gen_count, "width": 20, "height": 20});
                if let Some(id) = self.request("generate_map", json!({"params": params})) {
                    self.pending_maps.insert(id, MapTarget::Lab);
                }
            }

            ui.separator();
            ui.heading("Use");
            ui.horizontal_wrapped(|ui| {
                if ui.button("Check with A*").clicked() {
                    let problem = json!({"type": "navigation", "map": self.lab.map.to_json()});
                    self.request("plan", json!({"problem": problem}));
                }
                if ui.button("Optimise in Laboratory").clicked() {
                    self.page = crate::app::Page::Lab;
                }
                if ui.button("Use as testing arena").clicked() {
                    let map = self.lab.map.clone();
                    self.testing.set_arena(map);
                    self.page = crate::app::Page::Testing;
                }
            });
            if let Some(reference) = &self.lab.reference {
                let a = &reference["astar"];
                let text = if a["found"].as_bool() == Some(true) {
                    format!("A*: path {:.2} m, {} expansions", model::num(&a["gridLength"]), a["expansions"])
                } else {
                    "A*: no path for the robot footprint".to_string()
                };
                ui.label(RichText::new(text).small());
            }
        });
    }

    pub fn editor_page(&mut self, ui: &mut egui::Ui) {
        egui::Panel::right("editor-panel").default_size(300.0).resizable(true).show(ui, |ui| self.editor_panel(ui));
        egui::CentralPanel::default().show(ui, |ui| {
            let reference: Option<Vec<[f64; 2]>> = self.lab.reference.as_ref().map(|r| {
                let mut v = vec![[self.lab.map.start.x, self.lab.map.start.y]];
                v.extend(model::points(&r["astar"]["route"]));
                v
            });
            let caption = Some(format!(
                "{}  {:.1} x {:.1} m  |  tool: {}",
                self.lab.map.name,
                self.lab.map.width,
                self.lab.map.height,
                self.editor.editor.tool.label()
            ));
            let dark = self.dark();
            let overlay = Overlay {
                reference: reference.as_deref().filter(|r| r.len() > 1),
                robot_radius: 0.25,
                caption,
                ..Overlay::default()
            };
            let out = canvas::show(ui, &mut self.lab.view, &mut self.lab.map, &overlay, Some(&mut self.editor.editor), dark);
            if out.changed {
                self.lab.map_source = "edited".into();
                self.lab.reference = None;
                self.lab.replay.clear();
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn validation_reports_blocked_start_and_goal() {
        let mut map = WorldMap::empty("t", 10.0, 10.0);
        assert!(validate(&map).is_empty());
        map.obstacles.push(Shape::Circle { x: map.start.x, y: map.start.y, r: 0.5 });
        map.goal.x = 20.0;
        let issues = validate(&map);
        assert_eq!(issues.len(), 2, "{issues:?}");
    }
}
