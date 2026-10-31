//! Algorithms page: the engine's catalogue of algorithms, problems, controllers (with the injected
//! defects) and test-oracle properties, rendered from the capability message.

use eframe::egui::{self, RichText};

use crate::app::RoboApp;
use crate::model::ParamSpec;

fn param_table(ui: &mut egui::Ui, id: &str, params: &[ParamSpec]) {
    if params.is_empty() {
        ui.label(RichText::new("no parameters").weak());
        return;
    }
    egui::Grid::new(id).striped(true).num_columns(4).show(ui, |ui| {
        for h in ["parameter", "default", "range / options", "meaning"] {
            ui.label(RichText::new(h).strong());
        }
        ui.end_row();
        for p in params {
            ui.monospace(&p.name);
            ui.monospace(p.default_value.to_string());
            let range = if !p.options.is_empty() {
                p.options.join(" | ")
            } else {
                match (p.min, p.max) {
                    (Some(a), Some(b)) => format!("[{a}, {b}]"),
                    _ => p.kind.to_lowercase(),
                }
            };
            ui.monospace(range);
            ui.add(egui::Label::new(RichText::new(&p.description).small()).wrap());
            ui.end_row();
        }
    });
}

impl RoboApp {
    pub fn algorithms_page(&mut self, ui: &mut egui::Ui) {
        egui::CentralPanel::default().show(ui, |ui| {
            let Some(caps) = self.caps.clone() else {
                ui.label("Waiting for the engine...");
                return;
            };
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.heading("Search algorithms");
                ui.label("All algorithms minimise fitness, share one evaluation-budget accounting and draw randomness \
                          from a seeded generator. Detailed write-ups: docs/algorithms/.");
                for a in &caps.algorithms {
                    egui::CollapsingHeader::new(RichText::new(format!("{}  —  {}", a.name, a.family)).strong())
                        .id_salt(&a.id)
                        .default_open(false)
                        .show(ui, |ui| {
                            ui.label(&a.summary);
                            ui.label(RichText::new(format!(
                                "representations: {}{}",
                                a.representations.join(", "),
                                if a.multi_objective { "  |  needs an objective vector" } else { "" }
                            )).small().weak());
                            param_table(ui, &format!("alg-{}", a.id), &a.params);
                        });
                }
                ui.add_space(12.0);
                ui.heading("Problems");
                for p in &caps.problems {
                    egui::CollapsingHeader::new(RichText::new(&p.name).strong()).id_salt(&p.id).show(ui, |ui| {
                        ui.label(&p.summary);
                        ui.label(format!("representation: {}  |  objectives: {}", p.representation,
                            if p.objectives.is_empty() { "single-objective".to_string() } else { p.objectives.join(", ") }));
                        for (kind, ops) in &p.operators {
                            ui.label(RichText::new(format!("{kind}: {}", ops.join(", "))).small().monospace());
                        }
                        ui.label(format!("default map: {}", p.default_map));
                        param_table(ui, &format!("prob-{}", p.id), &p.params);
                    });
                }
                ui.add_space(12.0);
                ui.heading("Controllers");
                egui::Grid::new("controllers").striped(true).num_columns(4).show(ui, |ui| {
                    for h in ["id", "kind", "description", "injected defect (expected violation)"] {
                        ui.label(RichText::new(h).strong());
                    }
                    ui.end_row();
                    for c in &caps.controllers {
                        ui.monospace(&c.id);
                        ui.label(&c.kind);
                        ui.add(egui::Label::new(RichText::new(&c.summary).small()).wrap());
                        match &c.defect {
                            Some(d) => ui.add(egui::Label::new(RichText::new(format!(
                                "{d} ({})", c.expected_property.clone().unwrap_or_default())).small().color(crate::canvas::DANGER)).wrap()),
                            None => ui.label("-"),
                        };
                        ui.end_row();
                    }
                });
                ui.add_space(12.0);
                ui.heading("Test-oracle properties");
                egui::Grid::new("properties").striped(true).num_columns(3).show(ui, |ui| {
                    for p in &caps.properties {
                        ui.monospace(&p.id);
                        ui.label(if p.safety { "safety (default set)" } else { "optional" });
                        ui.label(&p.description);
                        ui.end_row();
                    }
                });
            });
        });
    }
}
