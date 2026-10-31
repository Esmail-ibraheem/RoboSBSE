//! Settings: engine location, directories, appearance; engine restart and its error output.

use eframe::egui::{self, RichText};

use crate::app::{apply_style, RoboApp};
use crate::settings::Settings;

impl RoboApp {
    pub fn settings_page(&mut self, ui: &mut egui::Ui) {
        egui::CentralPanel::default().show(ui, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.heading("Engine");
                let mut s = self.settings.clone();
                egui::Grid::new("settings").num_columns(3).spacing([10.0, 8.0]).show(ui, |ui| {
                    ui.label("Java executable");
                    ui.add(egui::TextEdit::singleline(&mut s.java).desired_width(520.0));
                    if ui.button("Browse").clicked() {
                        if let Some(p) = rfd::FileDialog::new().pick_file() {
                            s.java = p.display().to_string();
                        }
                    }
                    ui.end_row();
                    ui.label("Engine jar");
                    ui.add(egui::TextEdit::singleline(&mut s.engine_jar).desired_width(520.0));
                    if ui.button("Browse").clicked() {
                        if let Some(p) = rfd::FileDialog::new().add_filter("jar", &["jar"]).pick_file() {
                            s.engine_jar = p.display().to_string();
                        }
                    }
                    ui.end_row();
                    ui.label("Experiments directory");
                    ui.add(egui::TextEdit::singleline(&mut s.experiments_dir).desired_width(520.0));
                    if ui.button("Browse").clicked() {
                        if let Some(p) = rfd::FileDialog::new().pick_folder() {
                            s.experiments_dir = p.display().to_string();
                        }
                    }
                    ui.end_row();
                    ui.label("Maps directory");
                    ui.add(egui::TextEdit::singleline(&mut s.maps_dir).desired_width(520.0));
                    if ui.button("Browse").clicked() {
                        if let Some(p) = rfd::FileDialog::new().pick_folder() {
                            s.maps_dir = p.display().to_string();
                        }
                    }
                    ui.end_row();
                    ui.label("Dark mode");
                    ui.checkbox(&mut s.dark_mode, "");
                    ui.end_row();
                    ui.label("UI scale");
                    ui.add(egui::Slider::new(&mut s.ui_scale, 0.75..=2.0).step_by(0.05));
                    ui.end_row();
                });
                let appearance_changed = s.dark_mode != self.settings.dark_mode || s.ui_scale != self.settings.ui_scale;
                if s != self.settings {
                    self.settings = s;
                    if appearance_changed {
                        apply_style(ui.ctx(), &self.settings);
                    }
                }
                ui.horizontal(|ui| {
                    if ui.button("Save settings").clicked() {
                        match self.settings.save() {
                            Ok(_) => self.notify(format!("settings saved to {}", Settings::file().display())),
                            Err(e) => self.error(format!("cannot save settings: {e}")),
                        }
                    }
                    if ui.button("Restart engine").clicked() {
                        let ctx = ui.ctx().clone();
                        self.restart_engine(&ctx);
                    }
                    if ui.button("Defaults").clicked() {
                        self.settings = Settings::default();
                        apply_style(ui.ctx(), &self.settings);
                    }
                });
                ui.label(RichText::new(format!("settings file: {}", Settings::file().display())).small().weak());
                ui.label(RichText::new(format!("command: {}", self.engine.command_line)).small().monospace());
                ui.label(RichText::new(format!("status: {:?}", self.engine.status)).small().monospace());
                ui.add_space(10.0);
                ui.heading("Engine error output");
                egui::ScrollArea::vertical().id_salt("stderr").max_height(300.0).stick_to_bottom(true).show(ui, |ui| {
                    if self.engine_stderr.is_empty() {
                        ui.label(RichText::new("(empty)").weak());
                    }
                    for line in &self.engine_stderr {
                        ui.label(RichText::new(line).monospace().small());
                    }
                });
            });
        });
    }
}
