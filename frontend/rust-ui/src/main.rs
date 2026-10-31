//! RoboSBSE desktop laboratory (egui). The UI configures experiments and visualises results; all
//! search, simulation and fitness evaluation run inside the Java engine process.

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod app;
mod canvas;
mod capture;
mod engine;
#[cfg(test)]
mod engine_tests;
mod model;
mod pages;
mod params;
mod runs;
mod settings;

use eframe::egui;

fn main() -> eframe::Result<()> {
    let args = capture::CliArgs::parse(std::env::args().skip(1));
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("RoboSBSE — Search-Based Robotics Laboratory")
            .with_inner_size([1560.0, 960.0])
            .with_min_inner_size([1024.0, 640.0]),
        ..Default::default()
    };
    eframe::run_native(
        "RoboSBSE",
        options,
        Box::new(move |cc| Ok(Box::new(app::RoboApp::new(cc, args)))),
    )
}
