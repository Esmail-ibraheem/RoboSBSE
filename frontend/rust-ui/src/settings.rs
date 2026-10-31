//! User settings (engine location, directories, appearance), stored as JSON next to the repository.

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::engine;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct Settings {
    pub java: String,
    pub engine_jar: String,
    pub experiments_dir: String,
    pub maps_dir: String,
    pub dark_mode: bool,
    pub ui_scale: f32,
}

impl Default for Settings {
    fn default() -> Self {
        let root = find_repo_root();
        Settings {
            java: engine::default_java(),
            engine_jar: root.join("backend/java-engine/build/libs/robosbse-engine.jar").display().to_string(),
            experiments_dir: root.join("experiments").display().to_string(),
            maps_dir: root.join("examples/maps").display().to_string(),
            dark_mode: true,
            ui_scale: 1.0,
        }
    }
}

impl Settings {
    pub fn file() -> PathBuf {
        find_repo_root().join("robosbse-ui.json")
    }

    /// Loads saved settings; environment overrides (`ROBOSBSE_JAVA`) win over the file.
    pub fn load() -> Settings {
        let mut s: Settings = std::fs::read_to_string(Self::file())
            .ok()
            .and_then(|t| serde_json::from_str(&t).ok())
            .unwrap_or_default();
        if let Ok(java) = std::env::var("ROBOSBSE_JAVA") {
            if !java.trim().is_empty() {
                s.java = java;
            }
        }
        if s.ui_scale < 0.5 || s.ui_scale > 3.0 {
            s.ui_scale = 1.0;
        }
        s
    }

    pub fn save(&self) -> Result<(), String> {
        let text = serde_json::to_string_pretty(self).map_err(|e| e.to_string())?;
        std::fs::write(Self::file(), text).map_err(|e| e.to_string())
    }

    pub fn results_dir(&self) -> PathBuf {
        Path::new(&self.experiments_dir).join("results")
    }

    pub fn benchmarks_dir(&self) -> PathBuf {
        Path::new(&self.experiments_dir).join("benchmarks")
    }

    pub fn configs_dir(&self) -> PathBuf {
        Path::new(&self.experiments_dir).join("configs")
    }
}

/// The repository root: the first ancestor of the working directory or of the executable that
/// contains `backend/java-engine`. Falls back to the working directory.
pub fn find_repo_root() -> PathBuf {
    let mut starts: Vec<PathBuf> = Vec::new();
    if let Ok(cwd) = std::env::current_dir() {
        starts.push(cwd);
    }
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            starts.push(dir.to_path_buf());
        }
    }
    for start in &starts {
        let mut dir: Option<&Path> = Some(start.as_path());
        while let Some(d) = dir {
            if d.join("backend").join("java-engine").is_dir() {
                return d.to_path_buf();
            }
            dir = d.parent();
        }
    }
    starts.into_iter().next().unwrap_or_else(|| PathBuf::from("."))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn settings_round_trip_and_fill_missing_fields() {
        let s: Settings = serde_json::from_str(r#"{"java":"x","uiScale":1.5}"#).unwrap();
        assert_eq!(s.java, "x");
        assert_eq!(s.ui_scale, 1.5);
        assert!(s.engine_jar.ends_with("robosbse-engine.jar"));
        let text = serde_json::to_string(&s).unwrap();
        assert_eq!(serde_json::from_str::<Settings>(&text).unwrap(), s);
    }

    #[test]
    fn repo_root_contains_the_engine() {
        // Tests run with the crate directory as working directory, inside the repository.
        assert!(find_repo_root().join("backend").join("java-engine").is_dir());
    }
}
