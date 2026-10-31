//! User settings (engine location, directories, appearance), stored as JSON next to the repository or
//! the portable download (or in `~/RoboSBSE` when that folder is read-only).
//!
//! Two layouts are recognised. A development checkout has `backend/java-engine`; the engine jar is built
//! there. A portable download (`scripts/package/package.py`) has the executable next to
//! `engine/robosbse-engine.jar`, `runtime/` (a bundled Java runtime) and `examples/`; in the macOS app
//! bundle these live in `Contents/Resources`.

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

/// The engine jar of a portable download, relative to its root.
const PORTABLE_JAR: &str = "engine/robosbse-engine.jar";

impl Default for Settings {
    fn default() -> Self {
        let root = find_repo_root();
        let portable = root.join(PORTABLE_JAR);
        let engine_jar = if portable.is_file() { portable } else { root.join("backend/java-engine/build/libs/robosbse-engine.jar") };
        Settings {
            java: bundled_java(&root).map(|j| j.display().to_string()).unwrap_or_else(engine::default_java),
            engine_jar: engine_jar.display().to_string(),
            experiments_dir: data_dir(&root).join("experiments").display().to_string(),
            maps_dir: root.join("examples/maps").display().to_string(),
            dark_mode: true,
            ui_scale: 1.0,
        }
    }
}

/// The Java runtime bundled with a portable download, if there is one.
pub fn bundled_java(root: &Path) -> Option<PathBuf> {
    let exe = if cfg!(windows) { "java.exe" } else { "java" };
    Some(root.join("runtime").join("bin").join(exe)).filter(|p| p.is_file())
}

/// Where settings and experiment results are written: the root itself when it is writable (a checkout or
/// an unpacked download), otherwise `RoboSBSE` in the home folder (e.g. a macOS app run from a read-only
/// location).
pub fn data_dir(root: &Path) -> PathBuf {
    let probe = root.join(".robosbse-write-test");
    if std::fs::write(&probe, b"").is_ok() {
        let _ = std::fs::remove_file(&probe);
        return root.to_path_buf();
    }
    let home = std::env::var_os(if cfg!(windows) { "USERPROFILE" } else { "HOME" }).map(PathBuf::from);
    home.unwrap_or_else(std::env::temp_dir).join("RoboSBSE")
}

impl Settings {
    pub fn file() -> PathBuf {
        let dir = data_dir(&find_repo_root());
        let _ = std::fs::create_dir_all(&dir);
        dir.join("robosbse-ui.json")
    }

    /// Loads saved settings; environment overrides (`ROBOSBSE_JAVA`) win over the file.
    pub fn load() -> Settings {
        let mut s: Settings = std::fs::read_to_string(Self::file())
            .ok()
            .and_then(|t| serde_json::from_str(&t).ok())
            .unwrap_or_default();
        // Saved paths go stale when a portable folder is moved: fall back to the defaults for this location.
        let defaults = Settings::default();
        if !Path::new(&s.engine_jar).is_file() {
            s.engine_jar = defaults.engine_jar;
        }
        if Path::new(&s.java).is_absolute() && !Path::new(&s.java).is_file() {
            s.java = defaults.java;
        }
        if !Path::new(&s.maps_dir).is_dir() {
            s.maps_dir = defaults.maps_dir;
        }
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

/// The installation root: the first ancestor of the working directory or of the executable that is a
/// repository checkout (contains `backend/java-engine`) or a portable download (contains
/// `engine/robosbse-engine.jar`); for a macOS app bundle also `Contents/Resources`. Falls back to the
/// working directory.
pub fn find_repo_root() -> PathBuf {
    let mut starts: Vec<PathBuf> = Vec::new();
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            starts.push(dir.join("..").join("Resources"));
            starts.push(dir.to_path_buf());
        }
    }
    if let Ok(cwd) = std::env::current_dir() {
        starts.insert(0, cwd);
    }
    for start in &starts {
        let mut dir: Option<&Path> = Some(start.as_path());
        while let Some(d) = dir {
            if d.join("backend").join("java-engine").is_dir() || d.join(PORTABLE_JAR).is_file() {
                return d.canonicalize().map(strip_unc).unwrap_or_else(|_| d.to_path_buf());
            }
            dir = d.parent();
        }
    }
    starts.into_iter().next().unwrap_or_else(|| PathBuf::from("."))
}

/// `canonicalize` on Windows returns `\\?\C:\...` paths, which some programs (e.g. java -jar) reject.
fn strip_unc(p: PathBuf) -> PathBuf {
    let text = p.display().to_string();
    match text.strip_prefix(r"\\?\") {
        Some(rest) if !rest.starts_with("UNC") => PathBuf::from(rest),
        _ => p,
    }
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
