//! Engine process management: spawns `java -jar robosbse-engine.jar serve` and talks JSON Lines
//! over its stdin/stdout. A reader thread parses every line and forwards it to the UI thread
//! through a channel; the UI never blocks on the engine.

use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::thread;

use eframe::egui;
use serde_json::{json, Value};

/// Protocol version spoken by this UI (must match the engine's `protocol` field).
pub const PROTOCOL_VERSION: i64 = 1;

pub enum EngineEvent {
    Message(Value),
    Stderr(String),
    Exited(String),
}

#[derive(Clone, Debug, PartialEq)]
pub enum EngineStatus {
    Starting,
    Ready,
    Failed(String),
    Stopped,
}

pub struct Engine {
    child: Option<Child>,
    stdin: Option<ChildStdin>,
    rx: Receiver<EngineEvent>,
    pub status: EngineStatus,
    next_id: u64,
    pub command_line: String,
}

impl Engine {
    /// Starts the engine. Failures are reported through `status`, never by panicking.
    pub fn spawn(java: &str, jar: &Path, cwd: &Path, ctx: egui::Context) -> Engine {
        let (tx, rx) = mpsc::channel();
        let command_line = format!("{} -jar {} serve", java, jar.display());
        if !jar.exists() {
            return Engine {
                child: None,
                stdin: None,
                rx,
                status: EngineStatus::Failed(format!(
                    "engine jar not found at {} — build it with scripts/build (gradlew engineJar)",
                    jar.display()
                )),
                next_id: 1,
                command_line,
            };
        }
        let mut command = Command::new(java);
        command
            .arg("-jar")
            .arg(jar)
            .arg("serve")
            .current_dir(cwd)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        #[cfg(windows)]
        {
            use std::os::windows::process::CommandExt;
            const CREATE_NO_WINDOW: u32 = 0x0800_0000;
            command.creation_flags(CREATE_NO_WINDOW);
        }
        let mut child = match command.spawn() {
            Ok(c) => c,
            Err(e) => {
                return Engine {
                    child: None,
                    stdin: None,
                    rx,
                    status: EngineStatus::Failed(format!("cannot start '{java}': {e}")),
                    next_id: 1,
                    command_line,
                }
            }
        };
        let stdin = child.stdin.take();
        if let Some(stdout) = child.stdout.take() {
            let tx = tx.clone();
            let ctx = ctx.clone();
            thread::spawn(move || {
                let reader = BufReader::new(stdout);
                for line in reader.lines() {
                    match line {
                        Ok(line) if line.trim().is_empty() => continue,
                        Ok(line) => {
                            let event = match serde_json::from_str::<Value>(&line) {
                                Ok(v) => EngineEvent::Message(v),
                                Err(e) => EngineEvent::Stderr(format!("unparseable engine output ({e}): {line}")),
                            };
                            if tx.send(event).is_err() {
                                return;
                            }
                            ctx.request_repaint();
                        }
                        Err(e) => {
                            let _ = tx.send(EngineEvent::Exited(format!("read error: {e}")));
                            ctx.request_repaint();
                            return;
                        }
                    }
                }
                let _ = tx.send(EngineEvent::Exited("engine closed its output".into()));
                ctx.request_repaint();
            });
        }
        if let Some(stderr) = child.stderr.take() {
            let ctx = ctx.clone();
            thread::spawn(move || {
                for line in BufReader::new(stderr).lines().map_while(Result::ok) {
                    if tx.send(EngineEvent::Stderr(line)).is_err() {
                        return;
                    }
                    ctx.request_repaint();
                }
            });
        }
        Engine { child: Some(child), stdin, rx, status: EngineStatus::Starting, next_id: 1, command_line }
    }

    pub fn is_ready(&self) -> bool {
        self.status == EngineStatus::Ready
    }

    /// Sends a request; `id` and protocol version are filled in. Returns the request id.
    pub fn request(&mut self, kind: &str, body: Value) -> Result<String, String> {
        let id = format!("{}-{}", kind, self.next_id);
        self.next_id += 1;
        self.request_with_id(kind, &id, body)?;
        Ok(id)
    }

    pub fn request_with_id(&mut self, kind: &str, id: &str, body: Value) -> Result<(), String> {
        let mut message = match body {
            Value::Object(map) => Value::Object(map),
            Value::Null => json!({}),
            other => return Err(format!("request body must be an object, got {other}")),
        };
        message["v"] = json!(PROTOCOL_VERSION);
        message["type"] = json!(kind);
        message["id"] = json!(id);
        self.send(&message)
    }

    fn send(&mut self, message: &Value) -> Result<(), String> {
        let stdin = self.stdin.as_mut().ok_or_else(|| "engine is not running".to_string())?;
        let mut line = serde_json::to_string(message).map_err(|e| e.to_string())?;
        line.push('\n');
        stdin.write_all(line.as_bytes()).and_then(|_| stdin.flush()).map_err(|e| {
            self.status = EngineStatus::Failed(format!("write to engine failed: {e}")); 
            e.to_string()
        })
    }

    /// Drains all pending events without blocking.
    pub fn poll(&mut self) -> Vec<EngineEvent> {
        let mut events = Vec::new();
        loop {
            match self.rx.try_recv() {
                Ok(event) => {
                    match &event {
                        EngineEvent::Message(m) if m["type"] == "ready" => {
                            let protocol = m["protocol"].as_i64().unwrap_or(-1);
                            self.status = if protocol == PROTOCOL_VERSION {
                                EngineStatus::Ready
                            } else {
                                EngineStatus::Failed(format!(
                                    "protocol mismatch: engine speaks {protocol}, UI speaks {PROTOCOL_VERSION}"
                                ))
                            };
                        }
                        EngineEvent::Exited(reason) => {
                            if !matches!(self.status, EngineStatus::Failed(_)) {
                                self.status = EngineStatus::Failed(reason.clone());
                            }
                        }
                        _ => {}
                    }
                    events.push(event);
                }
                Err(TryRecvError::Empty) | Err(TryRecvError::Disconnected) => break,
            }
        }
        events
    }

    pub fn shutdown(&mut self) {
        if self.stdin.is_some() {
            let _ = self.send(&json!({"v": PROTOCOL_VERSION, "type": "shutdown", "id": "bye"}));
        }
        self.stdin = None;
        if let Some(mut child) = self.child.take() {
            for _ in 0..20 {
                if let Ok(Some(_)) = child.try_wait() {
                    break;
                }
                thread::sleep(std::time::Duration::from_millis(25));
            }
            let _ = child.kill();
            let _ = child.wait();
        }
        self.status = EngineStatus::Stopped;
    }
}

impl Drop for Engine {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// Java executable: `ROBOSBSE_JAVA`, then `JAVA_HOME/bin/java`, then `java` on the PATH.
pub fn default_java() -> String {
    if let Ok(java) = std::env::var("ROBOSBSE_JAVA") {
        if !java.trim().is_empty() {
            return java;
        }
    }
    if let Ok(home) = std::env::var("JAVA_HOME") {
        let exe = if cfg!(windows) { "java.exe" } else { "java" };
        let candidate = PathBuf::from(home).join("bin").join(exe);
        if candidate.exists() {
            return candidate.display().to_string();
        }
    }
    "java".to_string()
}
