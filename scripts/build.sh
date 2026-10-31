#!/usr/bin/env bash
# Builds the Java engine jar and the Rust desktop UI.  Usage: scripts/build.sh [--release]
source "$(dirname "$0")/env.sh"
(cd "$ROOT/backend/java-engine" && ./gradlew engineJar --no-daemon -q)
if [ "${1:-}" = "--release" ]; then
  (cd "$ROOT/frontend/rust-ui" && cargo build --release)
else
  (cd "$ROOT/frontend/rust-ui" && cargo build)
fi
