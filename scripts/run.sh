#!/usr/bin/env bash
# Starts the RoboSBSE desktop UI (builds it first if needed). Extra arguments are passed to the UI.
source "$(dirname "$0")/env.sh"
exe="$ROOT/frontend/rust-ui/target/release/robosbse"
[ -x "$exe" ] || exe="$ROOT/frontend/rust-ui/target/debug/robosbse"
if [ ! -f "$ROOT/backend/java-engine/build/libs/robosbse-engine.jar" ] || [ ! -x "$exe" ]; then
  "$ROOT/scripts/build.sh" --release
  exe="$ROOT/frontend/rust-ui/target/release/robosbse"
fi
cd "$ROOT"
exec "$exe" "$@"
