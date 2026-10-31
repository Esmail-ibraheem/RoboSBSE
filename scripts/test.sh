#!/usr/bin/env bash
# Runs all automated tests: JUnit (engine) and cargo test (UI, including the end-to-end engine test).
source "$(dirname "$0")/env.sh"
(cd "$ROOT/backend/java-engine" && ./gradlew test engineJar --no-daemon)
(cd "$ROOT/frontend/rust-ui" && cargo test)
echo "all tests passed"
