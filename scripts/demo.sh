#!/usr/bin/env bash
# Headless Phase 1 demonstration: route optimisation, failure discovery, reproduction.
source "$(dirname "$0")/env.sh"
jar="$ROOT/backend/java-engine/build/libs/robosbse-engine.jar"
[ -f "$jar" ] || "$ROOT/scripts/build.sh"
cd "$ROOT"
"$JAVA" -jar "$jar" run examples/experiments/nav-warehouse-ga.json --out experiments/results
"$JAVA" -jar "$jar" run examples/experiments/testing-bug1-ga.json --out experiments/results
latest=$(ls -d experiments/results/*testing-bug1-ga* | sort | tail -1)
"$JAVA" -jar "$jar" reproduce "$latest/failures/failure-1.json"
