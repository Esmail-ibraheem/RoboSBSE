#!/usr/bin/env bash
# Shared environment for the shell scripts (Linux/macOS). Machine-specific settings go into
# scripts/env.local.sh (not committed), e.g.  export JAVA_HOME=/opt/jdk-21
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "$ROOT/scripts/env.local.sh" ]; then source "$ROOT/scripts/env.local.sh"; fi
if [ -n "${ROBOSBSE_JAVA:-}" ]; then
  JAVA="$ROBOSBSE_JAVA"
elif [ -n "${JAVA_HOME:-}" ]; then
  JAVA="$JAVA_HOME/bin/java"
else
  JAVA=java
fi
version=$("$JAVA" -version 2>&1 | sed -n 's/.*version "\([0-9]*\).*/\1/p' | head -1)
if [ -z "$version" ] || [ "$version" -lt 21 ]; then
  echo "RoboSBSE needs Java 21+ (found '${version:-none}' at $JAVA); set JAVA_HOME" >&2
  exit 1
fi
export ROBOSBSE_JAVA="$JAVA"
