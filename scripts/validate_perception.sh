#!/usr/bin/env bash
# Standalone reproducible check using the exact fusion implementation used by ROS.
set -euo pipefail
TASK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TASK_OUTPUT="${1:-$TASK_ROOT/log/perception-validation}"
mkdir -p "$TASK_OUTPUT"
g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror -O2 \
  -I "$TASK_ROOT/src/embertrace_core/include" \
  "$TASK_ROOT/src/embertrace_core/src/perception_filter.cpp" \
  "$TASK_ROOT/src/embertrace_core/src/evaluate_perception.cpp" \
  -o "$TASK_OUTPUT/evaluate_perception"
python3 "$TASK_ROOT/scripts/generate_perception_trials.py" "$TASK_OUTPUT/synthetic_trials.csv"
"$TASK_OUTPUT/evaluate_perception" "$TASK_OUTPUT/synthetic_trials.csv" > "$TASK_OUTPUT/metrics.csv"
cat "$TASK_OUTPUT/metrics.csv"
g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror -O2 \
  -I "$TASK_ROOT/src/embertrace_core/include" \
  "$TASK_ROOT/src/embertrace_core/src/perception_filter.cpp" \
  "$TASK_ROOT/src/embertrace_core/src/visual_classifier.cpp" \
  "$TASK_ROOT/src/embertrace_core/test/test_perception_filter.cpp" \
  -o "$TASK_OUTPUT/test_perception_filter"
"$TASK_OUTPUT/test_perception_filter"
