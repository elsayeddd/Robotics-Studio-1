#!/usr/bin/env bash
# Compile and run the ROS-independent implementation and fixture regressions.
set -euo pipefail
TASK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TASK_OUTPUT="${1:-$TASK_ROOT/log/perception-validation}"
TASK_CORE="$TASK_ROOT/src/embertrace_core"
python3 "$TASK_ROOT/scripts/validate_repo.py"
bash "$TASK_ROOT/scripts/validate_perception.sh" "$TASK_OUTPUT"
g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror -O2 \
  -I "$TASK_CORE/include" \
  "$TASK_CORE/src/mission_logic.cpp" "$TASK_CORE/src/perception_filter.cpp" \
  "$TASK_CORE/test/test_mission_logic.cpp" -o "$TASK_OUTPUT/test_mission_logic"
"$TASK_OUTPUT/test_mission_logic"
g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror -O2 \
  -I "$TASK_CORE/include" \
  "$TASK_CORE/src/perception_filter.cpp" "$TASK_CORE/src/plume_model.cpp" \
  "$TASK_CORE/src/source_estimator.cpp" "$TASK_CORE/src/known_free_grid.cpp" \
  "$TASK_CORE/test/test_system_logic.cpp" -o "$TASK_OUTPUT/test_system_logic"
"$TASK_OUTPUT/test_system_logic"
EMBERTRACE_EVALUATOR="$TASK_OUTPUT/evaluate_perception" \
  python3 "$TASK_ROOT/scripts/test_simulation_tools.py"
