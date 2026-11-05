#!/usr/bin/env bash
# Open the simulator and ranger console; the presenter starts the mission in the UI.
set -euo pipefail
TASK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TASK_PLATFORM="${1:-drone}"
case "$TASK_PLATFORM" in
  drone|rover) ;;
  *) echo "Usage: $0 [drone|rover] [embertrace_forest.sdf|embertrace_forest_dense.sdf]" >&2; exit 2 ;;
esac
TASK_WORLD="${2:-embertrace_forest_dense.sdf}"
case "$TASK_WORLD" in
  embertrace_forest.sdf|embertrace_forest_dense.sdf) ;;
  *) echo "World must be embertrace_forest.sdf or embertrace_forest_dense.sdf" >&2; exit 2 ;;
esac
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-63}"
export LP_NUM_THREADS="${LP_NUM_THREADS:-6}"
export IGN_PARTITION="embertrace_showcase_${UID}_$$"
export GZ_PARTITION="$IGN_PARTITION"
cd "$TASK_ROOT"
echo "Starting the $TASK_PLATFORM showcase. Gazebo and the Ranger Console will open; click Start in the console."
exec /usr/bin/python3 "$TASK_ROOT/scripts/showcase_session.py" \
  bash "$TASK_ROOT/scripts/run_demo.sh" \
  mission_platform:="$TASK_PLATFORM" scenario:=fire \
  world_file:="$TASK_WORLD" gazebo_gui:=true \
  use_dashboard:=true use_rviz:=false
