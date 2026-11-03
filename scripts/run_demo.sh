#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f "$REPO_ROOT/install/setup.bash" ]]; then
  echo "The workspace is not built. Run ./scripts/build.sh first." >&2
  exit 1
fi

# ROS setup scripts read optional variables before defining them.
set +u
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u

exec ros2 launch embertrace_bringup embertrace_demo.launch.py \
  use_nav2:=true use_drone:=true use_rviz:=true use_dashboard:=true "$@"
