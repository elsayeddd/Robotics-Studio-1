#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble/setup.bash." >&2
  echo "Install ROS 2 Humble, then run this script again." >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
cd "$REPO_ROOT"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo
echo "Build complete. Run: source $REPO_ROOT/install/setup.bash"

