#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble/setup.bash." >&2
  echo "Install ROS 2 Humble, then run this script again." >&2
  exit 1
fi

# ROS setup scripts read optional variables before defining them.
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$REPO_ROOT"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
export MAKEFLAGS="${MAKEFLAGS:--j2}"
colcon build --symlink-install --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo
echo "Build complete. Run: source $REPO_ROOT/install/setup.bash"

