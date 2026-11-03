#!/usr/bin/env bash
set -euo pipefail

if [[ "$(. /etc/os-release && printf '%s' "$VERSION_ID")" != "22.04" ]]; then
  echo "EmberTrace targets Ubuntu 22.04. This system is not Ubuntu 22.04." >&2
  echo "Use Ubuntu 22.04 or WSL2 Ubuntu 22.04 to match ROS 2 Humble + Fortress." >&2
  exit 1
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is not installed." >&2
  echo "Install ros-humble-desktop from the official ROS 2 Humble Ubuntu guide:" >&2
  echo "https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  ros-humble-ros-ign \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-xacro \
  ros-humble-teleop-twist-keyboard \
  ros-humble-rviz2 \
  python3-colcon-common-extensions \
  python3-rosdep \
  qtbase5-dev

# ROS setup scripts read optional variables before defining them.
set +u
source /opt/ros/humble/setup.bash
set -u

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rosdep install --from-paths "$REPO_ROOT/src" --ignore-src -r -y --rosdistro humble

echo "Dependencies installed. Next run: ./scripts/build.sh"

