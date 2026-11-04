# VM startup and perception testing

Use Ubuntu 22.04, ROS 2 Humble and Gazebo Fortress. Keep the workspace on the VM's
Linux filesystem. A practical VM allocation is four CPU cores and 8 GB RAM.

## Get the integrated source

Use a GitHub account that has repository access. Configure your own Git author
name and email; do not copy another teammate's identity.

```bash
git clone git@github.com:elsayeddd/Robotics-Studio-1.git
cd Robotics-Studio-1
git switch main
git pull --ff-only
```

For an existing clone, inspect `git status` first and preserve unrelated local
changes. The IR/CO2 updates are integrated into main; no ZIP patch is needed.

## Dependencies and build

Install ROS Humble desktop using the official Ubuntu deb instructions linked in
the README, then:

```bash
./scripts/install_dependencies.sh
sudo apt-get install -y xvfb libgl1-mesa-dri
./scripts/build.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=73 colcon test --event-handlers console_direct+
colcon test-result --verbose
```

The build helper limits package and compiler concurrency for VM memory. Build the
whole workspace after changing message definitions.

## Run a mission

```bash
./scripts/run_demo.sh mission_platform:=rover scenario:=no_fire
```

Choose `mission_platform:=drone` for the aircraft alone or `combined` for the rover
and drone available with one selected by wind. In another sourced terminal, run preflight for the same mode:

```bash
ros2 run embertrace_core demo_preflight.py --platform rover --timeout 120
ros2 service call /embertrace/operator_command \
  embertrace_interfaces/srv/OperatorCommand "{command: 'START'}"
```

For sensor debugging without ground navigation use `use_nav2:=false` and preflight
`--no-nav2`. `use_thermal:=false` is an explicitly reduced sensor debug mode. It is
not full IR acceptance. A no-fire scene should have ambient CO2 near 420 ppm and no
confirmed fire. The dashboard displays stream freshness, CO2 and decoded IR peaks.

```bash
ros2 topic echo /embertrace/perception_status --once
ros2 topic echo /embertrace/thermal_cue --once
ros2 topic echo /diagnostics --once
```

A standalone drone uses `/drone1/perception_status` and `/drone1/thermal_cue`.

## Automated checks

Stop any interactive launch before running a test that starts its own simulation.

```bash
LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a python3 scripts/ros_demo_smoke.py --platform rover
LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a python3 scripts/ros_demo_smoke.py --platform drone
LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a python3 scripts/ros_demo_smoke.py --platform combined
ROS_DOMAIN_ID=74 LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a python3 scripts/ros_perception_campaign.py
```

The mode checks require actual odometry movement, pause/resume, measured return,
zero command output and a latched emergency stop. The campaign records actual
Gazebo RGB, IR and environmental streams for six scenarios using separate training
and held-out seeds. Inspect the results in `log/` and the source-controlled
[integration review](integration-review.md). Synthetic regression results alone do
not establish Gazebo performance or physical sensor accuracy.

For interactive fire testing, restart with `scenario:=fire`. For false alarms and
sensor loss, use `visual_decoy`, `dust`, `hot_surface` and `dropout`. See the
[demo guide](demo-guide.md) and [perception methods](perception-validation.md) for
recording, metric definitions and exact expected transitions.
