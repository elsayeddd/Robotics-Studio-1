# Rover and aerial scout demo guide

## Get and build this implementation in WSL

Use Ubuntu 22.04 / ROS 2 Humble / Gazebo Fortress, with WSLg available for Gazebo,
RViz and the Qt dashboard. The code is on `feature/perception-validation` until
its pull request is merged.

```bash
cd ~/projects/Robotics-Studio-1
git fetch origin
git switch feature/perception-validation
git pull --ff-only
source /opt/ros/humble/setup.bash
./scripts/install_dependencies.sh
./scripts/build.sh
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

## First demo: both vehicles, no fire

In terminal 1:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/run_demo.sh scenario:=no_fire
```

This starts Gazebo, the rover, a blue aerial scout, both sensor bridges, local
perception pipelines, diagnostics, SLAM, Nav2, the existing patrol/search nodes,
RViz and the dashboard. It leaves the mission idle so sensors and navigation can
initialise before motion begins. The scout starts hovering at 8 m.

In terminal 2 (from the repository root):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run embertrace_core demo_preflight.py --timeout 60
ros2 service call /embertrace/operator_command \
  embertrace_interfaces/srv/OperatorCommand "{command: 'START'}"
```

Proceed when every preflight row says PASS. The rover follows the original patrol
route while the drone follows its bounded aerial sweep. Watch the vehicle poses
and the navigation status; verify the rover actually reaches successive waypoints.
Two complete patrol loops remain a target-machine acceptance step.

```bash
ros2 topic echo /embertrace/navigation_status
ros2 topic echo /diagnostics
```

`/diagnostics` lists odometry, IMU, LiDAR, RGB, depth and the environmental stream
for each vehicle. Point-cloud streams are monitored as supplementary diagnostics.
Depth obstacle points also feed the rover's Nav2 costmaps.

## Fire investigation demo

Stop the first launch with Ctrl+C, then start a fresh trial:

```bash
./scripts/run_demo.sh scenario:=fire
```

Run preflight and START again. The rover's local smoke/temperature measurements
can trigger investigation. Fresh drone environmental evidence can also request
investigation and select a short rover probe toward the measured sample position.
The rover still requires its own fresh environmental evidence to confirm an alert.

The search planner retains the existing source estimator and projects every
investigation goal into known free map space. An aerial cue in unexplored or
blocked terrain does not grant permission to drive there. In that case, inspect
the map and navigation logs rather than expecting a direct flight-to-rover goal.

A sustained local plume should lead through INVESTIGATE, TRACK, VERIFY and ALERT.
An ALERT holds movement. Source coordinates come from the estimator; if the fit
is not fresh and sufficiently certain, the alert explicitly reports the latest
sample position as an unresolved-location fallback.

## False alarms and sensor dropout

| Launch argument | What changes | Expected behaviour |
| --- | --- | --- |
| `scenario:=no_fire` | No flame marker, ambient field | Patrol; no confirmation |
| `scenario:=dust` | Smoke plume, ambient temperature, no flame marker | Investigation possible; no smoke-only confirmation |
| `scenario:=hot_surface` | Heat field, no smoke plume or flame marker | Thermal evidence; no heat-only confirmation |
| `scenario:=visual_decoy` | Flame-colour marker, ambient smoke/temperature | Vision alone never confirms |
| `scenario:=dropout` | Fire field; local sensor stream suppressed for 8 s starting 25 simulated seconds after first odometry | Data goes stale, rover pauses, commands go to zero |

After dropout ends, wait for fresh sensors and perception warmup, then explicitly
RESUME. The system does not automatically resume a paused mission.

```bash
ros2 service call /embertrace/operator_command \
  embertrace_interfaces/srv/OperatorCommand "{command: 'RESUME'}"
```

Other controls:

```bash
ros2 service call /embertrace/operator_command embertrace_interfaces/srv/OperatorCommand "{command: 'PAUSE'}"
ros2 service call /embertrace/operator_command embertrace_interfaces/srv/OperatorCommand "{command: 'RETURN'}"
ros2 service call /embertrace/operator_command embertrace_interfaces/srv/OperatorCommand "{command: 'E_STOP'}"
ros2 service call /embertrace/operator_command embertrace_interfaces/srv/OperatorCommand "{command: 'RESET'}"
```

RETURN sends the rover to the existing base waypoint and the drone to its base
hover position. E_STOP latches: START/RESUME/RETURN cannot bypass it; RESET is
required. The rover command guard publishes zero when mission permission, sensor
readiness or a command heartbeat is missing. The aerial controller applies the
same mission and sensor checks, a geofence and a LiDAR proximity hold.

## Record perception acceptance evidence

While a scenario is running, open another sourced terminal:

```bash
ros2 run embertrace_core record_perception.py \
  --robot husky1 --trial fire_01 --duration 120 \
  --output log/trials/fire_01.csv
ros2 run embertrace_core evaluate_perception log/trials/fire_01.csv
```

Use `--robot drone1` and a separate filename to record the aerial stream. Record
at least fire, no_fire and visual_decoy; include dust/hot_surface/dropout for a
stronger report. Repeat with distinct trial IDs. The evaluation-only truth topic
never feeds autonomy. It labels local observability in this mathematical model,
not a real fire dataset.

To combine trials (one header only):

```bash
python3 - <<'PY'
import csv
from pathlib import Path
paths = sorted(Path('log/trials').glob('*_*.csv'))
with Path('log/trials/combined.csv').open('w', newline='') as output:
    writer = csv.writer(output)
    for index, path in enumerate(paths):
        with path.open(newline='') as source:
            reader = csv.reader(source)
            header = next(reader)
            if index == 0:
                writer.writerow(header)
            writer.writerows(reader)
PY
ros2 run embertrace_core evaluate_perception log/trials/combined.csv > log/trials/metrics.csv
```

Keep training and acceptance trials separate. The threshold sweep utility and
metric definitions are in [Perception validation](perception-validation.md).

To record the complete automated campaign (six scenarios, separate training and
held-out seeds, both vehicles) with the mission idle:

```bash
python3 scripts/ros_perception_campaign.py
```

This launches fresh trials near the source, records local sensor observations,
checks dropout and a visible decoy, tunes on rover training data, and writes
held-out metrics for each vehicle to `log/gazebo-perception/`. It takes several
minutes. Close an existing demo first. CI runs the same campaign under Xvfb and
preserves all CSVs and launch logs as a downloadable artifact.

## Isolate startup problems

For sensor/teleoperation work without Nav2 or the drone:

```bash
./scripts/run_demo.sh use_nav2:=false use_drone:=false
ros2 run embertrace_core demo_preflight.py --no-nav2 --no-drone
```

START the mission before using teleoperation. Route teleop into the guard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/husky1/autonomy_cmd_vel
```

If GUI windows fail in WSL, first confirm that WSLg can open another Linux GUI
application. `use_rviz:=false use_dashboard:=false` removes those two optional
windows. Gazebo still needs a working rendering context to produce camera/LiDAR
samples; suppressing a window does not substitute for sensor readiness.

For an automated short check on a ROS machine:

```bash
python3 scripts/ros_demo_smoke.py
```

On a headless Linux machine with Xvfb and Mesa installed:

```bash
LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a python3 scripts/ros_demo_smoke.py
```

The check validates preflight, actual movement of both vehicles and zero command
output after E-stop, and preserves the launch log under `log/demo-smoke/`.

## Scope of this demo

The new drone is an ideal velocity-controlled Gazebo sensor proxy. It includes
collision geometry, RGB-D, LiDAR, IMU, odometry and the same local environmental
field sampler with altitude attenuation. It is suitable for demonstrating aerial
scouting and rover coordination. Rotor dynamics, takeoff/landing, battery models,
PX4/ArduPilot integration and physical sensor drivers are not implemented.
