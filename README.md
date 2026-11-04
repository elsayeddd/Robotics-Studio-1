# EmberTrace

EmberTrace is a ROS 2 forest-fire investigation demonstrator. A simulated Husky rover patrols a forest, measures a noisy smoke plume, decides when to leave patrol and investigate, estimates where the source lies, and reports the result to a ranger-facing interface.

The repository is designed for a mixed-experience university team. Existing ROS packages handle simulation, localisation and navigation; the project-specific behaviour is divided into small, documented C++ nodes that can be developed and tested independently.

## Project status

The navigation, patrol and source estimation modules are integrated with timestamp-aware perception, sensor health and an aerial scout on `main`. See [Demo guide](docs/demo-guide.md) and [Perception validation](docs/perception-validation.md) for commands, measured synthetic results and outstanding recorded-trial acceptance.

| Capability | Status | Notes |
| --- | --- | --- |
| Forest world and Husky sensor model | Implemented | Self-contained local trees, rocks, logs, base station, fire source and textured terrain |
| ROS–Gazebo bridge | Implemented | Odometry, IMU, LiDAR, RGB-D camera, commands and simulation clock |
| Mathematical smoke and temperature field | Implemented baseline | Noisy Gaussian plume driven by robot position and configurable wind |
| Environmental perception | Implemented; recorded trials pending | Timestamp-aware filtering/hysteresis, stale/dropout recovery, validity and guarded confirmation |
| Mission decision state machine | Implemented baseline | Idle, patrol, investigate, track, recover, verify, alert, return and emergency stop |
| Search and source estimation | Implemented baseline | Bounded sample history, source/covariance fit, map-safe goals and timed recovery; unit tests included |
| Sample heatmap in RViz | Implemented baseline | Colour-coded smoke samples and current estimate |
| Ranger dashboard | Implemented baseline | C++/Qt status display and mission controls |
| Nav2 goal execution | Implemented baseline; rover validation pending | Namespaced `NavigateToPose` client transforms and bounds-checks goals, cancels on mission changes and publishes result status |
| Camera fire verification | Implemented simulation heuristic | Connected colour patches, valid stamped cues and consecutive frames; real-image validation pending |
| Aerial scout | Implemented demo proxy | Velocity-controlled drone, RGB-D/LiDAR/IMU, local environment samples and rover investigation cues |
| Sensor health and motion guard | Implemented | All sensor streams monitored; missing readiness/mission/command heartbeat inhibits motion |
| Perception evaluation | Implemented | Six scenarios, CSV recorder/evaluator, threshold sweep, synthetic confusion matrices and latency |
| Patrol route manager | Implemented baseline; rover validation pending | YAML route loading, sequential goals, pause/resume and return to base; odometry is transformed into the route frame for arrival checks |
| Automated end-to-end tests | Partial | Plume, search and patrol unit tests are included; full Gazebo/Nav2 rover trials remain |

See [Development roadmap](docs/development-roadmap.md) for ownership-ready tasks and acceptance criteria.

## Mission modes

Use the same integrated launch with a different primary platform:

```bash
./scripts/run_demo.sh mission_platform:=rover
./scripts/run_demo.sh mission_platform:=drone
./scripts/run_demo.sh mission_platform:=combined
```

Standalone drone mode starts only the aircraft and its sensor pipeline. It patrols,
investigates local evidence, estimates the source, reports alerts and returns to its
launch position. Combined mode uses the aircraft as a scout; rover-local evidence
owns fire confirmation. `platform_select.launch.py platform:=auto` selects an
entire standalone mission from `wind_speed_mps`, and live sensor wind status gates
both vehicles during operation.

See [integration review and test evidence](docs/integration-review.md).

## Supported platform

The supplied teaching package used the `ros_ign_*` API and Ignition Fortress. EmberTrace therefore targets:

- Ubuntu 22.04 LTS, including Ubuntu 22.04 under WSL2;
- ROS 2 Humble;
- Ignition/Gazebo Fortress;
- C++17;
- `colcon` and `ament_cmake`.

Humble's `ros_ign` packages are compatibility shims for `ros_gz`, so the supplied launch conventions remain usable. Do not install a different Gazebo major release alongside Fortress unless the team deliberately migrates the project.

## Quick start

### 1. Install ROS 2

Follow the official [ROS 2 Humble Ubuntu installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Select the desktop installation.

### 2. Install project dependencies

From the repository root:

```bash
./scripts/install_dependencies.sh
```

The script checks for Ubuntu 22.04 and ROS 2 Humble, installs the declared packages, and runs `rosdep`. It intentionally stops with guidance instead of attempting to install ROS itself.

### 3. Build

```bash
source /opt/ros/humble/setup.bash
./scripts/build.sh
source install/setup.bash
```

### 4. Run the integrated demo

```bash
./scripts/run_demo.sh
```

Equivalent launch command:

```bash
ros2 launch embertrace_bringup embertrace_demo.launch.py \
  use_nav2:=true use_drone:=true use_rviz:=true use_dashboard:=true scenario:=fire
```

Run the read-only readiness check in a second sourced terminal before START:

```bash
ros2 run embertrace_core demo_preflight.py --timeout 60
```

Begin with `scenario:=no_fire` to inspect patrol and drone motion; then restart
with `scenario:=fire`. See [Demo guide](docs/demo-guide.md) for false alarms,
dropout, recording and isolated sensor/teleoperation modes.

### 5. Send an operator command without the dashboard

```bash
ros2 service call /embertrace/operator_command \
  embertrace_interfaces/srv/OperatorCommand "{command: 'START'}"
```

Valid commands are `START`, `PAUSE`, `RESUME`, `RETURN`, `RESET` and `E_STOP`.

## What should appear

- Gazebo opens the `embertrace_forest` world with a Husky rover at the ranger base and an aerial scout hovering above its base.
- RViz shows the robot, LiDAR, smoke samples and source estimate. The map and investigation goals appear when SLAM is running.
- The EmberTrace dashboard displays mission state, smoke, temperature, confidence and alert information.
- After `START`, the state is `PATROL`. Moving the robot into the plume causes `INVESTIGATE` and then `TRACK`; sufficiently strong combined evidence causes `VERIFY` and `ALERT`.

For a launch with `use_nav2:=false`, START the mission, then use teleoperation to move through the plume:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/husky1/autonomy_cmd_vel
```

## Repository layout

```text
embertrace_ros2/
├── docs/                       Architecture, interfaces, simulation and team guidance
├── scripts/                    Dependency, build, run and validation helpers
└── src/
    ├── embertrace_interfaces/  Custom messages and operator service
    ├── embertrace_sim/         Gazebo world, Husky, bridges, Nav2/SLAM config and RViz
    ├── embertrace_core/        C++ sensing, perception, decisions, search and visualisation
    ├── embertrace_ui/          C++/Qt operator dashboard
    └── embertrace_bringup/     One launch file and project-level parameters
```

The detailed data flow and node responsibilities are in [System architecture](docs/architecture.md). Topic and service names are defined in [Interface contract](docs/interface-contract.md).

For the exact installation set and common runtime problems, see [Dependencies](docs/dependencies.md) and [Troubleshooting](docs/troubleshooting.md).

## Recommended teammate workflow

1. Read [Start here for teammates](docs/teammate-start-here.md).
2. Claim one task from [Development roadmap](docs/development-roadmap.md).
3. Create a branch such as `feature/nav2-goal-adapter`.
4. Build only the package you are changing during iteration:

   ```bash
   colcon build --symlink-install --packages-select embertrace_core
   ```

5. Run `./scripts/validate_repo.py` and package tests before opening a pull request.

## Important limitations

- Smoke is a mathematical field, not computational fluid dynamics. This keeps the system deterministic and achievable in 12 weeks.
- The environment simulator knows the hidden fire coordinates, but publishes only local sensor readings. Decision nodes must never subscribe to ground truth.
- Scenario switches remove the visible fire model for no_fire, dust and hot_surface trials. The visual_decoy scenario deliberately retains it without a fire field.
- The colour-based camera verifier is a simulation heuristic, and the drone is an ideal velocity-controlled proxy without rotor dynamics or a physical flight controller.
- The synthetic perception report does not establish recorded Gazebo or hardware performance. Complete at least three recorded perception scenarios and ten source-localisation rover trials on the target machine.

## Origin of supplied material

The simulation structure, Husky model, sensor bridge conventions, forest texture and meshes were adapted from the supplied `41068_ignition_bringup_v1.zip`. See [Original package audit](docs/original-package-audit.md) and [NOTICE](NOTICE.md) before publishing the repository publicly.

## Infrared and CO2 sensing

Native IR cameras, explicit CO2, freshness/persistence and evaluation changes are documented in
[perception validation](docs/perception-validation.md).
Follow [VM/GitHub and simulation startup](docs/perception-vm-startup.md) for a fresh machine.
See the integration review for the validation actually completed.
