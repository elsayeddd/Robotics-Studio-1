# EmberTrace

EmberTrace is a ROS 2 forest-fire investigation demonstrator. A simulated Husky rover patrols a forest, measures a noisy smoke plume, decides when to leave patrol and investigate, estimates where the source lies, and reports the result to a ranger-facing interface.

The repository is designed for a mixed-experience university team. Existing ROS packages handle simulation, localisation and navigation; the project-specific behaviour is divided into small, documented C++ nodes that can be developed and tested independently.

## Project status

All three feature branches are represented on `main`. Runtime acceptance of the complete autonomous rover remains outstanding.

| Capability | Status | Notes |
| --- | --- | --- |
| Forest world and Husky sensor model | Implemented | Self-contained local trees, rocks, logs, base station, fire source and textured terrain |
| ROS–Gazebo bridge | Implemented | Odometry, IMU, LiDAR, RGB-D camera, commands and simulation clock |
| Mathematical smoke and temperature field | Implemented baseline | Noisy Gaussian plume driven by robot position and configurable wind |
| Environmental perception | Implemented baseline | Exponential filtering, thresholding and confidence calculation |
| Mission decision state machine | Implemented baseline | Idle, patrol, investigate, track, recover, verify, alert, return and emergency stop |
| Search and source estimation | Implemented baseline | Bounded sample history, source/covariance fit, map-safe goals and timed recovery; unit tests included |
| Sample heatmap in RViz | Implemented baseline | Colour-coded smoke samples and current estimate |
| Ranger dashboard | Implemented baseline | C++/Qt status display and mission controls |
| Nav2 goal execution | Implemented baseline; rover validation pending | Namespaced `NavigateToPose` client transforms and bounds-checks goals, cancels on mission changes and publishes result status |
| Camera fire verification | Scaffolded | Simple colour cue works; robust CV model and test set remain outstanding |
| Patrol route manager | Implemented baseline; rover validation pending | YAML route loading, sequential goals, pause/resume and return to base; odometry is transformed into the route frame for arrival checks |
| Automated end-to-end tests | Partial | Plume, search and patrol unit tests are included; full Gazebo/Nav2 rover trials remain |

See [Development roadmap](docs/development-roadmap.md) for ownership-ready tasks and acceptance criteria.

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

### 4. Run the complete scaffold

```bash
./scripts/run_demo.sh
```

Equivalent launch command:

```bash
ros2 launch embertrace_bringup embertrace_demo.launch.py \
  use_nav2:=false use_rviz:=true use_dashboard:=true
```

Start with `use_nav2:=false`. This verifies Gazebo, the sensor bridge, plume simulation, state machine, heatmap and dashboard. The search planner publishes source estimates from samples, but only publishes goals when SLAM supplies a map and TF.

### 5. Send an operator command without the dashboard

```bash
ros2 service call /embertrace/operator_command \
  embertrace_interfaces/srv/OperatorCommand "{command: 'START'}"
```

Valid commands are `START`, `PAUSE`, `RESUME`, `RETURN`, `RESET` and `E_STOP`.

## What should appear

- Gazebo opens the `embertrace_forest` world with a Husky rover at the ranger base.
- RViz shows the robot, LiDAR, smoke samples and source estimate. The map and investigation goals appear when SLAM is running.
- The EmberTrace dashboard displays mission state, smoke, temperature, confidence and alert information.
- After `START`, the state is `PATROL`. Moving the robot into the plume causes `INVESTIGATE` and then `TRACK`; sufficiently strong combined evidence causes `VERIFY` and `ALERT`.

For the baseline launch without Nav2, use teleoperation to move through the plume:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/husky1/cmd_vel
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
- The visible fire model helps demonstration and camera work. It can be hidden for blind trials using the `show_fire_marker` parameter and a future world-state service.
- The current colour-based camera verifier is deliberately simple and must not be presented as a real wildfire detector.
- The search baseline was built with ROS 2 Humble before the navigation and patrol branches were integrated. Build and test the combined package, full Gazebo/Nav2 launch and ten rover trials on the target machine.

## Origin of supplied material

The simulation structure, Husky model, sensor bridge conventions, forest texture and meshes were adapted from the supplied `41068_ignition_bringup_v1.zip`. See [Original package audit](docs/original-package-audit.md) and [NOTICE](NOTICE.md) before publishing the repository publicly.
