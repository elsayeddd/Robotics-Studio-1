# EmberTrace

EmberTrace is a ROS 2 forest-fire investigation demonstrator. A wind-selected Husky rover or aerial simulation proxy patrols a forest, fuses local smoke/CO2, temperature and camera evidence, investigates anomalies, reports a location and sensor data, and returns to its measured launch pose.

The repository is designed for a mixed-experience university team. Existing ROS packages handle simulation, localisation and navigation; the project-specific behaviour is divided into small, documented C++ nodes that can be developed and tested independently.

## Project status

The November 5 merge connects both local perception pipelines to wind-based
mission dispatch, autonomous investigation/return, scene states and the dashboard.
See [Completion handover](docs/completion-handover.md) for the C1/C3 requirement
matrix and [integration review](docs/integration-review.md) for current acceptance evidence.

| Capability | Implementation | Validation status |
| --- | --- | --- |
| IR/CO2/smoke/temperature/RGB perception | Independent per-platform filters, image persistence, timestamp alignment and stale watchdogs | Pure regressions passed; native images/recorded trials pending |
| Wind dispatch and mission policy | Stable safe wind selects drone or rover; selected-local confirmation, E-stop, timeouts and ACK | 45 pure policy checks passed; native transport tests provided |
| Patrol, search and return | Nav2 rover route; bounded drone ascent/sweep/investigation/landing; completion and battery return | Numerical tests passed; actual autonomous trials pending |
| Scene state and shared inputs | Healthy/burning/burnt tree rendering/emissions; configurable wind/battery | Eight Python fixture/replay tests passed; Fortress services pending |
| Map/dashboard/manual control | Both platforms, source/sample marker, health, controls and event log | Native Qt/TF review and interaction tests pending |
| Evaluation | Recorder/replay, training-only tuning, six-scenario and ten-autonomous-run scripts | 60 synthetic trials reproduced; target-machine campaigns pending |

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

### 4. Run the integrated demo

```bash
./scripts/run_demo.sh
```

Equivalent launch command:

```bash
ros2 launch embertrace_bringup embertrace_demo.launch.py \
  use_nav2:=true use_drone:=true use_rviz:=true use_dashboard:=true scenario:=fire
```

For independent missions, add `mission_platform:=rover` or
`mission_platform:=drone`. The default `auto` (also named `combined`) starts both
vehicles and activates one according to safe wind conditions. Standalone rover
mode can operate at low wind; drone-only mode holds above 5 m/s instead of selecting
an absent rover. Pass the same `--platform` to preflight.

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

Commands are `START`, `PAUSE`, `RESUME`, `RETURN`, `RESET`, `E_STOP`, `ACK`,
`MANUAL_ON` and `MANUAL_OFF`. Deployment requires stable safe wind and fresh sensors.

## What should appear

- Gazebo opens the `embertrace_forest` world with a Husky rover and a grounded aerial proxy at their launch positions.
- RViz shows the robot, LiDAR, smoke samples and source estimate. The map and investigation goals appear when SLAM is running.
- The dashboard displays a SLAM map, both platforms, smoke/CO2/local temperature/IR, confidence, batteries, health, alerts and controls.
- After START, default 2.5 m/s wind selects the drone, which takes off and patrols. With `wind_speed_mps:=7.0`, the rover owns the mission. The unselected vehicle stays held. Local evidence drives investigation/verification/alert; alert hold, two-loop completion or low battery triggers return.

For manual rover operation, use safe rover-selection wind, START and MANUAL_ON,
then hold the dashboard buttons or remap keyboard input to
`/embertrace/manual_cmd_vel`. Do not publish around the command guards. Follow the
[VM startup guide](docs/perception-vm-startup.md) for exact terminal commands.

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
- Scenarios set the initial scene. The dynamic tree service keeps fire/no_fire/dropout tree state, thermal appearance and gas emissions consistent. The visual_decoy retains flame-like geometry at ambient temperature without a fire field.
- The colour-based camera verifier is a simulation heuristic, and the drone is an ideal velocity-controlled proxy without rotor dynamics or a physical flight controller.
- The synthetic perception report does not establish recorded Gazebo or hardware performance. Complete at least three recorded perception scenarios and ten autonomous seeded trials across the selected platforms on the target machine.

## Origin of supplied material

The simulation structure, Husky model, sensor bridge conventions, forest texture and meshes were adapted from the supplied `41068_ignition_bringup_v1.zip`. See [Original package audit](docs/original-package-audit.md) and [NOTICE](NOTICE.md) before publishing the repository publicly.

## Complete perception integration update

Native IR cameras, explicit CO2, freshness/persistence and evaluation changes are documented in
[perception validation](docs/perception-validation.md).
Follow [VM/GitHub and simulation startup](docs/perception-vm-startup.md) for a fresh machine.
ROS/Gazebo target-machine acceptance remains required.
