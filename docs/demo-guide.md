# Rover and drone demonstration

Use [perception-vm-startup.md](perception-vm-startup.md) for a complete Linux VM,
GitHub branch, dependency, build and testing walkthrough. The implementation and
remaining acceptance evidence are listed in [completion-handover.md](completion-handover.md).
The code is a supplied snapshot; no remote branch is assumed to exist yet.

## Start a controlled demonstration

In terminal 1, inside the built repository:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=26
ros2 launch embertrace_bringup embertrace_demo.launch.py scenario:=no_fire
```

This starts both models and sensor pipelines, shared wind/battery simulation, tree
states, mission, search, rover Nav2/SLAM, dashboard and RViz. Both vehicles wait for
START. The drone begins grounded and takes off only when selected and permitted.

In terminal 2, source the same workspace and set the same ROS_DOMAIN_ID:

```bash
ros2 run embertrace_core demo_preflight.py --timeout 90
ros2 service call /embertrace/operator_command \
  embertrace_interfaces/srv/OperatorCommand "{command: START}"
```

Wait for all preflight checks to pass. Default 2.5 m/s wind selects the drone; the
rover remains held. For a rover demonstration, stop the entire launch and restart
with `wind_speed_mps:=7.0`. Only the selected platform advances local confirmation,
search and return. Two patrol loops without a fire lead to automatic return.

For a fire mission restart with `scenario:=fire`. Persistent gas plus heat or
supporting visual evidence drives investigation, verification and alert. The alert
marks its position as a fitted source or an unresolved detection sample. After the
alert hold, the selected robot returns to its measured launch position; the drone
lands. Missing required sensors or unsafe wind pauses the mission. RESUME is
explicit and re-checks readiness. E_STOP remains latched until RESET.

## Controls and scenarios

The dashboard shows the SLAM map, both robot poses, source/sample marker, local
CO2/smoke/temperature/IR/wind, batteries and stale/disconnected status. It provides
six mission controls, alert acknowledgement, hold/release manual buttons and
wind/tree controls. Press Manual control before using the direction buttons;
release or loss of focus stops the manual command. Manual control retains all
selection, sensor, wind and E-stop gates.

| Scenario | Fixture | Expected confirmed-fire behaviour |
| --- | --- | --- |
| no_fire | Healthy tree, ambient field | No confirmed alert |
| fire | Burning tree, smoke/CO2/heat | Confirmation when sufficiently observable |
| dust | Smoke without CO2/fire heat | Anomaly possible, no independent fire confirmation |
| hot_surface | Heated rock without combustion gas | Thermal anomaly, no independent fire confirmation |
| visual_decoy | Flame-colour geometry at ambient temperature | Colour alone cannot confirm |
| dropout | Burning tree with controlled missing raw samples | Selected platform pauses after freshness expires; warmup and RESUME required |

For fire/no_fire/dropout, changing the tree state updates appearance/IR/gas together.
An unavailable scene change withholds raw sensing. Above 12 m/s wind neither vehicle
may deploy; above 5 m/s the drone is held and stable safe wind can select the rover.

## Record acceptance

Stop manual launches before scripts that launch their own Gazebo world:

```bash
bash scripts/validate_all.sh
ROS_DOMAIN_ID=74 python3 scripts/ros_demo_smoke.py
ROS_DOMAIN_ID=74 python3 scripts/ros_perception_campaign.py --output log/perception-recorded-01
ROS_DOMAIN_ID=75 python3 scripts/run_mission_campaign.py --output log/autonomous-fire-01 --trials 10
```

Use a new campaign output directory for every run. Keep failed, negative and missing
frames/trials. For headless rendering prefix with `LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a`.
Native ROS/Gazebo/Qt acceptance remains to be run on the VM. Synthetic filter and
numerical source-fit tests do not replace actual camera/navigation evidence.

The aerial model is an ideal velocity-controlled sensor proxy, including commanded
takeoff/landing and simulated battery behaviour. It does not implement rotor
dynamics, a Parrot/PX4/ArduPilot autopilot or physical sensor drivers.

For independent deployment, add `mission_platform:=rover` or
`mission_platform:=drone` to the main launch. Run preflight with the matching
`--platform rover` or `--platform drone`. The default `auto` and alias `combined`
make both vehicles available and hold the unselected one. Wind limits remain
active in standalone mode; an absent vehicle cannot be selected.
