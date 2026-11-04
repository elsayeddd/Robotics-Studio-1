# Troubleshooting

## `ros2` is not found

```bash
source /opt/ros/humble/setup.bash
```

Add that line to your shell startup file only after confirming the Humble installation works.

## EmberTrace packages are not found

Build and source the workspace in the same terminal:

```bash
./scripts/build.sh
source install/setup.bash
ros2 pkg list | grep embertrace
```

## Gazebo opens with missing ground or models

Launch through `embertrace_sim` or `embertrace_bringup`; the launch file sets the model resource path. Starting the SDF directly without setting `IGN_GAZEBO_RESOURCE_PATH` will not resolve `model://forest_ground` and other local assets.

## No camera/LiDAR data

Check the Ignition bridge and Gazebo topics:

```bash
ros2 node list | grep bridge
ros2 topic hz /husky1/scan
ros2 topic hz /husky1/camera/image
ign topic -l | grep husky1
```

If the robot never spawned, inspect the `spawn_husky1` output first.

## Dashboard does not open under WSL

Windows 11 with WSLg should display Qt applications directly. On systems without GUI forwarding, launch with:

```bash
ros2 launch embertrace_bringup embertrace_demo.launch.py use_dashboard:=false
```

The operator service and RViz remain available.

## RViz reports transform errors

Start with fixed frame `husky1_odom`. `husky1_map` is not available until SLAM runs. Confirm:

```bash
ros2 topic echo /husky1/tf --once
ros2 run tf2_ros tf2_echo husky1_odom husky1_base_link \
  --ros-args -r /tf:=/husky1/tf -r /tf_static:=/husky1/tf_static
```

## No smoke values

Check odometry first. The environment node intentionally waits for `/husky1/odometry`:

```bash
ros2 topic echo /husky1/odometry --once
ros2 topic echo /embertrace/raw_environment
```

## Nav2 does not move the rover

Run preflight with `--platform rover` or `--platform combined`. Check that Nav2 lifecycle nodes are active, `/husky1/navigate_to_pose` exists, and `/husky1/map` plus the namespaced TF tree are available. Inspect `/embertrace/navigation_status` for stale, rejected or out-of-bounds goals. The rover velocity guard also requires fresh sensor, mission, command and safe-wind heartbeats. Send START only after preflight passes.

For the standalone aircraft use `mission_platform:=drone` and preflight `--platform drone`. It uses its aerial controller and does not require a ground Nav2 action server.

