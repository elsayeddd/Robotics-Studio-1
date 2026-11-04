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

## Gazebo is slow or Ogre2 crashes under WSLg

Use the normal launch with `gui_render_engine:=ogre` (the WSLg default). It runs
the native Ogre2 sensor renderer on software GL under Xvfb and opens a separate
GPU Ogre desktop viewer. This setup was tested with the RTX 2060/D3D12 renderer
and the larger forest. It keeps native thermal images available without forcing
the desktop viewer onto software rendering.

The `GL3PlusTextureGpu::copyTo` exception matches the upstream
[Ogre2/WSL rendering report](https://github.com/gazebosim/gz-rendering/issues/662).
Switching only the viewer engine does not fix that exception in a GPU sensor
server. On a machine with compatible Ogre2 drivers, `gui_render_engine:=ogre2`
uses the normal combined process. For headless tests, use
`LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a` and `gazebo_gui:=false`.

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

