# Dependencies

## Operating-system baseline

| Dependency | Required version | Reason |
| --- | --- | --- |
| Ubuntu | 22.04 LTS | Binary platform for ROS 2 Humble and Gazebo Fortress |
| ROS 2 | Humble desktop | C++ middleware, messages, launch, RViz and tools |
| Ignition/Gazebo | Fortress | Simulator used by the supplied teaching package |
| Compiler | GCC supporting C++17 | All custom nodes use C++17 |
| Qt | Qt 5 Widgets | Ranger dashboard |

## ROS packages

- `ros_ign` (`ros_ign_gazebo` and `ros_ign_bridge` shims);
- `navigation2` and `nav2_bringup`;
- `slam_toolbox`;
- `robot_localization`;
- `robot_state_publisher`;
- `xacro`;
- `rviz2`;
- standard message packages installed with ROS desktop;
- `teleop_twist_keyboard` for early manual testing.

The authoritative dependency declarations are the five `package.xml` files. `rosdep` reads them and installs missing packages.

## Why Humble/Fortress is pinned

The supplied package uses `ros_ign_gazebo`, `ros_ign_bridge`, `ign gazebo` conventions and Fortress-era Gazebo plugin names. Humble binaries pair with Fortress, while the `ros_ign` names are maintained as compatibility shims. Migrating to Jazzy/Harmonic is possible but should be a separate team decision because launch package names, plugin names and message types can change together.

## Optional development tools

```bash
sudo apt-get install -y clang-format cppcheck gdb
```

These are helpful but not needed to run the simulation.


## Headless acceptance and VM memory

Install `xvfb` and `libgl1-mesa-dri` to run sensor rendering without a desktop.
Visible WSLg launches also use Xvfb for the sensor server; `xvfb` is declared in
the simulator package so rosdep installs it:

```bash
sudo apt-get install -y xvfb libgl1-mesa-dri
LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=2 xvfb-run -a python3 scripts/ros_demo_smoke.py --platform rover
```

`python3-yaml` is required by repository validation and the trial tools. The build
helper limits both Colcon package workers and Make jobs. Set `MAKEFLAGS` explicitly
if changing this limit; Colcon otherwise adds CPU-count compiler parallelism even
when `CMAKE_BUILD_PARALLEL_LEVEL` is set.
