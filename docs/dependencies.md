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

