# C++ node development notes

Each source file starts with its completed baseline and `TODO(student)` work. Keep one executable per file until the team is comfortable with ROS composition.

## Recommended ownership

| File | Purpose | Suggested owner |
| --- | --- | --- |
| `plume_model.cpp` | Ground-truth mathematical environment | Simulation member |
| `environment_simulator_node.cpp` | Robot-position sampling and raw readings | Simulation member |
| `camera_verifier_node.cpp` | Visual fire evidence | Perception member |
| `perception_node.cpp` | Filtering and evidence fusion | Perception member |
| `mission_manager_node.cpp` | Authoritative decision state machine | Autonomy member |
| `search_planner_node.cpp` | Surge/cast search goals | Autonomy member |
| `heatmap_node.cpp` | RViz operator visualisation | UI/testing member |
| `nav2_goal_adapter_node.cpp` | Send commands over nav2 | Autonomy/control member |

## Nav2 adapter to add

Begin with `../scaffolds/nav2_goal_adapter_node.cpp` when the navigation package is running reliably:

1. Include `rclcpp_action/rclcpp_action.hpp` and `nav2_msgs/action/navigate_to_pose.hpp`.
2. Subscribe to `/embertrace/navigation_goal`.
3. Create an action client for `/husky1/navigate_to_pose`.
4. Transform a goal into `husky1_map` if it arrives in `husky1_odom`.
5. Send only one active goal and cancel it before replacing it.
6. Subscribe to `/embertrace/mission_state` and cancel immediately when motion is not allowed.
7. Publish a small status message so the mission manager can react to success/failure.
8. Add `rclcpp_action`, `nav2_msgs`, `tf2_ros` and `tf2_geometry_msgs` dependencies to both manifests.

Do not put Nav2 action code directly into the mission manager. Keeping it separate makes state-machine testing possible without a running simulator.
