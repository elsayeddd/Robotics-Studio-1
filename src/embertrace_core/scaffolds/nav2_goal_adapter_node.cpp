/**
 * STARTER FILE — intentionally not included in CMakeLists.txt yet.
 *
 * Purpose:
 *   Convert /embertrace/navigation_goal messages into Nav2 NavigateToPose actions.
 *
 * Before enabling this file:
 *   1. Add rclcpp_action, nav2_msgs, tf2_ros and tf2_geometry_msgs dependencies.
 *   2. Transform goals from husky1_odom into husky1_map.
 *   3. Cancel the active action whenever MissionState.motion_allowed is false.
 *   4. Publish result status for the patrol and mission managers.
 *   5. Add the executable to embertrace_core/CMakeLists.txt.
 *
 * This safe starter subscribes and logs only. It cannot accidentally command motion.
 */

#include <memory>

#include "embertrace_interfaces/msg/mission_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace embertrace_core
{

class Nav2GoalAdapterNode : public rclcpp::Node
{
public:
  Nav2GoalAdapterNode()
  : Node("nav2_goal_adapter_node")
  {
    state_subscription_ = create_subscription<embertrace_interfaces::msg::MissionState>(
      "/embertrace/mission_state", rclcpp::QoS(1).transient_local().reliable(),
      [this](const embertrace_interfaces::msg::MissionState::SharedPtr message) {
        motion_allowed_ = message->motion_allowed;
        if (!motion_allowed_) {
          // TODO(student): call async_cancel_goal on the active Nav2 goal.
          RCLCPP_WARN(get_logger(), "Motion disabled; Nav2 goal must be cancelled");
        }
      });

    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/embertrace/navigation_goal", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr goal) {
        if (!motion_allowed_) {
          RCLCPP_WARN(get_logger(), "Ignoring goal because the mission does not permit motion");
          return;
        }
        RCLCPP_INFO(
          get_logger(), "TODO: send Nav2 goal (%.2f, %.2f) from frame %s",
          goal->pose.position.x, goal->pose.position.y, goal->header.frame_id.c_str());
        // TODO(student): transform goal, create NavigateToPose::Goal and call async_send_goal.
      });
  }

private:
  bool motion_allowed_{false};
  rclcpp::Subscription<embertrace_interfaces::msg::MissionState>::SharedPtr state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
};

}  // namespace embertrace_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::Nav2GoalAdapterNode>());
  rclcpp::shutdown();
  return 0;
}

