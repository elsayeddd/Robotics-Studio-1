/**
 * STARTER FILE — intentionally not included in CMakeLists.txt yet.
 *
 * Purpose:
 *   Load patrol_waypoints.yaml and issue one waypoint at a time while the mission
 *   state is PATROL. Investigation goals must take priority when smoke is detected.
 *
 * Recommended implementation:
 *   - use yaml-cpp or convert the waypoint list into ROS parameters;
 *   - subscribe to a Nav2-adapter result topic;
 *   - publish the next geometry_msgs/PoseStamped only after SUCCESS;
 *   - pause the current index during investigation and resume sensibly afterward;
 *   - publish the base waypoint when state becomes RETURN_TO_BASE.
 */

#include <memory>

#include "embertrace_interfaces/msg/mission_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace embertrace_core
{

class PatrolManagerNode : public rclcpp::Node
{
public:
  PatrolManagerNode()
  : Node("patrol_manager_node")
  {
    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/embertrace/navigation_goal", 10);
    state_subscription_ = create_subscription<embertrace_interfaces::msg::MissionState>(
      "/embertrace/mission_state", rclcpp::QoS(1).transient_local().reliable(),
      [this](const embertrace_interfaces::msg::MissionState::SharedPtr message) {
        if (message->state == embertrace_interfaces::msg::MissionState::PATROL) {
          RCLCPP_INFO_ONCE(
            get_logger(), "TODO: load patrol waypoints and publish the first Nav2 goal");
        }
      });
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Subscription<embertrace_interfaces::msg::MissionState>::SharedPtr state_subscription_;
};

}  // namespace embertrace_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::PatrolManagerNode>());
  rclcpp::shutdown();
  return 0;
}

