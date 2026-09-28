/**
 * Waypoint patrol manager.
 *
 * COMPLETE BASELINE: loads the patrol loop from patrol_waypoints.yaml and
 * drives the rover around it one waypoint at a time while the mission is
 * PATROL. Pauses cleanly (freezing its place in the loop) whenever the
 * mission manager reports any other active state -- most notably
 * INVESTIGATE, so the search planner can take over the shared
 * /embertrace/navigation_goal topic -- and resumes at whichever loaded
 * waypoint is geometrically nearest to the rover's current position rather
 * than blindly continuing the frozen index. That one rule covers both a
 * plain operator PAUSE/RESUME and an operator dismissing a false alarm with
 * START while INVESTIGATE/TRACK/RECOVER/VERIFY was active, since in both
 * cases the rover may not be where it left off. On RETURN_TO_BASE it
 * abandons the loop and drives straight to the named base waypoint instead.
 *
 * Arrival at the current goal is detected locally from odometry against a
 * distance tolerance, because nav2_goal_adapter_node does not yet publish
 * action-result feedback (see scaffolds/README.md item 4). Swap
 * on_odometry()'s role for a result-topic callback once that lands; the
 * sequencing logic itself (embertrace_core/waypoint_patrol.hpp) does not
 * need to change.
 *
 * TODO(student): once nav2_goal_adapter_node reports action results, use
 * that instead of the odometry-distance heuristic; add a stuck/timeout
 * watchdog in case a goal is never reached.
 */

#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "embertrace_core/waypoint_patrol.hpp"
#include "embertrace_interfaces/msg/mission_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace embertrace_core
{

class PatrolManagerNode : public rclcpp::Node
{
public:
  using MissionState = embertrace_interfaces::msg::MissionState;

  PatrolManagerNode()
  : Node("patrol_manager_node")
  {
    const auto waypoints_file = declare_parameter<std::string>("waypoints_file", "");
    const auto base_waypoint_name = declare_parameter<std::string>(
      "base_waypoint_name", "ranger_base");
    const auto arrival_tolerance_m = declare_parameter<double>("arrival_tolerance_m", 0.5);
    const auto odometry_topic = declare_parameter<std::string>(
      "odometry_topic", "/husky1/odometry");
    frame_id_ = declare_parameter<std::string>("frame_id", "husky1_map");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    patrol_ = std::make_unique<WaypointPatrol>(arrival_tolerance_m);

    if (waypoints_file.empty()) {
      RCLCPP_FATAL(
        get_logger(),
        "Parameter 'waypoints_file' was not set; patrol manager has no route to run");
    } else {
      try {
        const PatrolRoute route = load_patrol_route(waypoints_file);
        if (!route.frame_id.empty()) {
          frame_id_ = route.frame_id;
        }
        patrol_->set_waypoints(route.waypoints, base_waypoint_name);
        RCLCPP_INFO(
          get_logger(), "Loaded %zu patrol waypoints from %s (frame %s)",
          route.waypoints.size(), waypoints_file.c_str(), frame_id_.c_str());
      } catch (const std::exception & error) {
        RCLCPP_FATAL(get_logger(), "%s", error.what());
      }
    }

    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/embertrace/navigation_goal", 10);
    state_subscription_ = create_subscription<MissionState>(
      "/embertrace/mission_state", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&PatrolManagerNode::on_mission_state, this, std::placeholders::_1));
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::SensorDataQoS(),
      std::bind(&PatrolManagerNode::on_odometry, this, std::placeholders::_1));
  }

private:
  static PatrolTrigger to_trigger(const std::uint8_t state)
  {
    if (state == MissionState::PATROL) {
      return PatrolTrigger::kPatrol;
    }
    if (state == MissionState::RETURN_TO_BASE) {
      return PatrolTrigger::kReturnToBase;
    }
    return PatrolTrigger::kHold;
  }

  static void set_yaw(geometry_msgs::msg::Pose & pose, const double yaw)
  {
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(yaw / 2.0);
    pose.orientation.w = std::cos(yaw / 2.0);
  }

  void publish_goal(const Waypoint & waypoint)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = frame_id_;
    goal.pose.position.x = waypoint.x;
    goal.pose.position.y = waypoint.y;
    set_yaw(goal.pose, waypoint.yaw);
    goal_publisher_->publish(goal);
    RCLCPP_INFO(
      get_logger(), "Patrol goal: %s (%.2f, %.2f)", waypoint.name.c_str(), waypoint.x,
      waypoint.y);
  }

  void on_mission_state(const MissionState::SharedPtr message)
  {
    if (!patrol_ || !patrol_->has_waypoints()) {
      return;
    }
    const auto goal = patrol_->on_mission_state(to_trigger(message->state));
    if (goal) {
      publish_goal(*goal);
    }
  }

  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!patrol_ || !patrol_->has_waypoints()) {
      return;
    }
    if (message->header.frame_id.empty()) {
      return;
    }
    geometry_msgs::msg::PointStamped position;
    position.header = message->header;
    position.point = message->pose.pose.position;
    position.header.stamp.sec = 0;
    position.header.stamp.nanosec = 0;
    try {
      position = tf_buffer_->transform(position, frame_id_);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for patrol pose transform: %s",
        error.what());
      return;
    }
    const auto goal = patrol_->update_pose(position.point.x, position.point.y);
    if (patrol_->loops_completed() > loops_logged_) {
      loops_logged_ = patrol_->loops_completed();
      RCLCPP_INFO(get_logger(), "Patrol loop %zu complete", loops_logged_);
    }
    if (goal) {
      publish_goal(*goal);
    }
  }

  std::string frame_id_{"husky1_map"};
  std::size_t loops_logged_{0};
  std::unique_ptr<WaypointPatrol> patrol_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Subscription<MissionState>::SharedPtr state_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
};

}  // namespace embertrace_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::PatrolManagerNode>());
  rclcpp::shutdown();
  return 0;
}
