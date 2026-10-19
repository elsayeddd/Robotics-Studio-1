#ifndef EMBERTRACE_CORE__NAV2_GOAL_ADAPTER_NODE_HPP_
#define EMBERTRACE_CORE__NAV2_GOAL_ADAPTER_NODE_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <chrono>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "embertrace_interfaces/msg/mission_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace embertrace_core {

class Nav2GoalAdapterNode : public rclcpp::Node {
public:
  Nav2GoalAdapterNode();
  ~Nav2GoalAdapterNode() = default;

  static void normaliseAngle(double & angle);

private:
  // ========================================================================
  // ROBOT CONFIGURATION PARAMETERS (all customizable at launch time)
  // ========================================================================
  std::string robot_namespace_;           // e.g., "husky1", "parrot1", ""
  std::string odom_topic_;                // e.g., "/odom", "/husky1/odometry"
  std::string goal_topic_;                // e.g., "/navigation_goal"
  std::string mission_state_topic_;       // e.g., "/mission_state"
  std::string action_name_;               // e.g., "/navigate_to_pose"
  std::string map_frame_;                 // e.g., "map", "husky1_map"
  std::string odom_frame_;                // e.g., "odom", "husky1_odom"


  // ========================================================================
  // STATE MANAGEMENT (thread-safe)
  // ========================================================================
  mutable std::mutex poseMtx_;
  geometry_msgs::msg::Pose pose_;
  double dist_to_goal_{0.0};
  double angle_to_goal_{0.0};
  double heading_error_{0.0};

  mutable std::mutex goalMtx_;
  std::optional<geometry_msgs::msg::PoseStamped> pending_goal_;
  rclcpp::Time pending_since_;
  geometry_msgs::msg::PoseStamped goal_;  // Last accepted goal for telemetry
  std::uint64_t generation_{0};

  mutable std::mutex goalHandleMtx_;
  rclcpp_action::Client<
    nav2_msgs::action::NavigateToPose>::GoalHandle::SharedPtr
    active_goal_handle_;

  // TF2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  double max_goal_age_s_;
  double pending_timeout_s_;
  double min_x_, max_x_, min_y_, max_y_;
  std::atomic<bool> motion_allowed_{false};
  int64_t state_{-1};

  // Action client
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;

  // Subscriptions
  rclcpp::Subscription<embertrace_interfaces::msg::MissionState>::SharedPtr
    state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    goal_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  // Publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;

  // Timer - created once, reused across goals
  rclcpp::TimerBase::SharedPtr retry_timer_;

  // ========================================================================
  // PRIVATE METHODS
  // ========================================================================
  void sendNav2Goal(const geometry_msgs::msg::PoseStamped & raw_goal);
  void publish_status(const std::string & status);
  void cancelActiveGoal(const std::string & reason);
  void updateDistAndAngleToGoal();
};

}  // namespace embertrace_core

#endif  // EMBERTRACE_CORE__NAV2_GOAL_ADAPTER_NODE_HPP_