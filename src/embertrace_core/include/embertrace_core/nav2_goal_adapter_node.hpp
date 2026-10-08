#ifndef EMBERTRACE_CORE_NAV2_GOAL_ADAPTER_NODE_HPP
#define EMBERTRACE_CORE_NAV2_GOAL_ADAPTER_NODE_HPP

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "embertrace_interfaces/msg/mission_state.hpp"

namespace embertrace_core {

class Nav2GoalAdapterNode : public rclcpp::Node
{
public:
  explicit Nav2GoalAdapterNode();
  ~Nav2GoalAdapterNode() override = default;

  // Public for testing
  static void normaliseAngle(double & angle);

protected:
  friend class Nav2GoalAdapterTest;

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  void sendNav2Goal(const geometry_msgs::msg::PoseStamped & raw_goal);
  void cancelActiveGoal(const std::string & reason);
  void updateDistAndAngleToGoal();

  bool motion_allowed_{false};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

  rclcpp::Subscription<embertrace_interfaces::msg::MissionState>::SharedPtr state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  geometry_msgs::msg::Pose pose_;
  std::mutex poseMtx_;
  geometry_msgs::msg::PoseStamped goal_;
  std::mutex goalMtx_;
  std::shared_ptr<GoalHandle> active_goal_handle_;
  std::mutex goalHandleMtx_;

  double dist_to_goal_{0.0};
  double angle_to_goal_{0.0}; // Angle between the husky's "forwards" direction and facing the goal
  double heading_error_{0.0}; // The difference between the heading and the angle to goal
};

}  // namespace embertrace_core

#endif // EMBERTRACE_CORE_NAV2_GOAL_ADAPTER_NODE_HPP