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
  using MissionState = embertrace_interfaces::msg::MissionState;

  void sendNav2Goal(const geometry_msgs::msg::PoseStamped & raw_goal);
  void cancelActiveGoal(const std::string & reason);
  void updateDistAndAngleToGoal();
  void publish_status(const char * value);//Adam's

  bool motion_allowed_{false};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_; // Adam's

  rclcpp::Subscription<embertrace_interfaces::msg::MissionState>::SharedPtr state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_; // Adam's

  geometry_msgs::msg::Pose pose_;
  std::mutex poseMtx_;
  geometry_msgs::msg::PoseStamped goal_;
  std::mutex goalMtx_;
  std::shared_ptr<GoalHandle> active_goal_handle_;
  std::mutex goalHandleMtx_;

  double dist_to_goal_{0.0};
  double angle_to_goal_{0.0}; // Angle between the husky's "forwards" direction and facing the goal
  double heading_error_{0.0}; // The difference between the heading and the angle to goal

  ////// From Adam's commit /////
  std::uint8_t state_{255};
  std::uint64_t generation_{0};
  double max_goal_age_s_{5.0};
  double pending_timeout_s_{15.0};
  double min_x_{-18.5}, max_x_{18.5}, min_y_{-18.5}, max_y_{18.5};
  std::string map_frame_;
  std::optional<geometry_msgs::msg::PoseStamped> pending_goal_;
  rclcpp::Time pending_since_{0, 0, RCL_ROS_TIME};

  std::shared_ptr<GoalHandle> active_goal_handle_;
  
  rclcpp::TimerBase::SharedPtr retry_timer_;
};

}  // namespace embertrace_core

#endif // EMBERTRACE_CORE_NAV2_GOAL_ADAPTER_NODE_HPP