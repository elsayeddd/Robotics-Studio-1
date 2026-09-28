/**
 * THIS IS ADAPTED FROM SCAFFOLDS
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

#include <cmath>
#include <memory>
#include <mutex>

#include "embertrace_interfaces/msg/mission_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace embertrace_core
{

class Nav2GoalAdapterNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit Nav2GoalAdapterNode()
  : Node("nav2_goal_adapter_node")
  {
     // TF2 machinery: listener fills the shared buffer from /tf and /tf_static
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Action client: "navigate_to_pose" is Nav2's default action name
    action_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    state_subscription_ = create_subscription<embertrace_interfaces::msg::MissionState>(
      "/embertrace/mission_state", rclcpp::QoS(1).transient_local().reliable(),
      [this](const embertrace_interfaces::msg::MissionState::SharedPtr message) {
        motion_allowed_ = message->motion_allowed;
        if (!motion_allowed_) {
          cancelActiveGoal("motion disallowed by mission state");
        }
      });

    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/embertrace/navigation_goal", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr goal) {
        goal_ = goal;
        if (!motion_allowed_) {
          RCLCPP_WARN(get_logger(), "Ignoring goal because the mission does not permit motion");
          return;
        }
        sendNav2Goal(*goal_msg);
      });

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/husky1/odometry", 50,
      [this](const nav_msgs::msg::Odometry::SharedPtr odom) {
        std::scoped_lock lock(poseMtx_);
        pose_ = odom->pose.pose;
        updateDistAndAngleToGoal();
      });

private:
  bool motion_allowed_{false};
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

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

  void cancelActiveGoal(const std::string & reason)
  {
    std::scoped_lock lock(goalHandleMtx_);
    if (active_goal_handle_ && active_goal_handle_->is_active()) {
      RCLCPP_WARN(get_logger(), "Cancelling active Nav2 goal (%s)", reason.c_str());
      action_client_->async_cancel_goal(active_goal_handle_);
      active_goal_handle_.reset();
    }
  }

  void sendNav2Goal(const geometry_msgs::msg::PoseStamped & raw_goal)
  {
    if (!action_client_->action_server_ready()) {
      RCLCPP_WARN(get_logger(), "Nav2 action server not ready; dropping goal");
      return;
    }

    // 1. Transform goal from its source frame (husky1_odom) into husky1_map.
    //    A goal in a moving odometry frame becomes stale the moment the
    //    robot moves, so it must be stamped in the map frame before sending.
    geometry_msgs::msg::PoseStamped goal_in_map;
    try {
      goal_in_map.header.stamp = get_clock()->now();  // make TF lookup use latest data
      goal_in_map.header.frame_id = "husky1_map";
      tf_buffer_->transform(raw_goal, goal_in_map, "husky1_map");
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "Could not transform goal into husky1_map: %s", ex.what());
      return;
    }

    // 2. Build and send the action goal.
    NavigateToPose::Goal nav_goal;
    nav_goal.pose = goal_in_map;
    nav_goal.behavior_tree.clear();  // lets Nav2 use its default behaviour tree

    RCLCPP_INFO(get_logger(), "Sending Nav2 goal (%.2f, %.2f)",
                nav_goal.pose.pose.position.x, nav_goal.pose.pose.position.y);

    auto send_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_options.goal_response_callback =
      [this](std::shared_ptr<GoalHandle> handle) {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "Nav2 rejected the goal");
        } else {
          std::scoped_lock lock(goalHandleMtx_);
          active_goal_handle_ = handle;  // saved so we can cancel later
        }
      };
    send_options.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Navigation succeeded");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(get_logger(), "Navigation cancelled");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Navigation aborted");
            break;
          default:
            RCLCPP_WARN(get_logger(), "Unknown result code");
        }
        std::scoped_lock lock(goalHandleMtx_);
        active_goal_handle_.reset();
      };

    action_client_->async_send_goal(nav_goal, send_options);
  }

  void updateDistAndAngleToGoal()
  {
    geometry_msgs::msg::Pose goal;
    { std::scoped_lock lock(goalMtx_); goal = goal_.pose; }

    // Guard against a default-constructed goal at the origin
    if (goal_.header.stamp.sec == 0) { return; }

    const double dx = goal.position.x - pose_.position.x;
    const double dy = goal.position.y - pose_.position.y;
    dist_to_goal_ = std::hypot(dx, dy);           // hypot(dx, dy)

    angle_to_goal_ = std::atan2(dy, dx);
    // Extract yaw from the quaternion, then wrap to [0, 2*pi]
    const double yaw = tf2::getYaw(pose_.orientation);
    heading_error_ = angle_to_goal_ - yaw;
    normaliseAngle(heading_error_);
  }

  //Assuming angle range is 0 to 2pi
  //Will need to change if range is -pi to pi
  void normaliseAngle(double &angle)
  {
    if (angle < 0.0){
      angle = angle + 2*M_PI;
    }
    else if (angle > 2*M_PI){
      angle = angle - 2*M_PI;
    }
  }

}; 

}  // namespace embertrace_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::Nav2GoalAdapterNode>());
  rclcpp::shutdown();
  return 0;
}

