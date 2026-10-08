#include "embertrace_core/nav2_goal_adapter_node.hpp"
#include <cmath>
#include <memory>

#include "embertrace_interfaces/msg/mission_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace embertrace_core {

// ============================================================================
// CONSTRUCTOR
// ============================================================================

Nav2GoalAdapterNode::Nav2GoalAdapterNode()
: Node("nav2_goal_adapter_node")
{
  // TF2 machinery: listener fills the shared buffer from /tf and /tf_static
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Action client: "navigate_to_pose" is Nav2's default action name
  action_client_ = rclcpp_action::create_client<NavigateToPose>(
    this, "navigate_to_pose");

  // Subscribe to mission state (transient_local for late joiners)
  state_subscription_ = create_subscription<embertrace_interfaces::msg::MissionState>(
    "/embertrace/mission_state", rclcpp::QoS(1).transient_local().reliable(),
    [this](const embertrace_interfaces::msg::MissionState::SharedPtr msg) {
      motion_allowed_ = msg->motion_allowed;
      if (!motion_allowed_) {
        cancelActiveGoal("motion disallowed by mission state");
      }
    });

  // Subscribe to navigation goals
  goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/embertrace/navigation_goal", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg) {
      if (!motion_allowed_) {
        RCLCPP_WARN(get_logger(), "Ignoring goal; mission does not permit motion");
        return;
      }
      sendNav2Goal(*goal_msg);
    });

  // Subscribe to odometry for telemetry/computed metrics
  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    "/husky1/odometry", 50,
    [this](const nav_msgs::msg::Odometry::SharedPtr odom) {
      std::scoped_lock lock(poseMtx_);
      pose_ = odom->pose.pose;
      updateDistAndAngleToGoal();
    });
}

// ============================================================================
// PUBLIC STATIC METHOD
// ============================================================================

void Nav2GoalAdapterNode::normaliseAngle(double & angle)
{
  angle = std::fmod(angle, 2.0 * M_PI);
  if (angle < 0.0) { angle += 2.0 * M_PI; }
}

// ============================================================================
// NAV2 GOAL HANDLING
// ============================================================================

void Nav2GoalAdapterNode::sendNav2Goal(const geometry_msgs::msg::PoseStamped & raw_goal)
{
  if (!action_client_->action_server_is_ready()) {
    RCLCPP_WARN(get_logger(), "Nav2 action server not ready; dropping goal");
    return;
  }

  // 1. Transform goal from its source frame (e.g., husky1_odom) into husky1_map
  geometry_msgs::msg::PoseStamped goal_in_map;
  try {
    goal_in_map.header.stamp = get_clock()->now();
    goal_in_map.header.frame_id = "husky1_map";
    tf_buffer_->transform(raw_goal, goal_in_map, "husky1_map");
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "Could not transform goal into husky1_map: %s", ex.what());
    return;
  }

  // 2. Build and send the action goal
  NavigateToPose::Goal nav_goal;
  nav_goal.pose = goal_in_map;
  nav_goal.behavior_tree.clear();  // Use Nav2's default behavior tree

  RCLCPP_INFO(get_logger(), "Sending Nav2 goal (%.2f, %.2f)",
              nav_goal.pose.pose.position.x, nav_goal.pose.pose.position.y);

  auto send_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  
  send_options.goal_response_callback =
    [this](std::shared_ptr<GoalHandle> handle) {
      if (!handle) {
        RCLCPP_ERROR(get_logger(), "Nav2 rejected the goal");
      } else {
        std::scoped_lock lock(goalHandleMtx_);
        active_goal_handle_ = handle;
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

void Nav2GoalAdapterNode::cancelActiveGoal(const std::string & reason)
{
  if (active_goal_handle_){
    RCLCPP_WARN(get_logger(), "Cancelling Nav2 goal (%s)", reason.c_str());
    
    auto cancel_future = action_client_->async_cancel_goal(active_goal_handle_);
    
    // Clear handle immediately; async_cancel is safe even if already done
    active_goal_handle_.reset();
  }
}

// ============================================================================
// TELEMETRY COMPUTATIONS
// ============================================================================

void Nav2GoalAdapterNode::updateDistAndAngleToGoal()
{
  geometry_msgs::msg::Pose goal;
  { 
    std::scoped_lock lock(goalMtx_); 
    goal = goal_.pose; 
  }

  // Guard against uninitialized goal
  if (goal_.header.stamp.sec == 0) { return; }

  const double dx = goal.position.x - pose_.position.x;
  const double dy = goal.position.y - pose_.position.y;
  dist_to_goal_ = std::hypot(dx, dy);           // Correct: hypot takes 2 args

  angle_to_goal_ = std::atan2(dy, dx);
  
  // Extract yaw from the quaternion, then wrap to [0, 2*pi]
  const double yaw = tf2::getYaw(pose_.orientation);
  heading_error_ = angle_to_goal_ - yaw;
  normaliseAngle(heading_error_);
}

}  // namespace embertrace_core