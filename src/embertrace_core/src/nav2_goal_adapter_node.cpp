/** Bridge mission goals to the namespaced Nav2 NavigateToPose action. */

#include <chrono>
#include "embertrace_core/nav2_goal_adapter_node.hpp"
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "embertrace_interfaces/msg/mission_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace embertrace_core {

// ============================================================================
// CONSTRUCTOR
// ============================================================================

Nav2GoalAdapterNode::Nav2GoalAdapterNode()
: Node("nav2_goal_adapter_node")
using namespace std::chrono_literals;

namespace embertrace_core
{
  const auto action_name = declare_parameter<std::string>(
    "action_name", "/husky1/navigate_to_pose");
  map_frame_ = declare_parameter<std::string>("map_frame", "husky1_map");
  max_goal_age_s_ = declare_parameter<double>("max_goal_age_s", 5.0);
  pending_timeout_s_ = declare_parameter<double>("pending_timeout_s", 15.0);
  min_x_ = declare_parameter<double>("world_min_x", -18.5);
  max_x_ = declare_parameter<double>("world_max_x", 18.5);
  min_y_ = declare_parameter<double>("world_min_y", -18.5);
  max_y_ = declare_parameter<double>("world_max_y", 18.5);

  
  // TF2 machinery: listener fills the shared buffer from /tf and /tf_static
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Action client: "navigate_to_pose" is Nav2's default action name
  action_client_ = rclcpp_action::create_client<NavigateToPose>(
    this, action_name);    

  status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/embertrace/navigation_status", 10);
  
  // Subscribe to mission state (transient_local for late joiners)
  state_subscription_ = create_subscription<MissionState>(
    "/embertrace/mission_state", rclcpp::QoS(1).transient_local().reliable(),
    [this](const MissionState::SharedPtr msg) {
      if (!msg->motion_allowed || state_ != msg->state) {
        pending_goal_.reset();
        ++generation_;
        cancelActiveGoal();
      }
      state_ = state->state;
      motion_allowed_ = state->motion_allowed;
    });

  // Subscribe to navigation goals
  goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/embertrace/navigation_goal", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg) {
      if (!motion_allowed_ || goal->header.frame_id.empty() ||
        !std::isfinite(goal->pose.position.x) ||
        !std::isfinite(goal->pose.position.y))
      {
        publish_status("REJECTED");
        RCLCPP_WARN(get_logger(), "Ignoring goal; mission does not permit motion");
        return;
      }
      const rclcpp::Time stamp(goal->header.stamp);
      if (stamp.nanoseconds() > 0) {
        const double age = (now() - stamp).seconds();
        if (age > max_goal_age_s_ || age < -1.0) {
          publish_status("STALE");
          return;
        }
      }
      ++generation_;
      cancelActiveGoal();
      pending_goal_ = *goal;
      pending_since_ = now();
      sendNav2Goal(*goal_msg);  
    });

  retry_timer_ = create_wall_timer(500ms, [this]() {sendNav2Goal(*goal_msg);}); //goal_msg might be unreachable from here


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
  if (!pending_goal_ || !motion_allowed_) {
    return;
  }
  if ((now() - pending_since_).seconds() > pending_timeout_s_) {
    pending_goal_.reset();
    publish_status("STALE");
    return;
  }
  if (!action_client_->wait_for_action_server(0s)) { //check for redundancy
    return;  // Nav2 starts after the other core nodes.
  }
  if (!action_client_->action_server_is_ready()) { //check for redundancy
    RCLCPP_WARN(get_logger(), "Nav2 action server not ready; dropping goal");
    return;
  }

  
  geometry_msgs::msg::PoseStamped goal_in_map;

  // 1. Same-frame check
  if (raw_goal.header.frame_id == "husky1_map") {
    goal_in_map = raw_goal;
  } else {
    // 2. Use zero-stamp for latest TF
    geometry_msgs::msg::PoseStamped temp = raw_goal;
    temp.header.stamp = tf2::TimePointZero;
    
    try {
      goal_in_map = tf_buffer_->transform(temp, "husky1_map");
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                          "Skipping goal; TF unavailable: %s", ex.what());
      return;
    }
  }

  // 3. Validate coordinates
  const auto &p = goal_in_map.pose.position;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
    p.x < min_x_ || p.x > max_x_ || p.y < min_y_ || p.y > max_y_)
  {
    pending_goal_.reset();
    publish_status("OUT_OF_BOUNDS");
    RCLCPP_ERROR(get_logger(), "Invalid goal coordinates (NaN/Inf)");
    return;
  }  
  
  // 4. Stamp before sending
  goal_in_map.header.stamp = get_clock()->now();

  NavigateToPose::Goal nav_goal;
  nav_goal.pose = goal_in_map;

  RCLCPP_INFO(get_logger(), "Sending Nav2 goal (%.2f, %.2f)", p.x, p.y);
    
  auto send_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  
  send_options.goal_response_callback =
    [this](std::shared_ptr<GoalHandle> handle) {
      if (!handle) {
        RCLCPP_ERROR(get_logger(), "Nav2 rejected the goal");
        if (request_generation == generation_) {publish_status("REJECTED");} //Adam's

      }
      ///////// Adam's 
      if (request_generation != generation_ || !motion_allowed_) {
        action_client_->async_cancel_goal(handle);
        return;
      }
      active_goal_handle_ = handle;
      publish_status("ACCEPTED");
    };
  
  send_options.result_callback =
    [this](const GoalHandle::WrappedResult & result) {
      
      if (request_generation != generation_) {return;} // These 2 lines
      active_goal_handle_.reset();                     // Are Adam's
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(get_logger(), "Navigation succeeded");
          publish_status("SUCCEEDED");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_INFO(get_logger(), "Navigation cancelled");
          publish_status("CANCELED");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(get_logger(), "Navigation aborted");
          publish_status("ABORTED");
          break;
        default:
          RCLCPP_WARN(get_logger(), "Unknown result code");
          publish_status("UNKNOWN");
      }
      std::scoped_lock lock(goalHandleMtx_);
      active_goal_handle_.reset();
    };

  action_client_->async_send_goal(nav_goal, send_options);
  publish_status("SENT");
}

/// @brief Adam's
/// @param value: the status that is 
/// published to "/embertrace/navigation_status"
/// As far as I know nothing else is subbed to that topic 
void Nav2GoalAdapterNode::publish_status(const char * value)
{
  std_msgs::msg::String status;
  status.data = value;
  status_publisher_->publish(status);
}


void Nav2GoalAdapterNode::cancelActiveGoal(const std::string & reason)
{
  if (active_goal_handle_){ //& active_goal_handle_->is_active()){
    RCLCPP_WARN(get_logger(), "Cancelling Nav2 goal (%s)", reason.c_str());
    
    auto cancel_future = action_client_->async_cancel_goal(active_goal_handle_);
    publish_status("CANCEL_REQUESTED");
  }
  // Clear handle immediately; async_cancel is safe even if already done
  active_goal_handle_.reset();
  
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
