/** Bridge mission goals to the namespaced Nav2 NavigateToPose action. */

#include "embertrace_core/nav2_goal_adapter_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "embertrace_interfaces/msg/mission_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace embertrace_core {

// ============================================================================
// CONSTRUCTOR
// ============================================================================

Nav2GoalAdapterNode::Nav2GoalAdapterNode() : Node("nav2_goal_adapter_node") {
  using namespace std::chrono_literals;

  // ==========================================================================
  // ROBOT CONFIGURATION PARAMETERS
  // Change these via launch file to adapt to different robots
  // ==========================================================================

  // Namespace prefix for topics (e.g., "husky1", "quadcopter1", "" for single robot)
  robot_namespace_ = declare_parameter<std::string>("robot_namespace", "husky1");

  // TF frame IDs
  map_frame_ = declare_parameter<std::string>(
      "map_frame", robot_namespace_.empty() ? "map" : robot_namespace_ + "_map");
  odom_frame_ = declare_parameter<std::string>(
      "odom_frame", robot_namespace_.empty() ? "odom" : robot_namespace_ + "_odom");

  // Topic names
  odom_topic_ = declare_parameter<std::string>(
      "odom_topic", robot_namespace_.empty() ? "/odom" : ("/" + robot_namespace_ + "/odometry"));
  goal_topic_ =
      declare_parameter<std::string>("navigation_goal_topic", "/embertrace/navigation_goal");
  mission_state_topic_ =
      declare_parameter<std::string>("mission_state_topic", "/embertrace/mission_state");

  // Nav2 action server name
  action_name_ = declare_parameter<std::string>(
      "action_name", robot_namespace_.empty() ? "/navigate_to_pose"
                                              : ("/" + robot_namespace_ + "/navigate_to_pose"));

  // World bounds for coordinate validation (in meters)
  const auto half_world_length = 18.5;
  min_x_ = declare_parameter<double>("world_min_x", -half_world_length);
  max_x_ = declare_parameter<double>("world_max_x", half_world_length);
  min_y_ = declare_parameter<double>("world_min_y", -half_world_length);
  max_y_ = declare_parameter<double>("world_max_y", half_world_length);

  boundary_frame_ = declare_parameter<std::string>("boundary_frame", odom_frame_);

  // Timing thresholds
  max_goal_age_s_ = declare_parameter<double>("max_goal_age_s", 5.0);
  pending_timeout_s_ = declare_parameter<double>("pending_timeout_s", 15.0);

  // Log loaded configuration
  RCLCPP_INFO(get_logger(), "Robot namespace: '%s'", robot_namespace_.c_str());
  RCLCPP_INFO(get_logger(), "Map frame: '%s', Odometry frame: '%s'", map_frame_.c_str(),
              odom_frame_.c_str());
  RCLCPP_INFO(get_logger(), "Odometry topic: %s", odom_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Action server: %s", action_name_.c_str());

  // TF2 machinery: listener fills the shared buffer from /tf and /tf_static
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Action client: use full namespace path
  action_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, action_name_);

  status_publisher_ = create_publisher<std_msgs::msg::String>("/embertrace/navigation_status", 10);

  // Create retry timer once upfront
  retry_timer_ = create_wall_timer(500ms, [this]() {
    geometry_msgs::msg::PoseStamped local_pending;
    bool has_pending = false;

    {
      std::scoped_lock lock(goalMtx_);
      if (!pending_goal_ || pending_goal_->header.frame_id.empty()) {
        retry_timer_->cancel();
        return;
      }
      local_pending = *pending_goal_;
      has_pending = true;
    }

    if (has_pending) {
      sendNav2Goal(local_pending);
    }
  });

  // Subscribe to mission state (transient_local for late joiners)
  // Use full namespace for MissionState
  state_subscription_ = create_subscription<embertrace_interfaces::msg::MissionState>(
      mission_state_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](const embertrace_interfaces::msg::MissionState::SharedPtr msg) {
        bool need_cancel = false;
        std::string reason;

        const bool selected =
            msg->active_platform == embertrace_interfaces::msg::MissionState::PLATFORM_ROVER;
        const bool permitted = selected && msg->motion_allowed && !msg->manual_control;
        if (!permitted || state_ != msg->state || epoch_ != msg->mission_epoch) {
          {
            std::scoped_lock lock(goalMtx_);
            pending_goal_.reset();
            requested_goal_.reset();
            ++generation_;
          }
          retry_timer_->cancel();
          need_cancel = true;
          reason = (msg->motion_allowed ? "state changed" : "motion disallowed");
        }
        state_ = msg->state;
        motion_allowed_ = permitted;
        epoch_ = msg->mission_epoch;

        if (need_cancel) {
          cancelActiveGoal(reason);
        }
      });

  // Subscribe to navigation goals
  goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg) {
        if (!motion_allowed_ || goal_msg->header.frame_id.empty() ||
            !std::isfinite(goal_msg->pose.position.x) ||
            !std::isfinite(goal_msg->pose.position.y)) {
          publish_status("REJECTED");
          RCLCPP_WARN(get_logger(),
                      "Ignoring goal; mission does not permit motion or invalid pose");
          return;
        }
        const rclcpp::Time stamp(goal_msg->header.stamp);
        if (stamp.nanoseconds() > 0) {
          const double age = (now() - stamp).seconds();
          if (age > max_goal_age_s_ || age < -1.0) {
            publish_status("STALE");
            return;
          }
        }

        {
          std::scoped_lock lock(goalMtx_);
          // I keep the current action for repeated search targets so Nav2's
          // progress checker can finish a leg or run its bounded recovery.
          if (requested_goal_ && requested_goal_->header.frame_id == goal_msg->header.frame_id) {
            const auto & previous = requested_goal_->pose;
            const auto & incoming = goal_msg->pose;
            const double yaw_delta = tf2::getYaw(previous.orientation) - tf2::getYaw(incoming.orientation);
            if (std::hypot(previous.position.x - incoming.position.x,
                           previous.position.y - incoming.position.y) <= 0.15 &&
                std::abs(std::atan2(std::sin(yaw_delta), std::cos(yaw_delta))) <= 0.15) {
              return;
            }
          }
          ++generation_;
          cancelActiveGoal("new goal arrived");
          pending_goal_ = *goal_msg;
          requested_goal_ = *goal_msg;
          pending_since_ = now();
          goal_ = *goal_msg;
        }

        if (retry_timer_->is_canceled()) {
          retry_timer_->reset();
        }
      });

  // Subscribe to odometry for telemetry/computed metrics
  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(), [this](const nav_msgs::msg::Odometry::SharedPtr odom) {
        {
          std::scoped_lock lock(poseMtx_);
          pose_ = odom->pose.pose;
        }
        updateDistAndAngleToGoal();
      });
}

// ============================================================================
// PUBLIC STATIC METHOD
// ============================================================================

void Nav2GoalAdapterNode::normaliseAngle(double &angle) {
  angle = std::fmod(angle, 2.0 * M_PI);
  if (angle < 0.0) {
    angle += 2.0 * M_PI;
  }
}

// ============================================================================
// NAV2 GOAL HANDLING
// ============================================================================

void Nav2GoalAdapterNode::sendNav2Goal(const geometry_msgs::msg::PoseStamped &raw_goal) {
  // Snapshot pending goal and state under lock at start
  bool has_pending;
  bool local_motion_allowed;
  uint64_t gen_at_snap;

  {
    std::scoped_lock lock(goalMtx_);
    has_pending = static_cast<bool>(pending_goal_);
    local_motion_allowed = motion_allowed_;
    gen_at_snap = generation_;
  }

  if (!has_pending || !local_motion_allowed) {
    return;
  }

  if ((now() - pending_since_).seconds() > pending_timeout_s_) {
    {
      std::scoped_lock lock(goalMtx_);
      pending_goal_.reset();
      requested_goal_.reset();
    }
    publish_status("STALE");
    return;
  }

  if (!action_client_->action_server_is_ready()) {
    RCLCPP_DEBUG(get_logger(), "Nav2 action server not yet available");
    return;
  }

  geometry_msgs::msg::PoseStamped goal_in_map;

  if (raw_goal.header.frame_id == map_frame_) {
    goal_in_map = raw_goal;
  } else {
    geometry_msgs::msg::PoseStamped temp = raw_goal;
    temp.header.stamp.sec = 0;
    temp.header.stamp.nanosec = 0;

    try {
      goal_in_map = tf_buffer_->transform(temp, map_frame_);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Skipping goal; TF unavailable: %s",
                           ex.what());
      return;
    }
  }

  geometry_msgs::msg::PoseStamped bounded;
  try {
    bounded = goal_in_map.header.frame_id == boundary_frame_
                  ? goal_in_map
                  : tf_buffer_->transform(goal_in_map, boundary_frame_);
  } catch (const tf2::TransformException &) {
    return;
  }
  const auto &p = bounded.pose.position;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < min_x_ || p.x > max_x_ || p.y < min_y_ ||
      p.y > max_y_) {
    {
      std::scoped_lock lock(goalMtx_);
      pending_goal_.reset();
      requested_goal_.reset();
    }
    publish_status("OUT_OF_BOUNDS");
    RCLCPP_ERROR(get_logger(), "Invalid goal coordinates (NaN/Inf or out of bounds)");
    return;
  }

  goal_in_map.header.stamp = get_clock()->now();

  nav2_msgs::action::NavigateToPose::Goal nav_goal;
  nav_goal.pose = goal_in_map;

  RCLCPP_INFO(get_logger(), "Sending Nav2 goal (%.2f, %.2f) gen=%lu", p.x, p.y, gen_at_snap);

  const std::uint64_t request_generation = gen_at_snap;

  auto send_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_options.goal_response_callback =
      [this, request_generation](
          std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>
              handle) {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "Nav2 rejected the goal");
          if (request_generation == generation_) {
            std::scoped_lock lock(goalMtx_);
            requested_goal_.reset();
            publish_status("REJECTED");
          }
          return;
        }

        {
          std::scoped_lock lock(goalMtx_);
          if (request_generation != generation_ || !motion_allowed_) {
            action_client_->async_cancel_goal(handle);
            return;
          }
        }

        {
          std::scoped_lock lock(goalHandleMtx_);
          active_goal_handle_ = handle;
        }
        publish_status("ACCEPTED");
      };

  send_options.result_callback =
      [this, request_generation](
          const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult
              &result) {
        if (request_generation != generation_) {
          return;
        }
        {
          std::scoped_lock lock(goalMtx_);
          requested_goal_.reset();
        }
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

  {
    std::scoped_lock lock(goalMtx_);
    if (gen_at_snap != generation_ || !motion_allowed_) {
      return;
    }
    pending_goal_.reset();
    goal_ = bounded;
  }
  action_client_->async_send_goal(nav_goal, send_options);
  publish_status("SENT");
}

// ============================================================================
// STATUS PUBLICATION
// ============================================================================

void Nav2GoalAdapterNode::publish_status(const std::string &status) {
  std_msgs::msg::String msg;
  msg.data = status;
  status_publisher_->publish(msg);
}

// ============================================================================
// CANCELLATION
// ============================================================================

void Nav2GoalAdapterNode::cancelActiveGoal(const std::string &reason) {
  std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>
      handle_to_cancel;

  {
    std::scoped_lock lock(goalHandleMtx_);
    if (!active_goal_handle_) {
      return;
    }
    handle_to_cancel = active_goal_handle_;
    active_goal_handle_.reset();
  }

  if (handle_to_cancel) {
    RCLCPP_WARN(get_logger(), "Cancelling Nav2 goal (%s)", reason.c_str());
    action_client_->async_cancel_goal(handle_to_cancel);
    publish_status("CANCEL_REQUESTED");
  }
}

// ============================================================================
// TELEMETRY COMPUTATIONS
// ============================================================================

void Nav2GoalAdapterNode::updateDistAndAngleToGoal() {
  geometry_msgs::msg::Pose local_goal_pose;
  bool has_goal;

  {
    std::scoped_lock lock(goalMtx_);
    has_goal = (goal_.header.stamp.sec != 0);
    if (has_goal) {
      local_goal_pose = goal_.pose;
    }
  }

  if (!has_goal) {
    return;
  }

  std::scoped_lock lock(poseMtx_);
  const double dx = local_goal_pose.position.x - pose_.position.x;
  const double dy = local_goal_pose.position.y - pose_.position.y;
  dist_to_goal_ = std::hypot(dx, dy);

  angle_to_goal_ = std::atan2(dy, dx);

  const double yaw = tf2::getYaw(pose_.orientation);
  heading_error_ = angle_to_goal_ - yaw;
  normaliseAngle(heading_error_);
}

}  // namespace embertrace_core
