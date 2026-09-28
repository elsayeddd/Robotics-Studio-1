/** Bridge mission goals to the namespaced Nav2 NavigateToPose action. */

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
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace embertrace_core
{

class Nav2GoalAdapterNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using MissionState = embertrace_interfaces::msg::MissionState;

  Nav2GoalAdapterNode()
  : Node("nav2_goal_adapter_node")
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

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name);
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/embertrace/navigation_status", 10);

    state_subscription_ = create_subscription<MissionState>(
      "/embertrace/mission_state", rclcpp::QoS(1).transient_local().reliable(),
      [this](const MissionState::SharedPtr state) {
        // A patrol goal must not keep moving the robot after investigation starts.
        if (!state->motion_allowed || state_ != state->state) {
          pending_goal_.reset();
          ++generation_;
          cancel_active_goal();
        }
        state_ = state->state;
        motion_allowed_ = state->motion_allowed;
      });

    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/embertrace/navigation_goal", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr goal) {
        if (!motion_allowed_ || goal->header.frame_id.empty() ||
          !std::isfinite(goal->pose.position.x) ||
          !std::isfinite(goal->pose.position.y))
        {
          publish_status("REJECTED");
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
        cancel_active_goal();
        pending_goal_ = *goal;
        pending_since_ = now();
        try_dispatch();
      });

    retry_timer_ = create_wall_timer(500ms, [this]() {try_dispatch();});
  }

private:
  void publish_status(const char * value)
  {
    std_msgs::msg::String status;
    status.data = value;
    status_publisher_->publish(status);
  }

  void cancel_active_goal()
  {
    if (active_goal_handle_ && active_goal_handle_->is_active()) {
      action_client_->async_cancel_goal(active_goal_handle_);
      publish_status("CANCEL_REQUESTED");
    }
    active_goal_handle_.reset();
  }

  void try_dispatch()
  {
    if (!pending_goal_ || !motion_allowed_) {
      return;
    }
    if ((now() - pending_since_).seconds() > pending_timeout_s_) {
      pending_goal_.reset();
      publish_status("STALE");
      return;
    }
    if (!action_client_->wait_for_action_server(0s)) {
      return;  // Nav2 starts after the other core nodes.
    }

    geometry_msgs::msg::PoseStamped requested = *pending_goal_;
    requested.header.stamp.sec = 0;
    requested.header.stamp.nanosec = 0;  // use the latest available transform
    geometry_msgs::msg::PoseStamped mapped;
    try {
      mapped = requested.header.frame_id == map_frame_ ?
        requested : tf_buffer_->transform(requested, map_frame_);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for goal transform: %s", error.what());
      return;
    }

    const auto & p = mapped.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
      p.x < min_x_ || p.x > max_x_ || p.y < min_y_ || p.y > max_y_)
    {
      pending_goal_.reset();
      publish_status("OUT_OF_BOUNDS");
      return;
    }
    mapped.header.stamp = now();
    pending_goal_.reset();
    const std::uint64_t request_generation = generation_;

    NavigateToPose::Goal nav_goal;
    nav_goal.pose = mapped;
    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback = [this, request_generation](
      std::shared_ptr<GoalHandle> handle)
      {
        if (!handle) {
          if (request_generation == generation_) {publish_status("REJECTED");}
          return;
        }
        if (request_generation != generation_ || !motion_allowed_) {
          action_client_->async_cancel_goal(handle);
          return;
        }
        active_goal_handle_ = handle;
        publish_status("ACCEPTED");
      };
    options.result_callback = [this, request_generation](
      const GoalHandle::WrappedResult & result)
      {
        if (request_generation != generation_) {return;}
        active_goal_handle_.reset();
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            publish_status("SUCCEEDED");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            publish_status("CANCELED");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            publish_status("ABORTED");
            break;
          default:
            publish_status("UNKNOWN");
        }
      };
    action_client_->async_send_goal(nav_goal, options);
    publish_status("SENT");
  }

  bool motion_allowed_{false};
  std::uint8_t state_{255};
  std::uint64_t generation_{0};
  double max_goal_age_s_{5.0};
  double pending_timeout_s_{15.0};
  double min_x_{-18.5}, max_x_{18.5}, min_y_{-18.5}, max_y_{18.5};
  std::string map_frame_;
  std::optional<geometry_msgs::msg::PoseStamped> pending_goal_;
  rclcpp::Time pending_since_{0, 0, RCL_ROS_TIME};
  std::shared_ptr<GoalHandle> active_goal_handle_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<MissionState>::SharedPtr state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
};

}  // namespace embertrace_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::Nav2GoalAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
