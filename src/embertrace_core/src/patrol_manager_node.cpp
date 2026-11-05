
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "embertrace_core/waypoint_patrol.hpp"
#include "embertrace_interfaces/msg/mission_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
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
    const auto arrival_tolerance_m = declare_parameter<double>("arrival_tolerance_m", 1.2);
    base_frame_ = declare_parameter<std::string>("base_frame", "husky1_base_link");
    const auto map_frame = declare_parameter<std::string>("map_frame", "husky1_map");
    const auto odom_frame = declare_parameter<std::string>("odom_frame", "husky1_odom");
    const auto odometry_topic = declare_parameter<std::string>("odometry_topic", "/husky1/odometry");
    frame_id_ = declare_parameter<std::string>("frame_id", map_frame);
    const auto status_topic = declare_parameter<std::string>(
      "status_topic", "/embertrace/navigation_status");
    const auto pose_rate_hz = declare_parameter<double>("pose_rate_hz", 10.0);

    RecoveryPolicy policy;
    policy.retry_delay_s = declare_parameter<double>("retry_delay_s", 2.0);
    policy.goal_timeout_s = declare_parameter<double>("goal_timeout_s", 90.0);
    policy.max_goal_attempts = static_cast<int>(
      declare_parameter<std::int64_t>("max_goal_attempts", 3));
    policy.min_success_age_s = declare_parameter<double>("min_success_age_s", 1.0);
    policy.base_settle_s = declare_parameter<double>("base_settle_s", 6.0);

    // Robot TF lives in the husky1 namespace; the launch file remaps /tf
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    patrol_ = std::make_unique<WaypointPatrol>(arrival_tolerance_m);
    patrol_->set_recovery_policy(policy);

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

    RCLCPP_INFO(get_logger(), "Configured patrol frame=%s, odom_frame=%s, odom_topic=%s",
      frame_id_.c_str(), odom_frame.c_str(), odometry_topic.c_str());
    loops_publisher_ = create_publisher<std_msgs::msg::UInt32>("/husky1/patrol_loops", 10);
    arrival_publisher_ = create_publisher<std_msgs::msg::String>("/husky1/mission_navigation_status", 10);
    home_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr message) {
        const auto& position = message->pose.pose.position;
        const double age = (now() - rclcpp::Time(message->header.stamp)).seconds();
        if (!home_set_ && message->header.frame_id == frame_id_ && age >= -.1 && age <= 1 &&
            std::isfinite(position.x) && std::isfinite(position.y)) {
          patrol_->set_home_pose(position.x, position.y);
          home_set_ = true;
        }
      });
    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/embertrace/navigation_goal", 10);
    return_complete_publisher_ = create_publisher<std_msgs::msg::Empty>(
      "/embertrace/return_complete", 10);
    state_subscription_ = create_subscription<MissionState>(
      "/embertrace/mission_state", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&PatrolManagerNode::on_mission_state, this, std::placeholders::_1));
    status_subscription_ = create_subscription<std_msgs::msg::String>(
      status_topic, 10,
      std::bind(&PatrolManagerNode::on_status, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, pose_rate_hz));
    pose_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&PatrolManagerNode::on_pose_timer, this));
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

  bool ready() const
  {
    return patrol_ && patrol_->has_waypoints();
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
      get_logger(), "Patrol goal: %s (%.2f, %.2f) attempt %d", waypoint.name.c_str(),
      waypoint.x, waypoint.y, patrol_->goal_attempts());
  }

  // Logs loop/skip progress and publishes the goal, if any, that the patrol
  // logic returned. Every call into patrol_ goes through here.
  void emit(const std::optional<Waypoint> & goal)
  {
    std_msgs::msg::UInt32 loops;
    loops.data = static_cast<std::uint32_t>(patrol_->loops_completed());
    loops_publisher_->publish(loops);
    if (patrol_->loops_completed() > loops_logged_) {
      loops_logged_ = patrol_->loops_completed();
      RCLCPP_INFO(get_logger(), "Patrol loop %zu complete", loops_logged_);
    }
    if (patrol_->skipped_waypoints() > skips_logged_) {
      skips_logged_ = patrol_->skipped_waypoints();
      RCLCPP_WARN(
        get_logger(), "Gave up on an unreachable waypoint (%zu skipped so far)", skips_logged_);
    }
    if (goal) {
      publish_goal(*goal);
    }

  const bool home = patrol_->return_complete();
  if (home && !arrival_announced_) {
    arrival_announced_ = true;
    RCLCPP_INFO(get_logger(), "Rover is parked at the base; requesting IDLE");
    announce_arrival();
  } else if (!home) {
      arrival_announced_ = false;
    }
  }

  void announce_arrival()
  {
    return_complete_publisher_->publish(std_msgs::msg::Empty());
    std_msgs::msg::String status;
    status.data = "RETURNED";
    arrival_publisher_->publish(status);
  }

  void on_mission_state(const MissionState::SharedPtr message)
  {
    if (!ready()) {
      return;
    }
    const bool announced_before = arrival_announced_;
    patrol_->set_time(now().seconds());
    const bool selected = message->active_platform == MissionState::PLATFORM_ROVER &&
                          message->motion_allowed && !message->manual_control;
    if (epoch_ != message->mission_epoch) {
      epoch_ = message->mission_epoch;
      arrival_announced_ = false;
    }
    emit(patrol_->on_mission_state(selected ? to_trigger(message->state) : PatrolTrigger::kHold));
    if (announced_before && arrival_announced_ && message->state == MissionState::RETURN_TO_BASE) {
      announce_arrival();
    }
  }

  void on_status(const std_msgs::msg::String::SharedPtr message)
  {
    if (!ready()) {
      return;
    }
    const std::string & status = message->data;
    RCLCPP_INFO(get_logger(), "Nav2 adapter status: %s", status.c_str());
    std::optional<NavResult> result;
    if (status == "SUCCEEDED") {
      result = NavResult::kSucceeded;
    } else if (status == "ABORTED" || status == "REJECTED" || status == "STALE") {
      result = NavResult::kFailed;
    } else if (status == "OUT_OF_BOUNDS") {
      result = NavResult::kOutOfBounds;
    }
    if (!result) {
      return;  // SENT, ACCEPTED, CANCELED, CANCEL_REQUESTED, ... need no action
    }
    patrol_->set_time(now().seconds());
    emit(patrol_->on_nav_result(*result));
  }

  void on_pose_timer()
  {
    if (!ready()) {
      return;
    }
    // Watchdog first: re-sends a dropped or overdue goal.
    emit(patrol_->tick(now().seconds()));

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(frame_id_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for transform %s -> %s: %s",
        frame_id_.c_str(), base_frame_.c_str(), error.what());
      return;
    }
    const auto& position = transform.transform.translation;
    const double age = (now() - rclcpp::Time(transform.header.stamp)).seconds();
    if (age < -.1 || age > 1 || !std::isfinite(position.x) || !std::isfinite(position.y)) return;
    // Shows where the rover believes it is (in the waypoint frame) and whether
    // a patrol leg is active, so a stalled run can be diagnosed from the log.
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "Rover pose in %s: (%.2f, %.2f)%s",
      frame_id_.c_str(), transform.transform.translation.x, transform.transform.translation.y,
      patrol_->is_patrolling() ? " [patrolling]" :
      (patrol_->is_returning() ? " [returning]" : " [idle]"));
    emit(
      patrol_->update_pose(transform.transform.translation.x, transform.transform.translation.y));
  }

  std::string frame_id_;
  std::string base_frame_{"husky1_base_link"};
  std::size_t loops_logged_{0};
  std::size_t skips_logged_{0};
  bool arrival_announced_{false}, home_set_{false};
  std::uint64_t epoch_{0};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr home_subscription_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr loops_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arrival_publisher_;
  std::unique_ptr<WaypointPatrol> patrol_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr return_complete_publisher_;
  rclcpp::Subscription<MissionState>::SharedPtr state_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
};

}  // namespace embertrace_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::PatrolManagerNode>());
  rclcpp::shutdown();
  return 0;
}
