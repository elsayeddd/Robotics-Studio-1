/** Sequence rover waypoints only while it owns the mission; report loops and home arrival. */

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "embertrace_core/waypoint_patrol.hpp"
#include "embertrace_interfaces/msg/mission_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace embertrace_core {

class PatrolManagerNode : public rclcpp::Node {
 public:
  using MissionState = embertrace_interfaces::msg::MissionState;

  PatrolManagerNode() : Node("patrol_manager_node") {
    const auto waypoints_file = declare_parameter<std::string>("waypoints_file", "");
    const auto base_waypoint_name =
        declare_parameter<std::string>("base_waypoint_name", "ranger_base");
    const auto arrival_tolerance_m = declare_parameter<double>("arrival_tolerance_m", 0.5);
    const auto odometry_topic =
        declare_parameter<std::string>("odometry_topic", "/husky1/odometry");
    goal_timeout_s_ = declare_parameter<double>("goal_timeout_s", 120);
    if (!std::isfinite(goal_timeout_s_) || goal_timeout_s_ <= 0) {
      throw std::invalid_argument("goal_timeout_s must be positive");
    }
    frame_id_ = declare_parameter<std::string>("frame_id", "husky1_map");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    patrol_ = std::make_unique<WaypointPatrol>(arrival_tolerance_m);

    if (waypoints_file.empty()) {
      RCLCPP_FATAL(get_logger(),
                   "Parameter 'waypoints_file' was not set; patrol manager has no route to run");
    } else {
      try {
        const PatrolRoute route = load_patrol_route(waypoints_file);
        if (!route.frame_id.empty()) {
          frame_id_ = route.frame_id;
        }
        patrol_->set_waypoints(route.waypoints, base_waypoint_name);
        RCLCPP_INFO(get_logger(), "Loaded %zu patrol waypoints from %s (frame %s)",
                    route.waypoints.size(), waypoints_file.c_str(), frame_id_.c_str());
      } catch (const std::exception& error) {
        RCLCPP_FATAL(get_logger(), "%s", error.what());
      }
    }

    loops_publisher_ = create_publisher<std_msgs::msg::UInt32>("/husky1/patrol_loops", 10);
    arrival_publisher_ =
        create_publisher<std_msgs::msg::String>("/husky1/mission_navigation_status", 10);
    goal_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("/embertrace/navigation_goal", 10);
    state_subscription_ = create_subscription<MissionState>(
        "/embertrace/mission_state", rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&PatrolManagerNode::on_mission_state, this, std::placeholders::_1));
    status_subscription_ = create_subscription<std_msgs::msg::String>(
        "/embertrace/navigation_status", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
          if (msg->data == "SENT" || msg->data == "ACCEPTED") {
            acknowledged_ = true;
          } else if (msg->data == "REJECTED" || msg->data == "STALE" || msg->data == "ABORTED") {
            acknowledged_ = false;
            if (msg->data == "ABORTED") {
              std_msgs::msg::String status;
              status.data = "ABORTED";
              arrival_publisher_->publish(status);
            }
          }
        });
    retry_ = create_wall_timer(std::chrono::seconds(2), [this]() {
      if (last_goal_ && goal_started_ &&
          std::chrono::duration<double>(std::chrono::steady_clock::now() - goal_start_).count() >
              goal_timeout_s_) {
        std_msgs::msg::String status;
        status.data = "TIMEOUT";
        arrival_publisher_->publish(status);
        last_goal_.reset();
        return;
      }
      if (last_goal_ && !acknowledged_ && retries_ < 3 &&
          (patrol_->is_patrolling() || patrol_->is_returning())) {
        auto retry = *last_goal_;
        retry.header.stamp = now();
        goal_publisher_->publish(retry);
        ++retries_;
      }
    });
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic, rclcpp::SensorDataQoS(),
        std::bind(&PatrolManagerNode::on_odometry, this, std::placeholders::_1));
  }

 private:
  static PatrolTrigger to_trigger(const std::uint8_t state) {
    if (state == MissionState::PATROL) {
      return PatrolTrigger::kPatrol;
    }
    if (state == MissionState::RETURN_TO_BASE) {
      return PatrolTrigger::kReturnToBase;
    }
    return PatrolTrigger::kHold;
  }

  static void set_yaw(geometry_msgs::msg::Pose& pose, const double yaw) {
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(yaw / 2.0);
    pose.orientation.w = std::cos(yaw / 2.0);
  }

  void publish_goal(const Waypoint& waypoint) {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = frame_id_;
    goal.pose.position.x = waypoint.x;
    goal.pose.position.y = waypoint.y;
    set_yaw(goal.pose, waypoint.yaw);
    goal_start_ = std::chrono::steady_clock::now();
    goal_started_ = true;
    last_goal_ = goal;
    acknowledged_ = false;
    retries_ = 0;
    goal_publisher_->publish(goal);
    RCLCPP_INFO(get_logger(), "Patrol goal: %s (%.2f, %.2f)", waypoint.name.c_str(), waypoint.x,
                waypoint.y);
  }

  void on_mission_state(const MissionState::SharedPtr message) {
    if (!patrol_ || !patrol_->has_waypoints()) {
      return;
    }
    const bool selected = message->active_platform == MissionState::PLATFORM_ROVER &&
                          message->motion_allowed && !message->manual_control;
    const auto trigger = selected ? to_trigger(message->state) : PatrolTrigger::kHold;
    if (trigger == PatrolTrigger::kHold) {
      last_goal_.reset();
    }
    if (epoch_ != message->mission_epoch) {
      epoch_ = message->mission_epoch;
      returned_reported_ = false;
    }
    const auto goal = patrol_->on_mission_state(trigger);
    if (goal) {
      publish_goal(*goal);
    }
  }

  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr message) {
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
    } catch (const tf2::TransformException& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Waiting for patrol pose transform: %s", error.what());
      return;
    }
    const double age = (now() - rclcpp::Time(message->header.stamp)).seconds();
    if (age < -.1 || age > 1 || !std::isfinite(position.point.x) ||
        !std::isfinite(position.point.y)) {
      return;
    }
    if (!home_set_) {
      patrol_->set_home_pose(position.point.x, position.point.y);
      home_set_ = true;
    }
    const auto goal = patrol_->update_pose(position.point.x, position.point.y);
    std_msgs::msg::UInt32 loops;
    loops.data = static_cast<std::uint32_t>(patrol_->loops_completed());
    loops_publisher_->publish(loops);
    if (patrol_->is_returning() && patrol_->has_reached_base() && !returned_reported_) {
      returned_reported_ = true;
      std_msgs::msg::String status;
      status.data = "RETURNED";
      arrival_publisher_->publish(status);
    }
    if (patrol_->loops_completed() > loops_logged_) {
      loops_logged_ = patrol_->loops_completed();
      RCLCPP_INFO(get_logger(), "Patrol loop %zu complete", loops_logged_);
    }
    if (goal) {
      publish_goal(*goal);
    }
  }

  bool acknowledged_{false}, returned_reported_{false}, home_set_{false}, goal_started_{false};
  double goal_timeout_s_{120};
  std::chrono::steady_clock::time_point goal_start_;
  std::uint64_t epoch_{0};
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr loops_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arrival_publisher_;
  int retries_{0};
  std::optional<geometry_msgs::msg::PoseStamped> last_goal_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  rclcpp::TimerBase::SharedPtr retry_;
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

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::PatrolManagerNode>());
  rclcpp::shutdown();
  return 0;
}
