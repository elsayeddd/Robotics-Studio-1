/**
 * Reactive plume-search goal generator.
 *
 * COMPLETE BASELINE: upwind surge while the plume is detected, alternating
 * crosswind casting during recovery, world-bound clamping and goal throttling.
 *
 * TODO(student): use sample-history gradients, validate goals against the Nav2
 * costmap, estimate source uncertainty and compare algorithms quantitatively.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "embertrace_interfaces/msg/environmental_reading.hpp"
#include "embertrace_interfaces/msg/mission_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace embertrace_core
{

constexpr double kPi = 3.14159265358979323846;

class SearchPlannerNode : public rclcpp::Node
{
public:
  using MissionState = embertrace_interfaces::msg::MissionState;

  SearchPlannerNode()
  : Node("search_planner_node")
  {
    surge_step_m_ = declare_parameter<double>("surge_step_m", 2.0);
    cast_step_m_ = declare_parameter<double>("cast_step_m", 2.5);
    goal_interval_s_ = declare_parameter<double>("goal_interval_s", 2.0);
    world_min_x_ = declare_parameter<double>("world_min_x", -18.5);
    world_max_x_ = declare_parameter<double>("world_max_x", 18.5);
    world_min_y_ = declare_parameter<double>("world_min_y", -18.5);
    world_max_y_ = declare_parameter<double>("world_max_y", 18.5);

    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/embertrace/navigation_goal", 10);
    state_subscription_ = create_subscription<MissionState>(
      "/embertrace/mission_state", rclcpp::QoS(1).transient_local().reliable(),
      [this](const MissionState::SharedPtr message) {state_ = message->state;});
    reading_subscription_ = create_subscription<embertrace_interfaces::msg::EnvironmentalReading>(
      "/embertrace/environment", rclcpp::SensorDataQoS(),
      std::bind(&SearchPlannerNode::on_reading, this, std::placeholders::_1));
  }

private:
  static void set_yaw(geometry_msgs::msg::Pose & pose, const double yaw)
  {
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(yaw / 2.0);
    pose.orientation.w = std::cos(yaw / 2.0);
  }

  void on_reading(const embertrace_interfaces::msg::EnvironmentalReading::SharedPtr reading)
  {
    const bool active = state_ == MissionState::INVESTIGATE || state_ == MissionState::TRACK ||
      state_ == MissionState::RECOVER;
    if (!active) {
      return;
    }

    const rclcpp::Time stamp(reading->header.stamp);
    if (last_goal_time_.nanoseconds() > 0 && (stamp - last_goal_time_).seconds() < goal_interval_s_) {
      return;
    }

    double heading = reading->wind_direction_rad + kPi;  // Upwind points toward the source.
    double step = surge_step_m_;
    if (state_ == MissionState::RECOVER || !reading->smoke_detected) {
      cast_left_ = !cast_left_;
      heading = reading->wind_direction_rad + (cast_left_ ? kPi / 2.0 : -kPi / 2.0);
      step = cast_step_m_;
    } else if (reading->fused_confidence > 0.75F) {
      step *= 0.5;  // Smaller moves reduce overshoot close to a likely source.
    }

    geometry_msgs::msg::PoseStamped goal;
    goal.header = reading->sample_pose.header;
    goal.header.stamp = now();
    goal.pose = reading->sample_pose.pose;
    goal.pose.position.x = std::clamp(
      goal.pose.position.x + step * std::cos(heading), world_min_x_, world_max_x_);
    goal.pose.position.y = std::clamp(
      goal.pose.position.y + step * std::sin(heading), world_min_y_, world_max_y_);
    set_yaw(goal.pose, heading);
    goal_publisher_->publish(goal);
    last_goal_time_ = stamp;
  }

  std::uint8_t state_{MissionState::IDLE};
  bool cast_left_{false};
  double surge_step_m_{2.0};
  double cast_step_m_{2.5};
  double goal_interval_s_{2.0};
  double world_min_x_{-18.5};
  double world_max_x_{18.5};
  double world_min_y_{-18.5};
  double world_max_y_{18.5};
  rclcpp::Time last_goal_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Subscription<MissionState>::SharedPtr state_subscription_;
  rclcpp::Subscription<embertrace_interfaces::msg::EnvironmentalReading>::SharedPtr
    reading_subscription_;
};

}  // namespace embertrace_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::SearchPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
