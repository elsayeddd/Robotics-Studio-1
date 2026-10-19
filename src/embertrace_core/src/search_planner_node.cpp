/**
 * Search planner: fit source from local smoke/heat samples, then navigate toward
 * that estimate. Recovery casts around the last detection. Only known free
 * occupancy cells with robot clearance can become goals.
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "embertrace_core/known_free_grid.hpp"
#include "embertrace_core/source_estimator.hpp"
#include "embertrace_interfaces/msg/environmental_reading.hpp"
#include "embertrace_interfaces/msg/mission_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace embertrace_core
{

constexpr double kPi = 3.14159265358979323846;

class SearchPlannerNode : public rclcpp::Node
{
public:
  using MissionState = embertrace_interfaces::msg::MissionState;
  using Reading = embertrace_interfaces::msg::EnvironmentalReading;

  SearchPlannerNode()
  : Node("search_planner_node")
  {
    SourceEstimatorConfig config;
    config.max_samples = static_cast<std::size_t>(std::max<int>(
      5, declare_parameter<int>("max_samples", 160)));
    config.minimum_spacing_m = std::max(
      0.05, declare_parameter<double>("minimum_spacing_m", 0.3));
    config.ambient_temperature_c = declare_parameter<double>("ambient_temperature_c", 22.0);
    config.baseline_smoke_ppm = declare_parameter<double>("baseline_smoke_ppm", 1.0);
    config.thermal_decay_m = declare_parameter<double>("thermal_decay_m", 7.0);
    config.plume_length_m = declare_parameter<double>("plume_length_m", 14.0);
    config.plume_width_m = declare_parameter<double>("plume_width_m", 1.6);
    config.plume_expansion = declare_parameter<double>("plume_expansion", 0.05);
    config.min_x = declare_parameter<double>("world_min_x", -18.5);
    config.max_x = declare_parameter<double>("world_max_x", 18.5);
    config.min_y = declare_parameter<double>("world_min_y", -18.5);
    config.max_y = declare_parameter<double>("world_max_y", 18.5);
    world_min_x_ = config.min_x;
    world_max_x_ = config.max_x;
    world_min_y_ = config.min_y;
    world_max_y_ = config.max_y;
    estimator_ = std::make_unique<SourceEstimator>(config);

    surge_step_m_ = std::max(0.2, declare_parameter<double>("surge_step_m", 2.0));
    cast_step_m_ = std::max(0.2, declare_parameter<double>("cast_step_m", 2.5));
    goal_interval_s_ = std::max(0.2, declare_parameter<double>("goal_interval_s", 2.0));
    estimate_interval_s_ = std::max(0.2, declare_parameter<double>("estimate_interval_s", 1.0));
    signal_loss_timeout_s_ = std::max(
      1.0, declare_parameter<double>("signal_loss_timeout_s", 20.0));
    search_timeout_s_ = std::max(1.0, declare_parameter<double>("search_timeout_s", 120.0));
    reading_timeout_s_ = std::max(0.5, declare_parameter<double>("reading_timeout_s", 3.0));
    robot_clearance_m_ = std::max(0.0, declare_parameter<double>("robot_clearance_m", 0.8));
    goal_snap_radius_m_ = std::max(
      0.0, declare_parameter<double>("goal_snap_radius_m", 2.0));
    max_map_occupancy_ = std::clamp<int>(
      declare_parameter<int>("max_map_occupancy", 20), 0, 99);
    map_topic_ = declare_parameter<std::string>("map_topic", "/husky1/map");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/embertrace/navigation_goal", 10);
    estimate_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/embertrace/source_estimate", 10);
    state_subscription_ = create_subscription<MissionState>(
      "/embertrace/mission_state", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&SearchPlannerNode::on_state, this, std::placeholders::_1));
    reading_subscription_ = create_subscription<Reading>(
      "/embertrace/environment", rclcpp::SensorDataQoS(),
      std::bind(&SearchPlannerNode::on_reading, this, std::placeholders::_1));
    scout_subscription_ = create_subscription<Reading>("/drone1/environment", rclcpp::SensorDataQoS(),
      [this](const Reading::SharedPtr reading) {
        const double age = (now() - rclcpp::Time(reading->header.stamp)).seconds();
        if (reading->header.frame_id == "husky1_odom" && age >= 0 && age <= reading_timeout_s_ &&
          (reading->smoke_detected || reading->temperature_anomaly) &&
          reading->fused_confidence >= 0.38)
        {scout_pose_ = reading->sample_pose; have_scout_ = true;}
      });
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&SearchPlannerNode::on_map, this, std::placeholders::_1));
    watchdog_ = create_wall_timer(1s, std::bind(&SearchPlannerNode::check_timeout, this));
  }

private:
  static bool search_state(const std::uint8_t state)
  {
    return state == MissionState::INVESTIGATE || state == MissionState::TRACK ||
           state == MissionState::RECOVER || state == MissionState::VERIFY;
  }

  static void set_yaw(geometry_msgs::msg::Pose & pose, const double yaw)
  {
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(yaw / 2.0);
    pose.orientation.w = std::cos(yaw / 2.0);
  }

  void on_state(const MissionState::SharedPtr message)
  {
    const bool was_searching = search_state(state_);
    state_ = message->state;
    if (state_ == MissionState::PATROL && was_searching) {
      // A dismissed investigation must not contaminate the next source fit.
      estimator_->clear();
      estimate_.reset();
    }
    if (!was_searching && search_state(state_)) {
      search_start_ = now();
      have_search_start_ = true;
      cast_count_ = 0;
      last_goal_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      have_last_goal_ = false;
    }
    if (state_ == MissionState::IDLE || state_ == MissionState::RETURN_TO_BASE ||
      state_ == MissionState::EMERGENCY_STOP || state_ == MissionState::ALERT)
    {
      estimator_->clear();
      estimate_.reset();
      last_signal_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      have_signal_ = false;
    }
  }

  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    if (message->header.frame_id.empty()) {
      return;
    }
    KnownFreeGrid grid;
    grid.width = message->info.width;
    grid.height = message->info.height;
    grid.resolution = message->info.resolution;
    grid.origin_x = message->info.origin.position.x;
    grid.origin_y = message->info.origin.position.y;
    const auto & q = message->info.origin.orientation;
    grid.origin_yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    grid.data = message->data;
    grid_ = std::move(grid);
    map_frame_ = message->header.frame_id;
  }

  void on_reading(const Reading::SharedPtr reading)
  {
    const auto & pose = reading->sample_pose;
    if (pose.header.frame_id.empty() ||
      !std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y))
    {
      return;
    }
    const rclcpp::Time stamp(reading->header.stamp);
    const rclcpp::Time current = now();
    if (have_reading_ && current < last_reading_time_) {
      // A new simulation run resets /clock and invalidates old time ordering.
      estimator_->clear();
      estimate_.reset();
      have_scout_ = false;
      last_sample_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      last_goal_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      have_last_goal_ = false;
      last_estimate_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      have_last_estimate_ = false;
      last_signal_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      have_signal_ = false;
      search_start_ = current;
      have_search_start_ = search_state(state_);
    }
    const double reading_age_s = (current - stamp).seconds();
    if (stamp.nanoseconds() > 0 &&
      ((last_sample_stamp_.nanoseconds() > 0 && stamp <= last_sample_stamp_) ||
      reading_age_s > reading_timeout_s_ || reading_age_s < -0.5))
    {
      return;  // Do not steer on delayed or out-of-order sensor data.
    }
    last_sample_stamp_ = stamp;
    last_reading_time_ = current;
    have_reading_ = true;

    if (state_ != MissionState::PATROL && !search_state(state_)) {
      return;
    }
    if (!sample_frame_.empty() && sample_frame_ != pose.header.frame_id) {
      estimator_->clear();
      estimate_.reset();
      have_signal_ = false;  // The old recovery anchor used another frame.
    }
    sample_frame_ = pose.header.frame_id;
    // A stationary reading can still drive a new goal after an aerial cue.
    estimator_->add({pose.pose.position.x, pose.pose.position.y, reading->smoke_ppm,
      reading->temperature_c, reading->wind_direction_rad});

    if (reading->smoke_detected) {
      last_signal_time_ = current;
      have_signal_ = true;
      last_signal_pose_ = pose;
      cast_count_ = 0;
    }
    if (!search_state(state_)) {
      return;
    }
    if (!have_last_estimate_ ||
      (current - last_estimate_time_).seconds() >= estimate_interval_s_)
    {
      estimate_ = estimator_->estimate();
      last_estimate_time_ = current;
      have_last_estimate_ = true;
      if (estimate_ && !search_timed_out(current)) {
        publish_estimate(*estimate_, pose.header.frame_id, current);
      }
    }
    // VERIFY has no motion permission, but may still refine an alert's location.
    if (state_ == MissionState::VERIFY || search_timed_out(current) ||
      signal_timed_out(current))
    {
      return;
    }
    if (have_last_goal_ &&
      (current - last_goal_time_).seconds() < goal_interval_s_)
    {
      return;
    }
    publish_goal(*reading, current);
  }

  bool search_timed_out(const rclcpp::Time & current) const
  {
    return have_search_start_ &&
           (current - search_start_).seconds() > search_timeout_s_;
  }

  bool signal_timed_out(const rclcpp::Time & current) const
  {
    return have_signal_ &&
           (current - last_signal_time_).seconds() > signal_loss_timeout_s_;
  }

  void publish_estimate(
    const SourceEstimate & source, const std::string & frame,
    const rclcpp::Time & current)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.frame_id = frame;
    message.header.stamp = current;
    message.pose.pose.position.x = source.x;
    message.pose.pose.position.y = source.y;
    message.pose.pose.orientation.w = 1.0;
    message.pose.covariance[0] = source.variance_x;
    message.pose.covariance[1] = source.covariance_xy;
    message.pose.covariance[6] = source.covariance_xy;
    message.pose.covariance[7] = source.variance_y;
    message.pose.covariance[14] = 1e6;
    message.pose.covariance[21] = 1e6;
    message.pose.covariance[28] = 1e6;
    message.pose.covariance[35] = 1e6;
    estimate_publisher_->publish(message);
  }

  void publish_goal(const Reading & reading, const rclcpp::Time & current)
  {
    const auto & position = reading.sample_pose.pose.position;
    double target_x = position.x;
    double target_y = position.y;
    bool casting = state_ == MissionState::RECOVER || !reading.smoke_detected;
    const double scout_age = have_scout_ ?
      (current - rclcpp::Time(scout_pose_.header.stamp)).seconds() : -1.0;
    if (!reading.smoke_detected && !reading.temperature_anomaly && have_scout_ &&
      scout_age >= 0 && scout_age <= 5.0 &&
      scout_pose_.header.frame_id == reading.sample_pose.header.frame_id)
    {
      // Flyover observations only choose a probe; they are not rover samples or truth.
      const double dx = scout_pose_.pose.position.x - position.x;
      const double dy = scout_pose_.pose.position.y - position.y;
      const double fraction = std::min(1.0, surge_step_m_ / std::max(0.01, std::hypot(dx, dy)));
      target_x += fraction * dx; target_y += fraction * dy;
      casting = false;
    } else if (casting) {
      // Expanding alternating crosswind casts revisit the last measured plume.
      const auto & anchor = have_signal_ ?
        last_signal_pose_.pose.position : position;
      const double crosswind = reading.wind_direction_rad + kPi / 2.0;
      const double side = cast_count_ % 2 == 0 ? 1.0 : -1.0;
      const double sweep = std::min(7.0, cast_step_m_ * (1.0 + cast_count_ / 2));
      target_x = anchor.x + side * sweep * std::cos(crosswind) +
        std::cos(reading.wind_direction_rad + kPi);
      target_y = anchor.y + side * sweep * std::sin(crosswind) +
        std::sin(reading.wind_direction_rad + kPi);
    } else if (estimate_ &&
      std::max(estimate_->variance_x, estimate_->variance_y) < 16.0)
    {
      const double dx = estimate_->x - position.x;
      const double dy = estimate_->y - position.y;
      const double distance = std::hypot(dx, dy);
      if (distance > 0.01) {
        const double fraction = std::min(1.0, surge_step_m_ / distance);
        target_x += fraction * dx;
        target_y += fraction * dy;
      }
    } else {
      // Before enough spatial evidence exists, use a short upwind probe.
      target_x += surge_step_m_ * std::cos(reading.wind_direction_rad + kPi);
      target_y += surge_step_m_ * std::sin(reading.wind_direction_rad + kPi);
    }

    // Preserve the scenario boundary constraint before consulting known map cells.
    target_x = std::clamp(target_x, world_min_x_, world_max_x_);
    target_y = std::clamp(target_y, world_min_y_, world_max_y_);
    if (map_frame_.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "No occupancy map; withholding search goals");
      return;
    }
    geometry_msgs::msg::PoseStamped requested = reading.sample_pose;
    requested.pose.position.x = target_x;
    requested.pose.position.y = target_y;
    // Request the latest transform: a pose stamped at now() can be newer than
    // the most recent SLAM transform by a few milliseconds.
    requested.header.stamp.sec = 0;
    requested.header.stamp.nanosec = 0;
    set_yaw(requested.pose, std::atan2(target_y - position.y, target_x - position.x));

    geometry_msgs::msg::PoseStamped goal;
    try {
      // Samples use odom coordinates; cells and navigation goals use map coordinates.
      goal = tf_buffer_->transform(requested, map_frame_);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "No map transform; withholding goal: %s",
        error.what());
      return;
    }
    const auto safe = grid_.nearest_free(
      goal.pose.position.x, goal.pose.position.y, goal_snap_radius_m_,
      robot_clearance_m_, max_map_occupancy_);
    if (!safe) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "No known free goal near search target");
      return;
    }
    goal.pose.position.x = safe->first;
    goal.pose.position.y = safe->second;
    goal.header.stamp = current;
    goal_publisher_->publish(goal);
    last_goal_time_ = current;
    have_last_goal_ = true;
    if (casting) {
      ++cast_count_;
    }
  }

  void check_timeout()
  {
    if (!search_state(state_)) {
      return;
    }
    const auto current = now();
    if (!have_reading_ ||
      (current - last_reading_time_).seconds() > reading_timeout_s_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Search readings stale; no new goals");
    }
    if (search_timed_out(current)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Search timeout reached; no new goals");
    } else if (signal_timed_out(current)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Plume lost beyond recovery deadline; no new goals");
    }
  }

  std::uint8_t state_{MissionState::IDLE};
  int cast_count_{0};
  bool have_reading_{false};
  bool have_scout_{false};
  geometry_msgs::msg::PoseStamped scout_pose_;
  rclcpp::Subscription<Reading>::SharedPtr scout_subscription_;
  bool have_signal_{false};
  bool have_search_start_{false};
  bool have_last_goal_{false};
  bool have_last_estimate_{false};
  double surge_step_m_{2.0};
  double cast_step_m_{2.5};
  double goal_interval_s_{2.0};
  double estimate_interval_s_{1.0};
  double signal_loss_timeout_s_{20.0};
  double search_timeout_s_{120.0};
  double reading_timeout_s_{3.0};
  double robot_clearance_m_{0.8};
  double goal_snap_radius_m_{2.0};
  double world_min_x_{-18.5};
  double world_max_x_{18.5};
  double world_min_y_{-18.5};
  double world_max_y_{18.5};
  int max_map_occupancy_{20};
  std::string map_topic_;
  std::string map_frame_;
  std::string sample_frame_;
  KnownFreeGrid grid_;
  std::unique_ptr<SourceEstimator> estimator_;
  std::optional<SourceEstimate> estimate_;
  geometry_msgs::msg::PoseStamped last_signal_pose_;
  rclcpp::Time last_sample_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_reading_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_signal_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_goal_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_estimate_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time search_start_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr estimate_publisher_;
  rclcpp::Subscription<MissionState>::SharedPtr state_subscription_;
  rclcpp::Subscription<Reading>::SharedPtr reading_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

}  // namespace embertrace_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::SearchPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
