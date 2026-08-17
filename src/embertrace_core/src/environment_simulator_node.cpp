/**
 * Environment simulator
 *
 * COMPLETE BASELINE:
 * - reads the simulated robot pose from odometry;
 * - samples the configurable plume and thermal field;
 * - publishes only local noisy sensor values.
 *
 * IMPORTANT: this is the only autonomy-side node allowed to know source_x/source_y.
 * TODO(student): add time-varying wind, sensor drift/dropout and scenario services only
 * after the fixed-wind baseline is tested.
 */

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "embertrace_core/plume_model.hpp"
#include "embertrace_interfaces/msg/environmental_reading.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace embertrace_core
{

class EnvironmentSimulatorNode : public rclcpp::Node
{
public:
  EnvironmentSimulatorNode()
  : Node("environment_simulator_node")
  {
    const auto odometry_topic = declare_parameter<std::string>("odometry_topic", "/husky1/odometry");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/embertrace/raw_environment");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "husky1_odom");
    const auto publish_rate_hz = declare_parameter<double>("publish_rate_hz", 5.0);
    const auto seed = declare_parameter<int>("random_seed", 42);

    PlumeParameters parameters;
    parameters.source_x = declare_parameter<double>("source_x", 12.0);
    parameters.source_y = declare_parameter<double>("source_y", 7.0);
    parameters.emission_strength_ppm = declare_parameter<double>("emission_strength_ppm", 120.0);
    parameters.baseline_smoke_ppm = declare_parameter<double>("baseline_smoke_ppm", 1.0);
    parameters.plume_length_m = declare_parameter<double>("plume_length_m", 14.0);
    parameters.plume_width_m = declare_parameter<double>("plume_width_m", 1.6);
    parameters.plume_expansion = declare_parameter<double>("plume_expansion", 0.05);
    parameters.smoke_noise_stddev = declare_parameter<double>("smoke_noise_stddev", 2.0);
    parameters.ambient_temperature_c = declare_parameter<double>("ambient_temperature_c", 22.0);
    parameters.source_temperature_c = declare_parameter<double>("source_temperature_c", 110.0);
    parameters.thermal_decay_m = declare_parameter<double>("thermal_decay_m", 7.0);
    parameters.wind_direction_rad = declare_parameter<double>("wind_direction_rad", -2.35619449);
    parameters.wind_speed_mps = declare_parameter<double>("wind_speed_mps", 2.5);
    plume_model_ = std::make_unique<PlumeModel>(parameters, static_cast<std::uint32_t>(seed));

    reading_publisher_ = create_publisher<embertrace_interfaces::msg::EnvironmentalReading>(
      output_topic, rclcpp::SensorDataQoS());
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        latest_odometry_ = *message;
        have_odometry_ = true;
      });

    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&EnvironmentSimulatorNode::publish_reading, this));

    RCLCPP_INFO(get_logger(), "Environmental field ready; waiting for %s", odometry_topic.c_str());
  }

private:
  void publish_reading()
  {
    if (!have_odometry_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for Husky odometry");
      return;
    }

    const auto & position = latest_odometry_.pose.pose.position;
    const auto sample = plume_model_->sample(position.x, position.y);

    embertrace_interfaces::msg::EnvironmentalReading message;
    message.header.stamp = now();
    message.header.frame_id = odom_frame_;
    message.sample_pose.header = message.header;
    message.sample_pose.pose = latest_odometry_.pose.pose;
    message.smoke_ppm = static_cast<float>(sample.smoke_ppm);
    message.temperature_c = static_cast<float>(sample.temperature_c);
    message.wind_speed_mps = static_cast<float>(sample.wind_speed_mps);
    message.wind_direction_rad = static_cast<float>(sample.wind_direction_rad);
    message.smoke_detected = false;
    message.temperature_anomaly = false;
    message.visual_confidence = 0.0F;
    message.fused_confidence = 0.0F;
    reading_publisher_->publish(message);
  }

  std::string odom_frame_;
  bool have_odometry_{false};
  nav_msgs::msg::Odometry latest_odometry_;
  std::unique_ptr<PlumeModel> plume_model_;
  rclcpp::Publisher<embertrace_interfaces::msg::EnvironmentalReading>::SharedPtr reading_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace embertrace_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::EnvironmentSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
