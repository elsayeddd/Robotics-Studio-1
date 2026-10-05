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
#include <cmath>
#include "embertrace_interfaces/msg/trial_truth.hpp"
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
    scenario_ = declare_parameter<std::string>("scenario", "fire");
    if (scenario_ != "fire" && scenario_ != "no_fire" && scenario_ != "dust" &&
      scenario_ != "hot_surface" && scenario_ != "visual_decoy" && scenario_ != "dropout")
    {throw std::invalid_argument("Unknown demo scenario");}
    odometry_timeout_s_ = declare_parameter<double>("odometry_timeout_s", 1.0);
    altitude_decay_m_ = declare_parameter<double>("altitude_decay_m", 8.0);
    dropout_start_s_ = declare_parameter<double>("dropout_start_s", 25.0);
    dropout_duration_s_ = declare_parameter<double>("dropout_duration_s", 8.0);
    const auto truth_topic = declare_parameter<std::string>(
      "truth_topic", "/embertrace/validation/truth");
    truth_publisher_ = create_publisher<embertrace_interfaces::msg::TrialTruth>(truth_topic, 10);

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
    if (scenario_ == "no_fire" || scenario_ == "visual_decoy") {
      parameters.emission_strength_ppm = 0.0;
      parameters.source_temperature_c = parameters.ambient_temperature_c;
    } else if (scenario_ == "dust") {
      parameters.source_temperature_c = parameters.ambient_temperature_c;
    } else if (scenario_ == "hot_surface") {
      parameters.emission_strength_ppm = 0.0;
    }
    ambient_ = parameters.ambient_temperature_c;
    baseline_ = parameters.baseline_smoke_ppm;
    plume_model_ = std::make_unique<PlumeModel>(parameters, static_cast<std::uint32_t>(seed));

    reading_publisher_ = create_publisher<embertrace_interfaces::msg::EnvironmentalReading>(
      output_topic, rclcpp::SensorDataQoS());
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        const auto & p = message->pose.pose.position;
        const double age = (now() - rclcpp::Time(message->header.stamp)).seconds();
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          message->header.frame_id != odom_frame_ || age < -0.1 || age > odometry_timeout_s_)
        {return;}
        latest_odometry_ = *message;
        have_odometry_ = true;
        if (!have_start_) {start_ = now(); have_start_ = true;}
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
    if (!have_odometry_ ||
      (now() - rclcpp::Time(latest_odometry_.header.stamp)).seconds() > odometry_timeout_s_ ||
      now() < rclcpp::Time(latest_odometry_.header.stamp)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for Husky odometry");
      return;
    }

    const auto & position = latest_odometry_.pose.pose.position;
    auto sample = plume_model_->sample(position.x, position.y);
    const double attenuation = std::exp(-std::max(0.0, position.z) / std::max(0.1, altitude_decay_m_));
    sample.smoke_ppm = baseline_ + (sample.smoke_ppm - baseline_) * attenuation;
    sample.temperature_c = ambient_ + (sample.temperature_c - ambient_) * attenuation;
    const double elapsed = (now() - start_).seconds();
    embertrace_interfaces::msg::TrialTruth truth;
    truth.header.stamp = now();
    truth.scenario = scenario_;
    // This evaluation-only topic is never subscribed to by decision nodes.
    truth.fire_present = (scenario_ == "fire" || scenario_ == "dropout") &&
      (sample.smoke_ppm >= 12.0 || sample.temperature_c >= 32.0);
    truth.sensor_dropout = scenario_ == "dropout" && elapsed >= dropout_start_s_ &&
      elapsed < dropout_start_s_ + dropout_duration_s_;
    truth_publisher_->publish(truth);
    if (truth.sensor_dropout) {return;}

    embertrace_interfaces::msg::EnvironmentalReading message;
    message.header.stamp = truth.header.stamp;
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

  std::string odom_frame_, scenario_;
  double odometry_timeout_s_{1.0}, altitude_decay_m_{8.0};
  double ambient_{22.0}, baseline_{1.0}, dropout_start_s_{25.0}, dropout_duration_s_{8.0};
  bool have_start_{false};
  rclcpp::Time start_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<embertrace_interfaces::msg::TrialTruth>::SharedPtr truth_publisher_;
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
