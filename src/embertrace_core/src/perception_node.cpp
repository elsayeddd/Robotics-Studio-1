/**
 * Environmental perception and sensor-fusion node.
 *
 * COMPLETE BASELINE: exponential filtering, hysteresis, temperature anomaly
 * detection and weighted fusion with an optional visual cue.
 *
 * TODO(student): add stale-sensor detection, calibrated confidence functions,
 * evaluation logs and unit tests derived from recorded trials.
 */

#include <algorithm>
#include <functional>
#include <memory>
#include <string>

#include "embertrace_interfaces/msg/environmental_reading.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace embertrace_core
{

class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode()
  : Node("perception_node")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/embertrace/raw_environment");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/embertrace/environment");
    const auto visual_topic = declare_parameter<std::string>(
      "visual_topic", "/embertrace/visual_fire_cue");

    filter_alpha_ = std::clamp(declare_parameter<double>("filter_alpha", 0.25), 0.01, 1.0);
    smoke_enter_ppm_ = declare_parameter<double>("smoke_enter_ppm", 12.0);
    smoke_exit_ppm_ = declare_parameter<double>("smoke_exit_ppm", 8.0);
    smoke_full_scale_ppm_ = declare_parameter<double>("smoke_full_scale_ppm", 45.0);
    baseline_smoke_ppm_ = declare_parameter<double>("baseline_smoke_ppm", 1.0);
    temperature_threshold_c_ = declare_parameter<double>("temperature_threshold_c", 32.0);
    temperature_full_scale_c_ = declare_parameter<double>("temperature_full_scale_c", 65.0);
    visual_weight_ = std::clamp(declare_parameter<double>("visual_weight", 0.15), 0.0, 0.5);

    publisher_ = create_publisher<embertrace_interfaces::msg::EnvironmentalReading>(
      output_topic, rclcpp::SensorDataQoS());
    reading_subscription_ = create_subscription<embertrace_interfaces::msg::EnvironmentalReading>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&PerceptionNode::on_reading, this, std::placeholders::_1));
    visual_subscription_ = create_subscription<std_msgs::msg::Float32>(
      visual_topic, rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::Float32::SharedPtr message) {
        visual_confidence_ = std::clamp(static_cast<double>(message->data), 0.0, 1.0);
      });
  }

private:
  static double normalise(const double value, const double low, const double high)
  {
    if (high <= low) {
      return 0.0;
    }
    return std::clamp((value - low) / (high - low), 0.0, 1.0);
  }

  void on_reading(const embertrace_interfaces::msg::EnvironmentalReading::SharedPtr input)
  {
    if (!filter_initialised_) {
      filtered_smoke_ppm_ = input->smoke_ppm;
      filtered_temperature_c_ = input->temperature_c;
      filter_initialised_ = true;
    } else {
      filtered_smoke_ppm_ = filter_alpha_ * input->smoke_ppm +
        (1.0 - filter_alpha_) * filtered_smoke_ppm_;
      filtered_temperature_c_ = filter_alpha_ * input->temperature_c +
        (1.0 - filter_alpha_) * filtered_temperature_c_;
    }

    if (!smoke_latched_ && filtered_smoke_ppm_ >= smoke_enter_ppm_) {
      smoke_latched_ = true;
    } else if (smoke_latched_ && filtered_smoke_ppm_ <= smoke_exit_ppm_) {
      smoke_latched_ = false;
    }

    const double smoke_confidence = normalise(
      filtered_smoke_ppm_, baseline_smoke_ppm_, smoke_full_scale_ppm_);
    const double temperature_confidence = normalise(
      filtered_temperature_c_, temperature_threshold_c_, temperature_full_scale_c_);
    const double environmental_weight = 1.0 - visual_weight_;
    const double fused = environmental_weight * (0.7 * smoke_confidence + 0.3 * temperature_confidence) +
      visual_weight_ * visual_confidence_;

    auto output = *input;
    output.header.stamp = now();
    output.sample_pose.header.stamp = output.header.stamp;
    output.smoke_ppm = static_cast<float>(filtered_smoke_ppm_);
    output.temperature_c = static_cast<float>(filtered_temperature_c_);
    output.smoke_detected = smoke_latched_;
    output.temperature_anomaly = filtered_temperature_c_ >= temperature_threshold_c_;
    output.visual_confidence = static_cast<float>(visual_confidence_);
    output.fused_confidence = static_cast<float>(std::clamp(fused, 0.0, 1.0));
    publisher_->publish(output);
  }

  bool filter_initialised_{false};
  bool smoke_latched_{false};
  double filter_alpha_{0.25};
  double filtered_smoke_ppm_{0.0};
  double filtered_temperature_c_{0.0};
  double visual_confidence_{0.0};
  double smoke_enter_ppm_{12.0};
  double smoke_exit_ppm_{8.0};
  double smoke_full_scale_ppm_{45.0};
  double baseline_smoke_ppm_{1.0};
  double temperature_threshold_c_{32.0};
  double temperature_full_scale_c_{65.0};
  double visual_weight_{0.15};

  rclcpp::Publisher<embertrace_interfaces::msg::EnvironmentalReading>::SharedPtr publisher_;
  rclcpp::Subscription<embertrace_interfaces::msg::EnvironmentalReading>::SharedPtr
    reading_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr visual_subscription_;
};

}  // namespace embertrace_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
