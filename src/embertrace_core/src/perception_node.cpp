/** Timestamp-aware local fusion; no ground-truth or other robot's samples. */
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include "embertrace_core/perception_filter.hpp"
#include "embertrace_interfaces/msg/environmental_reading.hpp"
#include "embertrace_interfaces/msg/perception_status.hpp"
#include "embertrace_interfaces/msg/visual_cue.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
namespace embertrace_core
{
class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode() : Node("perception_node")
  {
    PerceptionConfig c;
    c.alpha = declare_parameter<double>("filter_alpha", 0.25);
    c.nominal_period_s = declare_parameter<double>("nominal_period_s", 0.2);
    c.smoke_enter = declare_parameter<double>("smoke_enter_ppm", 12.0);
    c.smoke_exit = declare_parameter<double>("smoke_exit_ppm", 8.0);
    c.smoke_scale = declare_parameter<double>("smoke_full_scale_ppm", 45.0);
    c.baseline_smoke = declare_parameter<double>("baseline_smoke_ppm", 1.0);
    c.heat_enter = declare_parameter<double>("temperature_threshold_c", 32.0);
    c.heat_exit = declare_parameter<double>("temperature_exit_c", 29.0);
    c.heat_scale = declare_parameter<double>("temperature_full_scale_c", 65.0);
    c.visual_weight = declare_parameter<double>("visual_weight", 0.15);
    c.visual_threshold = declare_parameter<double>("visual_confirmation", 0.55);
    c.confirmation_threshold = declare_parameter<double>("alert_confidence", 0.74);
    c.environment_timeout_s = declare_parameter<double>("environment_timeout_s", 2.0);
    c.visual_timeout_s = declare_parameter<double>("visual_timeout_s", 1.0);
    c.future_tolerance_s = declare_parameter<double>("future_tolerance_s", 0.1);
    const int warmup = declare_parameter<int>("warmup_samples", 3);
    c.warmup_samples = warmup > 0 ? static_cast<std::size_t>(warmup) : 0;
    filter_ = std::make_unique<PerceptionFilter>(c);
    publisher_ = create_publisher<embertrace_interfaces::msg::EnvironmentalReading>(
      declare_parameter<std::string>("output_topic", "/embertrace/environment"), rclcpp::SensorDataQoS());
    status_ = create_publisher<embertrace_interfaces::msg::PerceptionStatus>(
      declare_parameter<std::string>("status_topic", "/embertrace/perception_status"), 10);
    reading_ = create_subscription<embertrace_interfaces::msg::EnvironmentalReading>(
      declare_parameter<std::string>("input_topic", "/embertrace/raw_environment"),
      rclcpp::SensorDataQoS(),
      [this](const embertrace_interfaces::msg::EnvironmentalReading::SharedPtr input) {
        const auto & p = input->sample_pose.pose.position;
        if (input->header.frame_id.empty() || input->sample_pose.header.frame_id != input->header.frame_id ||
          !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          input->sample_pose.header.stamp != input->header.stamp)
        {
          filter_->reset(); candidate_ = false; publish_status(); return;
        }
        const auto result = filter_->update({rclcpp::Time(input->header.stamp).seconds(),
          input->smoke_ppm, input->temperature_c, input->wind_speed_mps,
          input->wind_direction_rad}, now().seconds());
        candidate_ = result && result->confirmed_candidate;
        if (result && result->valid) {
          auto out = *input;
          // Preserve acquisition timestamps; downstream age checks must see latency.
          out.smoke_ppm = static_cast<float>(result->smoke);
          out.temperature_c = static_cast<float>(result->temperature);
          out.smoke_detected = result->smoke_detected;
          out.temperature_anomaly = result->temperature_anomaly;
          out.visual_confidence = static_cast<float>(result->visual);
          out.fused_confidence = static_cast<float>(result->confidence);
          publisher_->publish(out);
        }
        publish_status();
      });
    visual_ = create_subscription<embertrace_interfaces::msg::VisualCue>(
      declare_parameter<std::string>("visual_topic", "/embertrace/visual_cue"), rclcpp::SensorDataQoS(),
      [this](const embertrace_interfaces::msg::VisualCue::SharedPtr input) {
        filter_->set_visual(rclcpp::Time(input->header.stamp).seconds(), input->confidence,
          input->valid, now().seconds());
      });
    timer_ = create_wall_timer(200ms, [this]() {publish_status();});
  }
private:
  void publish_status()
  {
    embertrace_interfaces::msg::PerceptionStatus s;
    s.header.stamp = now();
    s.environment_fresh = filter_->environment_fresh(now().seconds());
    s.visual_fresh = filter_->visual_fresh(now().seconds());
    s.confirmation_candidate = s.environment_fresh && candidate_;
    s.rejected_samples = filter_->rejected_samples();
    s.consecutive_samples = static_cast<std::uint32_t>(filter_->consecutive_samples());
    status_->publish(s);
  }
  bool candidate_{false};
  std::unique_ptr<PerceptionFilter> filter_;
  rclcpp::Publisher<embertrace_interfaces::msg::EnvironmentalReading>::SharedPtr publisher_;
  rclcpp::Publisher<embertrace_interfaces::msg::PerceptionStatus>::SharedPtr status_;
  rclcpp::Subscription<embertrace_interfaces::msg::EnvironmentalReading>::SharedPtr reading_;
  rclcpp::Subscription<embertrace_interfaces::msg::VisualCue>::SharedPtr visual_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
