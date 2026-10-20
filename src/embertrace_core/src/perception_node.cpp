/** Timestamp-aware local fusion; no ground-truth or other robot's samples. */
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <stdexcept>
#include "embertrace_core/perception_filter.hpp"
#include "embertrace_interfaces/msg/environmental_reading.hpp"
#include "embertrace_interfaces/msg/perception_status.hpp"
#include "embertrace_interfaces/msg/visual_cue.hpp"
#include "embertrace_interfaces/msg/thermal_cue.hpp"
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
    c.alpha = declare_parameter<double>("filter_alpha", 0.4);
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
    c.co2_enter = declare_parameter<double>("co2_enter_ppm", 700);
    c.co2_exit = declare_parameter<double>("co2_exit_ppm", 600);
    c.co2_scale = declare_parameter<double>("co2_full_scale_ppm", 1500);
    c.baseline_co2 = declare_parameter<double>("baseline_co2_ppm", 420);
    c.require_co2 = declare_parameter<bool>("require_co2", false);
    c.require_thermal = declare_parameter<bool>("require_thermal", false);
    c.thermal_timeout_s = declare_parameter<double>("thermal_timeout_s", 1);
    drone_wind_limit_ = declare_parameter<double>("drone_wind_limit_mps", 5);
    rover_wind_limit_ = declare_parameter<double>("rover_wind_limit_mps", 12);
    if (!std::isfinite(drone_wind_limit_) || !std::isfinite(rover_wind_limit_) ||
      drone_wind_limit_ < 0 || rover_wind_limit_ <= drone_wind_limit_)
    {throw std::invalid_argument("Wind thresholds must be finite and increasing");}
    const int detections = declare_parameter<int>("detection_samples", 3);
    c.detection_samples = detections > 0 ? static_cast<std::size_t>(detections) : 0;
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
        filter_->tick(now().seconds(), steady());
        const auto & q = input->sample_pose.pose.orientation;
        const auto & p = input->sample_pose.pose.position;
        if (input->header.frame_id.empty() || input->sample_pose.header.frame_id != input->header.frame_id ||
          !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
          q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w < .5 ||
          !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          input->sample_pose.header.stamp != input->header.stamp)
        {
          filter_->invalidate_environment(); publish_status(); return;
        }
        const auto result = filter_->update({input->header.stamp.sec + input->header.stamp.nanosec * 1e-9,
          input->smoke_ppm, input->temperature_c, input->wind_speed_mps,
          input->wind_direction_rad, input->co2_ppm, input->co2_valid}, now().seconds(), steady());
        if (result && result->valid) {
          auto out = *input;
          // Preserve acquisition timestamps; downstream age checks must see latency.
          out.smoke_ppm = static_cast<float>(result->smoke);
          out.temperature_c = static_cast<float>(result->temperature);
          out.smoke_detected = result->smoke_detected;
          out.temperature_anomaly = result->temperature_anomaly;
          out.visual_confidence = static_cast<float>(result->visual);
          out.fused_confidence = static_cast<float>(result->confidence);
          out.co2_ppm = static_cast<float>(result->co2); out.co2_valid = result->co2_valid;
          out.co2_detected = result->co2_detected; out.thermal_valid = result->thermal_valid;
          out.thermal_hotspot = result->thermal_hotspot;
          out.thermal_peak_c = static_cast<float>(result->thermal_peak_c);
          latest_wind_ = input->wind_speed_mps; co2_valid_ = result->co2_valid;
          publisher_->publish(out);
        }
        publish_status();
      });
    visual_ = create_subscription<embertrace_interfaces::msg::VisualCue>(
      declare_parameter<std::string>("visual_topic", "/embertrace/visual_cue"), rclcpp::SensorDataQoS(),
      [this](const embertrace_interfaces::msg::VisualCue::SharedPtr input) {
        filter_->set_visual(input->header.stamp.sec + input->header.stamp.nanosec * 1e-9, input->confidence,
          input->valid && !input->header.frame_id.empty(), now().seconds(), steady());
      });
    thermal_ = create_subscription<embertrace_interfaces::msg::ThermalCue>(
      declare_parameter<std::string>("thermal_topic", "/embertrace/thermal_cue"), rclcpp::SensorDataQoS(),
      [this](embertrace_interfaces::msg::ThermalCue::SharedPtr input) {
        filter_->set_thermal(input->header.stamp.sec + input->header.stamp.nanosec * 1e-9,
          input->peak_temperature_c, input->hotspot,
          input->valid && !input->header.frame_id.empty(), now().seconds(), steady());
      });
    timer_ = create_wall_timer(200ms, [this]() {publish_status();});
  }
private:
  static double steady()
  {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  void publish_status()
  {
    filter_->tick(now().seconds(), steady());
    embertrace_interfaces::msg::PerceptionStatus s;
    s.header.stamp = now();
    s.environment_fresh = filter_->environment_fresh(now().seconds());
    s.visual_fresh = filter_->visual_fresh(now().seconds());
    s.confirmation_candidate = filter_->confirmation_candidate(now().seconds());
    s.thermal_fresh = filter_->thermal_fresh(now().seconds());
    s.co2_valid = s.environment_fresh && co2_valid_;
    s.wind_condition = !s.environment_fresh ? s.WIND_UNKNOWN :
      (latest_wind_ <= drone_wind_limit_ ? s.WIND_DRONE_SAFE :
      (latest_wind_ <= rover_wind_limit_ ? s.WIND_ROVER_ONLY : s.WIND_UNSAFE));
    s.rejected_samples = filter_->rejected_samples();
    s.consecutive_samples = static_cast<std::uint32_t>(filter_->consecutive_samples());
    status_->publish(s);
  }
  bool co2_valid_{false};
  double latest_wind_{0}, drone_wind_limit_{5}, rover_wind_limit_{12};
  std::unique_ptr<PerceptionFilter> filter_;
  rclcpp::Publisher<embertrace_interfaces::msg::EnvironmentalReading>::SharedPtr publisher_;
  rclcpp::Publisher<embertrace_interfaces::msg::PerceptionStatus>::SharedPtr status_;
  rclcpp::Subscription<embertrace_interfaces::msg::EnvironmentalReading>::SharedPtr reading_;
  rclcpp::Subscription<embertrace_interfaces::msg::VisualCue>::SharedPtr visual_;
  rclcpp::Subscription<embertrace_interfaces::msg::ThermalCue>::SharedPtr thermal_;
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
