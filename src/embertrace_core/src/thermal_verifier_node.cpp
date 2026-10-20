/** C1 R2: decode radiometric IR pixels, find a contiguous hotspot and debounce frames. */
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include "embertrace_core/image_evidence.hpp"
#include "embertrace_core/thermal_detector.hpp"
#include "embertrace_interfaces/msg/thermal_cue.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
namespace embertrace_core
{
class ThermalVerifierNode : public rclcpp::Node
{
public:
  ThermalVerifierNode() : Node("thermal_verifier_node")
  {
    resolution_ = declare_parameter<double>("kelvin_per_unit", .01);
    threshold_ = declare_parameter<double>("hotspot_threshold_c", 50);
    fraction_ = declare_parameter<double>("minimum_hot_fraction", .0005);
    const int cluster = declare_parameter<int>("minimum_cluster", 4);
    if (!std::isfinite(resolution_) || resolution_ <= 0 || !std::isfinite(threshold_) ||
      !std::isfinite(fraction_) || fraction_ < 0 || fraction_ > 1 || cluster < 1)
    {throw std::invalid_argument("Invalid thermal detector parameters");}
    cluster_ = static_cast<std::size_t>(cluster);
    evidence_ = std::make_unique<ImageEvidence>(declare_parameter<int>("required_frames", 2),
      declare_parameter<double>("image_timeout_s", 1));
    publisher_ = create_publisher<embertrace_interfaces::msg::ThermalCue>(
      declare_parameter<std::string>("output_topic", "/embertrace/thermal_cue"), rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      declare_parameter<std::string>("image_topic", "/husky1/thermal/image"), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr image) {
        const auto r = thermal_hotspot(image->data, image->width, image->height, image->step,
          image->encoding, image->is_bigendian != 0, resolution_, threshold_, cluster_, fraction_);
        embertrace_interfaces::msg::ThermalCue cue;
        cue.header = image->header;
        // Convert manually so malformed negative timestamps cannot throw from rclcpp::Time.
        const double stamp = image->header.stamp.sec + image->header.stamp.nanosec * 1e-9;
        cue.valid = evidence_->observe(stamp, r.has_value() && !cue.header.frame_id.empty(),
          r && r->hotspot, now().seconds(), steady());
        if (cue.valid) {
          cue.hotspot = evidence_->positive(); cue.peak_temperature_c = static_cast<float>(r->peak_c);
          cue.hot_pixels = static_cast<std::uint32_t>(r->hot_pixels);
          cue.hot_fraction = static_cast<float>(r->hot_fraction);
        }
        publisher_->publish(cue);
      });
    watchdog_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      evidence_->tick(now().seconds(), steady());
      if (!evidence_->available()) {
        embertrace_interfaces::msg::ThermalCue cue; cue.header.stamp = now();
        publisher_->publish(cue);  // Explicitly unavailable, never a positive heartbeat.
      }
    });
  }
private:
  static double steady()
  {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  double resolution_{.01}, threshold_{50}, fraction_{.0005};
  std::size_t cluster_{4};
  std::unique_ptr<ImageEvidence> evidence_;
  rclcpp::Publisher<embertrace_interfaces::msg::ThermalCue>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};
}
int main(int argc, char ** argv)
{rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<embertrace_core::ThermalVerifierNode>());
  rclcpp::shutdown(); return 0;}
