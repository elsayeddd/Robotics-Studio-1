/** Connected colour patches and consecutive frames; a simulation cue, not an ML detector. */
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include "embertrace_core/image_evidence.hpp"
#include "embertrace_core/visual_classifier.hpp"
#include "embertrace_interfaces/msg/visual_cue.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
namespace embertrace_core
{
class CameraVerifierNode : public rclcpp::Node
{
public:
  CameraVerifierNode() : Node("camera_verifier_node")
  {
    stride_ = declare_parameter<int>("sample_stride", 8);
    target_ = declare_parameter<double>("target_fraction", .025);
    cluster_ = declare_parameter<int>("minimum_cluster", 4);
    if (stride_ < 1 || cluster_ < 1 || !std::isfinite(target_) || target_ <= 0 || target_ > 1)
    {throw std::invalid_argument("Invalid RGB sampling/cluster/target parameters");}
    evidence_ = std::make_unique<ImageEvidence>(declare_parameter<int>("required_frames", 2),
      declare_parameter<double>("image_timeout_s", 1));
    cue_ = create_publisher<embertrace_interfaces::msg::VisualCue>(
      declare_parameter<std::string>("output_topic", "/embertrace/visual_cue"), rclcpp::SensorDataQoS());
    legacy_ = create_publisher<std_msgs::msg::Float32>(
      declare_parameter<std::string>("legacy_topic", "/embertrace/visual_fire_cue"), rclcpp::SensorDataQoS());
    image_ = create_subscription<sensor_msgs::msg::Image>(
      declare_parameter<std::string>("image_topic", "/husky1/camera/image"), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr image) {
        const double stamp = image->header.stamp.sec + image->header.stamp.nanosec * 1e-9;
        const auto score = flame_colour_score(image->data, image->width, image->height,
          image->step, image->encoding, stride_, target_, cluster_);
        embertrace_interfaces::msg::VisualCue cue; cue.header = image->header;
        cue.valid = evidence_->observe(stamp, score.has_value() && !cue.header.frame_id.empty(),
          score && *score >= .5, now().seconds(), steady());
        if (cue.valid && evidence_->positive()) {cue.confidence = static_cast<float>(*score);}
        publish(cue);
      });
    watchdog_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      evidence_->tick(now().seconds(), steady());
      if (!evidence_->available()) {
        embertrace_interfaces::msg::VisualCue cue; cue.header.stamp = now(); publish(cue);
      }
    });
  }
private:
  static double steady()
  {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  void publish(const embertrace_interfaces::msg::VisualCue & cue)
  {
    cue_->publish(cue); std_msgs::msg::Float32 legacy;
    legacy.data = cue.valid ? cue.confidence : 0; legacy_->publish(legacy);
  }
  int stride_{8}, cluster_{4};
  double target_{.025};
  std::unique_ptr<ImageEvidence> evidence_;
  rclcpp::Publisher<embertrace_interfaces::msg::VisualCue>::SharedPtr cue_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr legacy_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};
}
int main(int argc, char ** argv)
{rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<embertrace_core::CameraVerifierNode>());
  rclcpp::shutdown(); return 0;}
