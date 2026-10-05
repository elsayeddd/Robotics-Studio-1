/** Connected colour patches and consecutive frames; a simulation cue, not an ML detector. */
#include <algorithm>
#include <memory>
#include <string>
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
    stride_ = std::max(1, declare_parameter<int>("sample_stride", 8));
    target_ = declare_parameter<double>("target_fraction", 0.025);
    cluster_ = std::max(1, declare_parameter<int>("minimum_cluster", 4));
    frames_ = std::max(1, declare_parameter<int>("required_frames", 2));
    timeout_ = declare_parameter<double>("image_timeout_s", 1.0);
    cue_ = create_publisher<embertrace_interfaces::msg::VisualCue>(
      declare_parameter<std::string>("output_topic", "/embertrace/visual_cue"), rclcpp::SensorDataQoS());
    legacy_ = create_publisher<std_msgs::msg::Float32>(
      declare_parameter<std::string>("legacy_topic", "/embertrace/visual_fire_cue"), rclcpp::SensorDataQoS());
    image_ = create_subscription<sensor_msgs::msg::Image>(
      declare_parameter<std::string>("image_topic", "/husky1/camera/image"), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr image) {
        const double stamp = rclcpp::Time(image->header.stamp).seconds(), clock = now().seconds();
        if (clock < last_clock_) {last_stamp_ = -1; consecutive_ = 0;}
        last_clock_ = clock;
        embertrace_interfaces::msg::VisualCue cue;
        cue.header = image->header;
        const auto score = flame_colour_score(image->data, image->width, image->height,
          image->step, image->encoding, stride_, target_, cluster_);
        cue.valid = score.has_value() && stamp > 0 && clock - stamp <= timeout_ &&
          stamp - clock <= 0.1 && stamp > last_stamp_ && !image->header.frame_id.empty();
        if (cue.valid) {
          if (stamp - last_stamp_ > timeout_) {consecutive_ = 0;}
          last_stamp_ = stamp;
          consecutive_ = *score >= 0.5 ? consecutive_ + 1 : 0;
          cue.confidence = consecutive_ >= frames_ ? static_cast<float>(*score) : 0.0F;
        } else {consecutive_ = 0;}
        cue_->publish(cue);
        std_msgs::msg::Float32 legacy;
        legacy.data = cue.valid ? cue.confidence : 0.0F;
        legacy_->publish(legacy);
      });
  }
private:
  int stride_{8}, cluster_{4}, frames_{2}, consecutive_{0};
  double target_{0.025}, timeout_{1.0}, last_stamp_{-1.0}, last_clock_{-1.0};
  rclcpp::Publisher<embertrace_interfaces::msg::VisualCue>::SharedPtr cue_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr legacy_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_;
};
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::CameraVerifierNode>());
  rclcpp::shutdown();
  return 0;
}
