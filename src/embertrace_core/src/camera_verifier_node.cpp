/**
 * Camera verifier starter implementation.
 *
 * COMPLETE BASELINE: samples pixels and reports the fraction matching a broad
 * orange/red flame heuristic. No OpenCV dependency is required.
 *
 * TODO(student): record a labelled simulation dataset, handle illumination changes,
 * add temporal consistency and compare this baseline with an OpenCV or ML detector.
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"

namespace embertrace_core
{

class CameraVerifierNode : public rclcpp::Node
{
public:
  CameraVerifierNode()
  : Node("camera_verifier_node")
  {
    const auto image_topic = declare_parameter<std::string>(
      "image_topic", "/husky1/camera/image");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/embertrace/visual_fire_cue");
    sample_stride_ = std::max(1L, declare_parameter<int>("sample_stride", 8));
    target_fraction_ = std::max(0.0001, declare_parameter<double>("target_fraction", 0.025));

    publisher_ = create_publisher<std_msgs::msg::Float32>(output_topic, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      std::bind(&CameraVerifierNode::on_image, this, std::placeholders::_1));
  }

private:
  void on_image(const sensor_msgs::msg::Image::SharedPtr image)
  {
    const bool rgb = image->encoding == sensor_msgs::image_encodings::RGB8 ||
      image->encoding == sensor_msgs::image_encodings::RGBA8;
    const bool bgr = image->encoding == sensor_msgs::image_encodings::BGR8 ||
      image->encoding == sensor_msgs::image_encodings::BGRA8;
    if (!rgb && !bgr) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Unsupported image encoding: %s", image->encoding.c_str());
      return;
    }

    const std::size_t channels =
      (image->encoding == sensor_msgs::image_encodings::RGBA8 ||
      image->encoding == sensor_msgs::image_encodings::BGRA8) ? 4U : 3U;
    std::size_t inspected = 0U;
    std::size_t fire_coloured = 0U;

    for (std::uint32_t row = 0; row < image->height; row += sample_stride_) {
      const std::size_t row_offset = static_cast<std::size_t>(row) * image->step;
      for (std::uint32_t col = 0; col < image->width; col += sample_stride_) {
        const std::size_t index = row_offset + static_cast<std::size_t>(col) * channels;
        if (index + 2U >= image->data.size()) {
          continue;
        }
        const std::uint8_t red = image->data[index + (rgb ? 0U : 2U)];
        const std::uint8_t green = image->data[index + 1U];
        const std::uint8_t blue = image->data[index + (rgb ? 2U : 0U)];
        const bool candidate = red > 160U && green > 35U && green < 205U && blue < 115U &&
          static_cast<int>(red) > static_cast<int>(green) + 35;
        fire_coloured += candidate ? 1U : 0U;
        ++inspected;
      }
    }

    std_msgs::msg::Float32 cue;
    const double fraction = inspected == 0U ? 0.0 :
      static_cast<double>(fire_coloured) / static_cast<double>(inspected);
    cue.data = static_cast<float>(std::clamp(fraction / target_fraction_, 0.0, 1.0));
    publisher_->publish(cue);
  }

  int sample_stride_{8};
  double target_fraction_{0.025};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

}  // namespace embertrace_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::CameraVerifierNode>());
  rclcpp::shutdown();
  return 0;
}
