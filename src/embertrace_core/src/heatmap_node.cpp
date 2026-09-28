/**
 * RViz smoke-sample heatmap.
 *
 * COMPLETE BASELINE: stores a bounded, spatially down-sampled history and publishes
 * blue-to-red cylinders. This node is operator-only and is not an autonomy input.
 *
 * TODO(student): add a proper interpolated grid, a legend and CSV/bag export.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "embertrace_interfaces/msg/environmental_reading.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace embertrace_core
{

class HeatmapNode : public rclcpp::Node
{
public:
  HeatmapNode()
  : Node("heatmap_node")
  {
    max_samples_ = std::max<int>(10, declare_parameter<int>("max_samples", 300));
    minimum_spacing_m_ = std::max(0.05, declare_parameter<double>("minimum_spacing_m", 0.35));
    display_max_smoke_ppm_ = std::max(
      1.0, declare_parameter<double>("display_max_smoke_ppm", 60.0));

    publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/embertrace/sample_markers", rclcpp::QoS(1).transient_local().reliable());
    subscription_ = create_subscription<embertrace_interfaces::msg::EnvironmentalReading>(
      "/embertrace/environment", rclcpp::SensorDataQoS(),
      std::bind(&HeatmapNode::on_reading, this, std::placeholders::_1));
  }

private:
  void on_reading(const embertrace_interfaces::msg::EnvironmentalReading::SharedPtr reading)
  {
    if (!samples_.empty()) {
      const auto & previous = samples_.back().sample_pose.pose.position;
      const auto & current = reading->sample_pose.pose.position;
      if (std::hypot(current.x - previous.x, current.y - previous.y) < minimum_spacing_m_) {
        return;
      }
    }

    samples_.push_back(*reading);
    if (samples_.size() > static_cast<std::size_t>(max_samples_)) {
      samples_.erase(samples_.begin());
    }
    publish_markers();
  }

  void publish_markers()
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    int id = 1;
    for (const auto & sample : samples_) {
      visualization_msgs::msg::Marker marker;
      marker.header = sample.sample_pose.header;
      marker.header.stamp = now();
      marker.ns = "smoke_samples";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = sample.sample_pose.pose;
      marker.pose.position.z = 0.06;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.45;
      marker.scale.y = 0.45;
      marker.scale.z = 0.10;
      const double level = std::clamp(
        static_cast<double>(sample.smoke_ppm) / display_max_smoke_ppm_, 0.0, 1.0);
      marker.color.r = static_cast<float>(level);
      marker.color.g = static_cast<float>(1.0 - std::abs(2.0 * level - 1.0));
      marker.color.b = static_cast<float>(1.0 - level);
      marker.color.a = 0.85F;
      array.markers.push_back(marker);
    }
    publisher_->publish(array);
  }

  int max_samples_{300};
  double minimum_spacing_m_{0.35};
  double display_max_smoke_ppm_{60.0};
  std::vector<embertrace_interfaces::msg::EnvironmentalReading> samples_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Subscription<embertrace_interfaces::msg::EnvironmentalReading>::SharedPtr subscription_;
};

}  // namespace embertrace_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::HeatmapNode>());
  rclcpp::shutdown();
  return 0;
}
