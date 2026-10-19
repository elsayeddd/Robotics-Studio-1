/** Acquisition-time health for every bridged sensor on one vehicle. */
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "embertrace_interfaces/msg/environmental_reading.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
using namespace std::chrono_literals;
namespace embertrace_core
{
class SensorHealthNode : public rclcpp::Node
{
public:
  SensorHealthNode() : Node("sensor_health_node")
  {
    const auto robot = declare_parameter<std::string>("robot", "husky1");
    const auto root = "/" + robot;
    timeout_ = declare_parameter<double>("sensor_timeout_s", 2.0);
    monitor<nav_msgs::msg::Odometry>(root + "/odometry", true);
    monitor<sensor_msgs::msg::Imu>(root + "/imu", true);
    monitor<sensor_msgs::msg::LaserScan>(root + "/scan", true);
    monitor<sensor_msgs::msg::Image>(root + "/camera/image", true);
    monitor<sensor_msgs::msg::Image>(root + "/camera/depth/image", true);
    monitor<sensor_msgs::msg::PointCloud2>(root + "/scan/points", false);
    monitor<sensor_msgs::msg::PointCloud2>(root + "/camera/depth/points", false);
    monitor<embertrace_interfaces::msg::EnvironmentalReading>(
      declare_parameter<std::string>("environment_topic", "/embertrace/raw_environment"), true);
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    ready_ = create_publisher<std_msgs::msg::Bool>(
      declare_parameter<std::string>("ready_topic", "/embertrace/sensors_ready"), 10);
    timer_ = create_wall_timer(200ms, [this]() {publish();});
  }
private:
  struct Stream {bool required{false}; bool seen{false}; double stamp{0};};
  template<typename Message> void monitor(const std::string & topic, bool required)
  {
    streams_[topic].required = required;
    subscriptions_.push_back(create_subscription<Message>(topic, rclcpp::SensorDataQoS(),
      [this, topic](const typename Message::SharedPtr msg) {
        auto & stream = streams_.at(topic);
        const double stamp = rclcpp::Time(msg->header.stamp).seconds();
        const double clock = now().seconds();
        if (clock < last_clock_) {for (auto & entry : streams_) {entry.second.seen = false;}}
        last_clock_ = clock;
        if (stamp <= 0 || clock - stamp < -0.1 || clock - stamp > timeout_ ||
          (stream.seen && stamp <= stream.stamp)) {return;}
        stream.stamp = stamp; stream.seen = true;
      }));
  }
  void publish()
  {
    const double clock = now().seconds();
    if (clock < last_clock_) {for (auto & entry : streams_) {entry.second.seen = false;}}
    last_clock_ = clock;
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    std_msgs::msg::Bool ready;
    ready.data = true;
    for (const auto & entry : streams_) {
      const auto & stream = entry.second;
      const double age = clock - stream.stamp;
      const bool fresh = stream.seen && age >= -0.1 && age <= timeout_;
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "sensor:" + entry.first;
      status.hardware_id = entry.first;
      status.level = fresh ? status.OK : (stream.required ? status.ERROR : status.WARN);
      status.message = fresh ? "Fresh" : (stream.seen ? "Stale" : "Waiting for first sample");
      diagnostic_msgs::msg::KeyValue value;
      value.key = "age_s"; value.value = stream.seen ? std::to_string(age) : "unavailable";
      status.values.push_back(value);
      array.status.push_back(status);
      if (stream.required && !fresh) {ready.data = false;}
    }
    diagnostics_->publish(array);
    ready_->publish(ready);
  }
  double timeout_{2.0}, last_clock_{-1};
  std::map<std::string, Stream> streams_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::SensorHealthNode>());
  rclcpp::shutdown(); return 0;
}
