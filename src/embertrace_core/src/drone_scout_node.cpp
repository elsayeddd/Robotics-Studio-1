/** Bounded aerial sweep of a velocity-controlled sensor proxy (not an autopilot). */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include "embertrace_interfaces/msg/mission_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
using namespace std::chrono_literals;
namespace embertrace_core
{
class DroneScoutNode : public rclcpp::Node
{
public:
  using State = embertrace_interfaces::msg::MissionState;
  DroneScoutNode() : Node("drone_scout_node")
  {
    altitude_ = declare_parameter<double>("altitude_m", 8.0);
    speed_ = declare_parameter<double>("max_speed_mps", 1.5);
    xs_ = declare_parameter<std::vector<double>>("sweep_x", {-12.0, 12.0, 12.0, -12.0, -12.0, 12.0});
    ys_ = declare_parameter<std::vector<double>>("sweep_y", {-8.0, -8.0, 0.0, 0.0, 8.0, 8.0});
    if (xs_.empty() || xs_.size() != ys_.size() || altitude_ < 6 || altitude_ > 12 ||
      !std::isfinite(altitude_) || !std::isfinite(speed_) || speed_ <= 0 || speed_ > 3)
    {throw std::invalid_argument("Invalid scout altitude, speed or sweep route");}
    for (std::size_t i = 0; i < xs_.size(); ++i) {
      if (!std::isfinite(xs_[i]) || !std::isfinite(ys_[i]) ||
        std::abs(xs_[i]) > 18 || std::abs(ys_[i]) > 18)
      {throw std::invalid_argument("Scout route outside demo geofence");}
    }
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    geometry_msgs::msg::TransformStamped camera;
    camera.header.stamp = now(); camera.header.frame_id = "drone1_base_link";
    camera.child_frame_id = "drone1_camera_link";
    camera.transform.translation.z = -0.12;
    camera.transform.rotation.y = std::sin(3.14159265358979323846 / 4.0);
    camera.transform.rotation.w = std::cos(3.14159265358979323846 / 4.0);
    geometry_msgs::msg::TransformStamped lidar;
    lidar.header = camera.header; lidar.child_frame_id = "drone1_lidar_link";
    lidar.transform.translation.z = 0.25;
    lidar.transform.rotation.w = 1.0;
    static_tf_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{camera, lidar});
    command_ = create_publisher<geometry_msgs::msg::Twist>("/drone1/cmd_vel", 10);
    odom_ = create_subscription<nav_msgs::msg::Odometry>("/drone1/odometry", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto & p = msg->pose.pose.position;
        const double age = (now() - rclcpp::Time(msg->header.stamp)).seconds();
        if (msg->header.frame_id != "husky1_odom" || age < -0.1 || age > 1.0 ||
          !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {return;}
        pose_ = *msg; have_pose_ = true;
        geometry_msgs::msg::TransformStamped transform;
        transform.header = msg->header;
        transform.child_frame_id = "drone1_base_link";
        transform.transform.translation.x = p.x;
        transform.transform.translation.y = p.y;
        transform.transform.translation.z = p.z;
        transform.transform.rotation = msg->pose.pose.orientation;
        tf_->sendTransform(transform);
      });
    state_ = create_subscription<State>("/embertrace/mission_state",
      rclcpp::QoS(1).transient_local().reliable(), [this](const State::SharedPtr msg) {
        state_value_ = msg->state; allowed_ = msg->motion_allowed;
        state_stamp_ = rclcpp::Time(msg->header.stamp).seconds();
        if (!allowed_) {command_->publish(geometry_msgs::msg::Twist());}
        if (state_value_ == State::IDLE) {index_ = 0;}
      });
    health_ = create_subscription<std_msgs::msg::Bool>("/drone1/sensors_ready", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {ready_ = msg->data; health_stamp_ = now().seconds();});
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>("/drone1/scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        obstacle_ = false;
        for (const float range : msg->ranges) {
          if (std::isfinite(range) && range >= msg->range_min && range < 0.8F) {obstacle_ = true;}
        }
      });
    timer_ = create_wall_timer(50ms, [this]() {tick();});
  }
private:
  void tick()
  {
    const double clock = now().seconds();
    const double pose_stamp = have_pose_ ? rclcpp::Time(pose_.header.stamp).seconds() : -1;
    const auto fresh = [clock](double stamp, double timeout) {
      return stamp >= 0 && clock >= stamp && clock - stamp <= timeout;
    };
    geometry_msgs::msg::Twist command;
    if (allowed_ && ready_ && fresh(state_stamp_, 1.5) && fresh(health_stamp_, 1.0) &&
      fresh(pose_stamp, 1.0) && !obstacle_)
    {
      const auto & p = pose_.pose.pose.position;
      const bool returning = state_value_ == State::RETURN_TO_BASE;
      const double tx = returning ? -14.0 : xs_[index_];
      const double ty = returning ? -12.0 : ys_[index_];
      if (std::hypot(tx - p.x, ty - p.y) < 0.6 && std::abs(altitude_ - p.z) < 0.4 && !returning) {
        index_ = (index_ + 1) % xs_.size();
      }
      // The proxy remains at yaw zero; Twist linear axes match world axes.
      const double dx = tx - p.x, dy = ty - p.y;
      const double gain = std::min(0.7, speed_ / std::max(0.01, std::hypot(dx, dy)));
      if (std::abs(p.x) <= 18.5 && std::abs(p.y) <= 18.5 && p.z >= 5 && p.z <= 12.5) {
        command.linear.x = gain * dx;
        command.linear.y = gain * dy;
        command.linear.z = std::clamp(0.7 * (altitude_ - p.z), -0.5, 0.5);
      }
    }
    command_->publish(command);
  }
  std::size_t index_{0};
  bool allowed_{false}, ready_{false}, have_pose_{false}, obstacle_{false};
  std::uint8_t state_value_{State::IDLE};
  double altitude_{8}, speed_{1.5}, state_stamp_{-1}, health_stamp_{-1};
  std::vector<double> xs_, ys_;
  nav_msgs::msg::Odometry pose_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<State>::SharedPtr state_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::DroneScoutNode>());
  rclcpp::shutdown(); return 0;
}
