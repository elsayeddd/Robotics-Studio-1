/** Sole rover cmd_vel publisher: gates Nav2/teleop on mission and sensor heartbeats. */
#include <chrono>
#include <cmath>
#include <memory>
#include "embertrace_interfaces/msg/mission_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
using namespace std::chrono_literals;
namespace embertrace_core
{
class VelocityGuardNode : public rclcpp::Node
{
public:
  VelocityGuardNode() : Node("velocity_guard_node")
  {
    output_ = create_publisher<geometry_msgs::msg::Twist>("/husky1/cmd_vel", 10);
    command_ = create_subscription<geometry_msgs::msg::Twist>("/husky1/autonomy_cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
          !std::isfinite(msg->linear.z) || !std::isfinite(msg->angular.x) ||
          !std::isfinite(msg->angular.y) || !std::isfinite(msg->angular.z)) {return;}
        latest_ = *msg; command_stamp_ = now().seconds();
      });
    state_ = create_subscription<embertrace_interfaces::msg::MissionState>(
      "/embertrace/mission_state", rclcpp::QoS(1).transient_local().reliable(),
      [this](const embertrace_interfaces::msg::MissionState::SharedPtr msg) {
        allowed_ = msg->motion_allowed;
        state_stamp_ = rclcpp::Time(msg->header.stamp).seconds();
        if (!allowed_) {latest_ = geometry_msgs::msg::Twist(); output_->publish(latest_);}
      });
    health_ = create_subscription<std_msgs::msg::Bool>("/embertrace/sensors_ready", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        ready_ = msg->data; health_stamp_ = now().seconds();
      });
    timer_ = create_wall_timer(50ms, [this]() {
      const double clock = now().seconds();
      const auto fresh = [clock](double stamp, double timeout) {
        return stamp >= 0 && clock >= stamp && clock - stamp <= timeout;
      };
      const bool can_move = allowed_ && ready_ && fresh(state_stamp_, 1.5) &&
        fresh(health_stamp_, 1.0) && fresh(command_stamp_, 0.5);
      output_->publish(can_move ? latest_ : geometry_msgs::msg::Twist());
    });
  }
private:
  bool allowed_{false}, ready_{false};
  double state_stamp_{-1}, health_stamp_{-1}, command_stamp_{-1};
  geometry_msgs::msg::Twist latest_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_;
  rclcpp::Subscription<embertrace_interfaces::msg::MissionState>::SharedPtr state_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::VelocityGuardNode>());
  rclcpp::shutdown(); return 0;
}
