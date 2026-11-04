/** Sole rover velocity output: validates mission, sensors and command heartbeats. */
#include <chrono>
#include <cmath>
#include <memory>

#include "embertrace_core/stream_watchdog.hpp"
#include "embertrace_interfaces/msg/mission_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
namespace embertrace_core {
class VelocityGuardNode : public rclcpp::Node {
 public:
  using State = embertrace_interfaces::msg::MissionState;
  VelocityGuardNode() : Node("velocity_guard_node") {
    output_ = create_publisher<geometry_msgs::msg::Twist>("/husky1/cmd_vel", 10);
    autonomy_ = create_subscription<geometry_msgs::msg::Twist>(
        "/husky1/autonomy_cmd_vel", 10,
        [this](geometry_msgs::msg::Twist::SharedPtr msg) { accept(*msg, false); });
    manual_ = create_subscription<geometry_msgs::msg::Twist>(
        "/embertrace/manual_cmd_vel", 10,
        [this](geometry_msgs::msg::Twist::SharedPtr msg) { accept(*msg, true); });
    state_ = create_subscription<State>(
        "/embertrace/mission_state", rclcpp::QoS(1).transient_local(),
        [this](State::SharedPtr msg) {
          const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
          const bool fresh = state_clock_.observe(stamp, now().seconds(), wall());
          if (!fresh && (msg->motion_allowed || msg->manual_control)) {
            return;
          }
          const bool selected = msg->active_platform == State::PLATFORM_ROVER;
          const bool changed = epoch_ != msg->mission_epoch || manual_mode_ != msg->manual_control;
          epoch_ = msg->mission_epoch;
          manual_mode_ = msg->manual_control;
          allowed_ = selected && (msg->motion_allowed || msg->manual_control);
          if (!allowed_ || changed) {
            stop();
            command_clock_.invalidate();
          }
        });
    health_ = create_subscription<std_msgs::msg::Bool>(
        "/embertrace/sensors_ready", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {
          if (!msg->data) {
            health_clock_.invalidate();
            stop();
          } else {
            health_clock_.observe(now().seconds(), now().seconds(), wall());
          }
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
      const double now_s = now().seconds(), wall_s = wall();
      state_clock_.tick(now_s);
      health_clock_.tick(now_s);
      command_clock_.tick(now_s);
      const bool ready = allowed_ && state_clock_.fresh(now_s, wall_s) &&
                         health_clock_.fresh(now_s, wall_s) && command_clock_.fresh(now_s, wall_s);
      output_->publish(ready ? latest_ : geometry_msgs::msg::Twist());
    });
  }

 private:
  static double wall() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
  void stop() {
    latest_ = geometry_msgs::msg::Twist();
    output_->publish(latest_);
  }
  void accept(const geometry_msgs::msg::Twist& msg, bool manual) {
    if (!allowed_ || manual != manual_mode_) {
      return;
    }
    for (double v :
         {msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z}) {
      if (!std::isfinite(v)) {
        stop();
        command_clock_.invalidate();
        return;
      }
    }
    if (std::abs(msg.linear.x) > 2 || std::abs(msg.linear.y) > 1 || std::abs(msg.angular.z) > 2) {
      stop();
      command_clock_.invalidate();
      return;
    }
    latest_ = msg;
    command_clock_.observe(now().seconds(), now().seconds(), wall());
  }
  bool allowed_{false}, manual_mode_{false};
  std::uint64_t epoch_{0};
  StreamWatchdog state_clock_{1.5}, health_clock_{1}, command_clock_{.5};
  geometry_msgs::msg::Twist latest_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr autonomy_, manual_;
  rclcpp::Subscription<State>::SharedPtr state_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace embertrace_core
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::VelocityGuardNode>());
  rclcpp::shutdown();
  return 0;
}
