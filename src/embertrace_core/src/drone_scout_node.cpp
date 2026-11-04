/** Velocity-controlled aerial platform: takeoff, patrol, local investigation and landing. */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "embertrace_core/stream_watchdog.hpp"
#include "embertrace_interfaces/msg/mission_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
namespace embertrace_core {
class DroneScoutNode : public rclcpp::Node {
 public:
  using State = embertrace_interfaces::msg::MissionState;
  DroneScoutNode() : Node("drone_scout_node") {
    altitude_ = declare_parameter<double>("altitude_m", 8);
    speed_ = declare_parameter<double>("max_speed_mps", 1.5);
    xs_ = declare_parameter<std::vector<double>>("sweep_x", {-12., 12., 12., -12., -12., 12.});
    ys_ = declare_parameter<std::vector<double>>("sweep_y", {-8., -8., 0., 0., 8., 8.});
    if (xs_.empty() || xs_.size() != ys_.size() || !std::isfinite(altitude_) || altitude_ < 6 ||
        altitude_ > 12 || !std::isfinite(speed_) || speed_ <= 0 || speed_ > 3) {
      throw std::invalid_argument("Invalid aerial route parameters");
    }
    for (std::size_t i = 0; i < xs_.size(); ++i) {
      if (!std::isfinite(xs_[i]) || !std::isfinite(ys_[i]) || std::abs(xs_[i]) > 18 ||
          std::abs(ys_[i]) > 18) {
        throw std::invalid_argument("Aerial route exceeds geofence");
      }
    }
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    geometry_msgs::msg::TransformStamped camera;
    camera.header.stamp = now();
    camera.header.frame_id = "drone1_base_link";
    camera.child_frame_id = "drone1_camera_link";
    camera.transform.translation.z = -.12;
    camera.transform.rotation.y = std::sin(3.141592653589793 / 4);
    camera.transform.rotation.w = std::cos(3.141592653589793 / 4);
    auto lidar = camera;
    lidar.child_frame_id = "drone1_lidar_link";
    lidar.transform.translation.z = .25;
    lidar.transform.rotation.y = 0;
    lidar.transform.rotation.w = 1;
    static_tf_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{camera, lidar});
    output_ = create_publisher<geometry_msgs::msg::Twist>("/drone1/cmd_vel", 10);
    navigation_ = create_publisher<std_msgs::msg::String>("/drone1/mission_navigation_status", 10);
    loops_ = create_publisher<std_msgs::msg::UInt32>("/drone1/patrol_loops", 10);
    odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/drone1/odometry", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          const auto& p = msg->pose.pose.position;
          if (msg->header.frame_id != "husky1_odom" || !std::isfinite(p.x) || !std::isfinite(p.y) ||
              !std::isfinite(p.z))
            return;
          if (!pose_clock_.observe(seconds(msg->header.stamp), now().seconds(), wall())) return;
          if (seconds(msg->header.stamp) < seconds(pose_.header.stamp)) {
            have_home_ = false;
            index_ = loop_count_ = 0;
          }
          pose_ = *msg;
          if (!have_home_) {
            home_x_ = p.x;
            home_y_ = p.y;
            home_z_ = std::max(.6, p.z);
            have_home_ = true;
          }
          geometry_msgs::msg::TransformStamped transform;
          transform.header = msg->header;
          transform.child_frame_id = "drone1_base_link";
          transform.transform.translation.x = p.x;
          transform.transform.translation.y = p.y;
          transform.transform.translation.z = p.z;
          transform.transform.rotation = msg->pose.pose.orientation;
          tf_->sendTransform(transform);
        });
    state_ = create_subscription<State>(
        "/embertrace/mission_state", rclcpp::QoS(1).transient_local(),
        [this](State::SharedPtr msg) {
          const bool fresh =
              state_clock_.observe(seconds(msg->header.stamp), now().seconds(), wall());
          if (!fresh && (msg->motion_allowed || msg->manual_control)) return;
          const auto previous_phase = phase_;
          if (epoch_ != msg->mission_epoch) {
            epoch_ = msg->mission_epoch;
            goal_clock_.invalidate();
            manual_clock_.invalidate();
            returned_ = false;
          }
          selected_ = msg->active_platform == State::PLATFORM_DRONE;
          allowed_ = selected_ && (msg->motion_allowed || msg->manual_control);
          manual_mode_ = msg->manual_control;
          phase_ = msg->state;
          if (selected_ && phase_ == State::PATROL && previous_phase != State::PATROL &&
              have_home_) {
            // Resume coverage at the nearest route waypoint after a hold or investigation.
            const auto& p = pose_.pose.pose.position;
            double nearest = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < xs_.size(); ++i) {
              const double distance = std::hypot(xs_[i] - p.x, ys_[i] - p.y);
              if (distance < nearest) {
                nearest = distance;
                index_ = i;
              }
            }
          }
          if (!allowed_) output_->publish(geometry_msgs::msg::Twist());
        });
    health_ = create_subscription<std_msgs::msg::Bool>(
        "/drone1/sensors_ready", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data)
            health_clock_.observe(now().seconds(), now().seconds(), wall());
          else
            health_clock_.invalidate();
        });
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/drone1/scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
          if (!scan_clock_.observe(seconds(msg->header.stamp), now().seconds(), wall())) return;
          obstacle_ = false;
          for (float range : msg->ranges) {
            if (std::isfinite(range) && range >= msg->range_min && range < .8F) obstacle_ = true;
          }
        });
    goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/drone1/navigation_goal", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          const auto& p = msg->pose.position;
          if (msg->header.frame_id == "husky1_odom" && std::isfinite(p.x) && std::isfinite(p.y) &&
              std::abs(p.x) <= 18.5 && std::abs(p.y) <= 18.5 &&
              goal_clock_.observe(seconds(msg->header.stamp), now().seconds(), wall())) {
            target_x_ = p.x;
            target_y_ = p.y;
          }
        });
    manual_ = create_subscription<geometry_msgs::msg::Twist>(
        "/embertrace/manual_cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
          if (!manual_mode_ || !selected_) return;
          for (double v : {msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z})
            if (!std::isfinite(v)) return;
          manual_command_ = *msg;
          manual_clock_.observe(now().seconds(), now().seconds(), wall());
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { tick(); });
  }

 private:
  static double wall() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
  static double seconds(const builtin_interfaces::msg::Time& t) {
    return t.sec + t.nanosec * 1e-9;
  }
  void tick() {
    const double t = now().seconds(), w = wall();
    for (auto* clock :
         {&pose_clock_, &state_clock_, &health_clock_, &scan_clock_, &goal_clock_, &manual_clock_})
      clock->tick(t);
    geometry_msgs::msg::Twist command;
    const bool ready = allowed_ && have_home_ && pose_clock_.fresh(t, w) &&
                       state_clock_.fresh(t, w) && health_clock_.fresh(t, w) &&
                       scan_clock_.fresh(t, w);
    if (ready) {
      const auto& p = pose_.pose.pose.position;
      if (std::abs(p.x) <= 18.5 && std::abs(p.y) <= 18.5 && p.z >= .2 && p.z <= 12.5) {
        if (manual_mode_) {
          if (manual_clock_.fresh(t, w) && !obstacle_) {
            command.linear.x = std::clamp(manual_command_.linear.x, -speed_, speed_);
            command.linear.y = std::clamp(manual_command_.linear.y, -speed_, speed_);
            command.linear.z = std::clamp(manual_command_.linear.z, -.5, .5);
          }
        } else {
          const bool returning = phase_ == State::RETURN_TO_BASE;
          double x = xs_[index_], y = ys_[index_], z = altitude_;
          if (returning) {
            x = home_x_;
            y = home_y_;
            if (std::hypot(x - p.x, y - p.y) < .4) z = home_z_;
          } else if (phase_ != State::PATROL) {
            if (!goal_clock_.fresh(t, w)) {
              output_->publish(command);
              return;
            }
            x = target_x_;
            y = target_y_;
          }
          if (phase_ == State::PATROL && std::hypot(x - p.x, y - p.y) < .6 &&
              std::abs(z - p.z) < .4) {
            index_ = (index_ + 1) % xs_.size();
            if (index_ == 0) ++loop_count_;
          }
          const double dx = x - p.x, dy = y - p.y,
                       gain = std::min(.7, speed_ / std::max(.01, std::hypot(dx, dy)));
          // Ascend over the known tree-height envelope before moving horizontally.
          if (!obstacle_ && (p.z > altitude_ - .5 || returning)) {
            command.linear.x = gain * dx;
            command.linear.y = gain * dy;
          }
          if (!obstacle_) command.linear.z = std::clamp(.7 * (z - p.z), -.5, .5);
          if (returning && !returned_ && std::hypot(dx, dy) < .4 && std::abs(p.z - home_z_) < .2) {
            returned_ = true;
            command = geometry_msgs::msg::Twist();
            std_msgs::msg::String status;
            status.data = "RETURNED";
            navigation_->publish(status);
          }
        }
        // Stop outward motion before the position reaches the hard flight envelope.
        if ((p.x >= 18 && command.linear.x > 0) || (p.x <= -18 && command.linear.x < 0))
          command.linear.x = 0;
        if ((p.y >= 18 && command.linear.y > 0) || (p.y <= -18 && command.linear.y < 0))
          command.linear.y = 0;
        if ((p.z >= 12 && command.linear.z > 0) || (p.z <= .6 && command.linear.z < 0))
          command.linear.z = 0;
      }
    }
    output_->publish(command);
    std_msgs::msg::UInt32 loops;
    loops.data = static_cast<std::uint32_t>(loop_count_);
    loops_->publish(loops);
  }
  double altitude_{8}, speed_{1.5}, home_x_{-14}, home_y_{-12}, home_z_{.6}, target_x_{0},
      target_y_{0};
  bool have_home_{false}, selected_{false}, allowed_{false}, manual_mode_{false}, obstacle_{false},
      returned_{false};
  std::size_t index_{0}, loop_count_{0};
  std::uint64_t epoch_{0};
  std::uint8_t phase_{State::IDLE};
  std::vector<double> xs_, ys_;
  nav_msgs::msg::Odometry pose_;
  geometry_msgs::msg::Twist manual_command_;
  StreamWatchdog pose_clock_{1}, state_clock_{1.5}, health_clock_{1}, scan_clock_{1},
      goal_clock_{3}, manual_clock_{.5};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr loops_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<State>::SharedPtr state_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace embertrace_core
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::DroneScoutNode>());
  rclcpp::shutdown();
  return 0;
}
