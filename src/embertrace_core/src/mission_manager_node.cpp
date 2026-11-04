/** ROS adapter for the tested, platform-independent mission policy.
 * A single selected vehicle supplies local confirmation and source-estimation data.
 * The other vehicle's observations remain available for display and independent tests.
 */
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "embertrace_core/mission_logic.hpp"
#include "embertrace_interfaces/msg/environmental_reading.hpp"
#include "embertrace_interfaces/msg/fire_alert.hpp"
#include "embertrace_interfaces/msg/mission_state.hpp"
#include "embertrace_interfaces/msg/perception_status.hpp"
#include "embertrace_interfaces/srv/operator_command.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
namespace embertrace_core {
class MissionManagerNode : public rclcpp::Node {
 public:
  using Reading = embertrace_interfaces::msg::EnvironmentalReading;
  using State = embertrace_interfaces::msg::MissionState;
  MissionManagerNode() : Node("mission_manager_node") {
    MissionConfig c;
    c.localise_before_hold = declare_parameter<bool>("localise_before_hold", false);
    c.localisation_timeout_s = declare_parameter<double>("localisation_timeout_s", 120);
    c.rover_enabled = declare_parameter<bool>("rover_enabled", true);
    c.drone_enabled = declare_parameter<bool>("drone_enabled", true);
    c.require_co2 = declare_parameter<bool>("require_co2", true);
    c.require_thermal = declare_parameter<bool>("require_thermal_confirmation", true);
#define LOAD(member, name) c.member = declare_parameter<double>(name, c.member)
    LOAD(drone_wind_limit, "drone_wind_limit_mps");
    LOAD(rover_wind_limit, "rover_wind_limit_mps");
    LOAD(wind_hysteresis, "wind_hysteresis_mps");
    LOAD(wind_alpha, "wind_filter_alpha");
    LOAD(wind_dwell_s, "wind_dwell_s");
    LOAD(sensor_timeout_s, "reading_timeout_s");
    LOAD(wind_timeout_s, "wind_timeout_s");
    LOAD(investigate_confidence, "investigate_confidence");
    LOAD(track_confidence, "track_confidence");
    LOAD(verify_confidence, "verify_confidence");
    LOAD(alert_confidence, "alert_confidence");
    LOAD(lost_confidence, "lost_confidence");
    LOAD(visual_threshold, "visual_confirmation");
    LOAD(search_timeout_s, "search_timeout_s");
    LOAD(recovery_timeout_s, "recovery_timeout_s");
    LOAD(verify_timeout_s, "verify_timeout_s");
    LOAD(verify_retry_s, "verify_retry_s");
    LOAD(return_timeout_s, "return_timeout_s");
    LOAD(alert_hold_s, "alert_hold_s");
    LOAD(low_battery_fraction, "low_battery_fraction");
#undef LOAD
    const auto count = [this](const char* name, std::size_t fallback) {
      int value = declare_parameter<int>(name, static_cast<int>(fallback));
      return value > 0 ? static_cast<std::size_t>(value) : 0;
    };
    c.detection_samples = count("required_detection_samples", c.detection_samples);
    c.confirmation_samples = count("required_confirmation_samples", c.confirmation_samples);
    c.patrol_loops = count("patrol_loops", c.patrol_loops);
    max_estimate_age_ = declare_parameter<double>("max_estimate_age_s", 5);
    max_estimate_stddev_ = declare_parameter<double>("max_estimate_stddev_m", 4);
    if (!std::isfinite(max_estimate_age_) || max_estimate_age_ <= 0 ||
        !std::isfinite(max_estimate_stddev_) || max_estimate_stddev_ <= 0) {
      throw std::invalid_argument("Invalid source-estimate age or uncertainty limit");
    }
    policy_ = std::make_unique<MissionLogic>(c);
    state_ = create_publisher<State>("/embertrace/mission_state", rclcpp::QoS(1).transient_local());
    alert_ = create_publisher<embertrace_interfaces::msg::FireAlert>(
        "/embertrace/fire_alert", rclcpp::QoS(1).transient_local());
    active_reading_ =
        create_publisher<Reading>("/embertrace/active_environment", rclcpp::SensorDataQoS());
    events_ = create_publisher<std_msgs::msg::String>("/embertrace/mission_event", 50);
    wind_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/embertrace/wind", rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
          if (msg->header.frame_id != "husky1_odom") {
            return;
          }
          const double speed = std::hypot(msg->vector.x, msg->vector.y, msg->vector.z);
          policy_->wind(speed, seconds(msg->header.stamp), now().seconds(), wall());
          publish();
        });
    for (std::size_t i = 0; i < 2; ++i) {
      const auto p = i == 0 ? Platform::Rover : Platform::Drone;
      const std::string root = i == 0 ? "/embertrace" : "/drone1";
      const std::string robot = i == 0 ? "/husky1" : "/drone1";
      readings_[i] = create_subscription<Reading>(
          root + "/environment", rclcpp::SensorDataQoS(),
          [this, p, i](Reading::SharedPtr msg) { on_reading(p, i, *msg); });
      health_[i] = create_subscription<embertrace_interfaces::msg::PerceptionStatus>(
          root + "/perception_status", 10,
          [this, p, i](embertrace_interfaces::msg::PerceptionStatus::SharedPtr msg) {
            if (seconds(msg->header.stamp) < seconds(latest_[i].header.stamp)) {
              return;
            }
            const double age = now().seconds() - seconds(msg->header.stamp);
            if (age < -.1 || age > policy_->config().sensor_timeout_s || !msg->environment_fresh ||
                (policy_->config().require_co2 && !msg->co2_valid) ||
                (policy_->config().require_thermal && !msg->thermal_fresh)) {
              if (policy_->snapshot().active == p) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Health unavailable for %s: age %.3fs, environment=%d CO2=%d IR=%d",
                    platform_name(p), age, msg->environment_fresh, msg->co2_valid, msg->thermal_fresh);
              }
              policy_->unavailable(p);
            }
          });
      batteries_[i] = create_subscription<sensor_msgs::msg::BatteryState>(
          robot + "/battery", 10, [this, p](sensor_msgs::msg::BatteryState::SharedPtr msg) {
            const double age = now().seconds() - seconds(msg->header.stamp);
            if (age >= -.1 && age <= 2) {
              policy_->battery(p, msg->percentage, now().seconds(), wall());
            }
          });
      loops_[i] = create_subscription<std_msgs::msg::UInt32>(
          robot + "/patrol_loops", 10, [this, p](std_msgs::msg::UInt32::SharedPtr msg) {
            policy_->loops(p, msg->data, now().seconds(), wall());
          });
      navigation_[i] = create_subscription<std_msgs::msg::String>(
          robot + "/mission_navigation_status", 10,
          [this, p](std_msgs::msg::String::SharedPtr msg) {
            if (msg->data == "RETURNED") {
              policy_->arrived(p, now().seconds(), wall());
            } else if (msg->data == "ABORTED" || msg->data == "TIMEOUT") {
              policy_->navigation_failed(p, now().seconds(), wall());
            }
          });
    }
    estimate_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/embertrace/source_estimate", 10,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          const double stamp = seconds(msg->header.stamp);
          const double age = now().seconds() - stamp;
          if (stamp < estimate_epoch_start_ || age < -.1 || age > max_estimate_age_) return;
          latest_estimate_ = *msg;
          have_estimate_ = true;
        });
    commands_ = create_service<embertrace_interfaces::srv::OperatorCommand>(
        "/embertrace/operator_command",
        [this](embertrace_interfaces::srv::OperatorCommand::Request::SharedPtr request,
               embertrace_interfaces::srv::OperatorCommand::Response::SharedPtr response) {
          response->accepted =
              policy_->command(request->command, now().seconds(), wall(), response->message);
          publish();
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { publish(); });
  }

 private:
  static double wall() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
  static double seconds(const builtin_interfaces::msg::Time& t) {
    return t.sec + t.nanosec * 1e-9;
  }
  void on_reading(Platform platform, std::size_t i, const Reading& msg) {
    const auto& p = msg.sample_pose.pose.position;
    const auto& q = msg.sample_pose.pose.orientation;
    if (msg.header.frame_id != "husky1_odom" ||
        msg.sample_pose.header.frame_id != msg.header.frame_id ||
        msg.sample_pose.header.stamp != msg.header.stamp || !std::isfinite(p.x) ||
        !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(q.x) || !std::isfinite(q.y) ||
        !std::isfinite(q.z) || !std::isfinite(q.w) ||
        std::abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) > .1) {
      policy_->unavailable(platform);
      return;
    }
    MissionEvidence evidence{
        seconds(msg.header.stamp), msg.fused_confidence, msg.visual_confidence, msg.smoke_detected,
        msg.temperature_anomaly,   msg.co2_detected,     msg.co2_valid,         msg.thermal_valid, usable_estimate(msg)};
    if (!policy_->evidence(platform, evidence, now().seconds(), wall())) {
      return;
    }
    latest_[i] = msg;
    if (policy_->snapshot().active == platform) {
      active_reading_->publish(msg);
    }
    publish();
  }
  void publish() {
    policy_->tick(now().seconds(), wall());
    const auto s = policy_->snapshot();
    if (s.epoch != last_epoch_) {
      have_estimate_ = false;
      estimate_epoch_start_ = now().seconds();
      last_epoch_ = s.epoch;
      if (!s.confirmed) active_alert_id_ = 0;
    }
    State msg;
    msg.header.stamp = now();
    msg.header.frame_id = "husky1_odom";
    msg.state = static_cast<std::uint8_t>(s.phase);
    msg.state_name = phase_name(s.phase);
    msg.detail = s.detail;
    msg.motion_allowed = s.motion_allowed;
    msg.active_platform = static_cast<std::uint8_t>(s.active);
    msg.preferred_platform = static_cast<std::uint8_t>(s.preferred);
    msg.mission_epoch = s.epoch;
    msg.manual_control = s.manual;
    msg.alert_acknowledged = s.acknowledged;
    msg.progress =
        s.phase == MissionPhase::Idle
            ? 0
            : (s.phase == MissionPhase::Alert || s.phase == MissionPhase::Return ? 1 : .5F);
    state_->publish(msg);
    const std::string event = std::to_string(s.epoch) + " | " + phase_name(s.phase) + " | " +
                              platform_name(s.active) + " | " + s.detail +
                              (s.acknowledged ? " | acknowledged" : "");
    if (event != last_event_) {
      std_msgs::msg::String m;
      m.data = event;
      events_->publish(m);
      last_event_ = event;
    }
    if (s.warning_sequence > last_warning_ && s.active != Platform::None) {
      last_warning_ = s.warning_sequence;
      publish_alert(s, false, ++event_sequence_);
    }
    if (s.alert_sequence > last_alert_ && s.active != Platform::None) {
      last_alert_ = s.alert_sequence;
      confirmed_reading_ = latest_[s.active == Platform::Rover ? 0 : 1];
      active_alert_id_ = ++event_sequence_;
      publish_alert(s, true, active_alert_id_);
    } else if (s.phase == MissionPhase::Alert && last_phase_ != MissionPhase::Alert &&
               active_alert_id_ != 0 && s.confirmed) {
      // The same incident gains a resolved source (or a bounded fallback); it is
      // an update, not a second fire detection.
      publish_alert(s, true, active_alert_id_);
    }
    last_phase_ = s.phase;
  }
  bool usable_estimate(const Reading& r) const {
    const double age = now().seconds() - seconds(latest_estimate_.header.stamp);
    const auto& c = latest_estimate_.pose.covariance;
    const auto& p = latest_estimate_.pose.pose.position;
    return have_estimate_ && seconds(latest_estimate_.header.stamp) >= estimate_epoch_start_ && age >= 0 && age <= max_estimate_age_ &&
           latest_estimate_.header.frame_id == r.sample_pose.header.frame_id &&
           std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(c[0]) && std::isfinite(c[7]) &&
           c[0] >= 0 && c[7] >= 0 && std::max(c[0], c[7]) <= max_estimate_stddev_ * max_estimate_stddev_;
  }
  void publish_alert(const MissionSnapshot& state, bool confirmed, std::uint64_t id) {
    const auto& r = confirmed ? confirmed_reading_ : latest_[state.active == Platform::Rover ? 0 : 1];
    embertrace_interfaces::msg::FireAlert msg;
    msg.header.stamp = now();
    msg.header.frame_id = r.sample_pose.header.frame_id;
    msg.alert_id = id;
    msg.platform = static_cast<std::uint8_t>(state.active);
    msg.severity = confirmed ? msg.CRITICAL : msg.WARNING;
    msg.confirmed = confirmed;
    msg.confidence = r.fused_confidence;
    msg.estimated_source = r.sample_pose;
    msg.source_resolved = usable_estimate(r);
    if (msg.source_resolved) {
      msg.estimated_source.header = latest_estimate_.header;
      msg.estimated_source.pose = latest_estimate_.pose.pose;
    }
    msg.smoke_ppm = r.smoke_ppm;
    msg.temperature_c = r.temperature_c;
    msg.co2_ppm = r.co2_ppm;
    msg.co2_valid = r.co2_valid;
    msg.thermal_valid = r.thermal_valid;
    msg.thermal_peak_c = r.thermal_peak_c;
    msg.wind_speed_mps = r.wind_speed_mps;
    msg.evidence = std::string(confirmed ? "Persistent local gas/heat corroboration from "
                                         : "Suspected anomaly under investigation from ") +
                   platform_name(state.active);
    if (!msg.source_resolved) {
      msg.evidence += "; location is sample pose, source not resolved";
    }
    msg.recommended_action =
        confirmed ? "Review sensor evidence and location; notify the responsible ranger"
                  : "Monitor local sensor data while the selected vehicle investigates";
    alert_->publish(msg);
  }
  std::unique_ptr<MissionLogic> policy_;
  std::array<Reading, 2> latest_;
  Reading confirmed_reading_;
  geometry_msgs::msg::PoseWithCovarianceStamped latest_estimate_;
  bool have_estimate_{false};
  MissionPhase last_phase_{MissionPhase::Idle};
  std::uint64_t active_alert_id_{0};
  double estimate_epoch_start_{0};
  double max_estimate_age_{5}, max_estimate_stddev_{4};
  std::uint64_t last_epoch_{0}, last_alert_{0}, last_warning_{0}, event_sequence_{0};
  std::string last_event_;
  rclcpp::Publisher<State>::SharedPtr state_;
  rclcpp::Publisher<Reading>::SharedPtr active_reading_;
  rclcpp::Publisher<embertrace_interfaces::msg::FireAlert>::SharedPtr alert_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr wind_;
  std::array<rclcpp::Subscription<Reading>::SharedPtr, 2> readings_;
  std::array<rclcpp::Subscription<embertrace_interfaces::msg::PerceptionStatus>::SharedPtr, 2>
      health_;
  std::array<rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr, 2> batteries_;
  std::array<rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr, 2> loops_;
  std::array<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr, 2> navigation_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr estimate_;
  rclcpp::Service<embertrace_interfaces::srv::OperatorCommand>::SharedPtr commands_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace embertrace_core
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::MissionManagerNode>());
  rclcpp::shutdown();
  return 0;
}
