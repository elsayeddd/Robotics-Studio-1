/**
 * EmberTrace mission decision state machine.
 *
 * COMPLETE BASELINE: owns the authoritative mission state, applies debounced
 * evidence thresholds, handles operator commands and emits a fire alert.
 *
 * TODO(student): consume Nav2 action results, add explicit timeouts for every state,
 * persist mission events, and unit-test every valid/invalid transition.
 */

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "embertrace_interfaces/msg/environmental_reading.hpp"
#include "embertrace_interfaces/msg/fire_alert.hpp"
#include "embertrace_interfaces/msg/mission_state.hpp"
#include "embertrace_interfaces/srv/operator_command.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace embertrace_core
{

class MissionManagerNode : public rclcpp::Node
{
public:
  using MissionState = embertrace_interfaces::msg::MissionState;

  MissionManagerNode()
  : Node("mission_manager_node")
  {
    investigate_confidence_ = declare_parameter<double>("investigate_confidence", 0.22);
    track_confidence_ = declare_parameter<double>("track_confidence", 0.38);
    verify_confidence_ = declare_parameter<double>("verify_confidence", 0.68);
    alert_confidence_ = declare_parameter<double>("alert_confidence", 0.74);
    lost_confidence_ = declare_parameter<double>("lost_confidence", 0.12);
    visual_confirmation_ = declare_parameter<double>("visual_confirmation", 0.55);
    max_estimate_age_s_ = declare_parameter<double>("max_estimate_age_s", 5.0);
    max_estimate_stddev_m_ = declare_parameter<double>("max_estimate_stddev_m", 4.0);
    required_detection_samples_ = std::max<int>(
      1L, declare_parameter<int>("required_detection_samples", 3));
    required_confirmation_samples_ = std::max<int>(
      1L, declare_parameter<int>("required_confirmation_samples", 3));

    state_publisher_ = create_publisher<MissionState>(
      "/embertrace/mission_state", rclcpp::QoS(1).transient_local().reliable());
    alert_publisher_ = create_publisher<embertrace_interfaces::msg::FireAlert>(
      "/embertrace/fire_alert", rclcpp::QoS(1).transient_local().reliable());
    reading_subscription_ = create_subscription<embertrace_interfaces::msg::EnvironmentalReading>(
      "/embertrace/environment", rclcpp::SensorDataQoS(),
      std::bind(&MissionManagerNode::on_reading, this, std::placeholders::_1));
    estimate_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/embertrace/source_estimate", 10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
        latest_estimate_ = *message;
        have_estimate_ = true;
      });
    command_service_ = create_service<embertrace_interfaces::srv::OperatorCommand>(
      "/embertrace/operator_command",
      std::bind(
        &MissionManagerNode::on_command, this, std::placeholders::_1, std::placeholders::_2));
    state_timer_ = create_wall_timer(500ms, std::bind(&MissionManagerNode::publish_state, this));

    transition_to(MissionState::IDLE, "Waiting for operator START");
  }

private:
  static std::string uppercase(std::string text)
  {
    std::transform(text.begin(), text.end(), text.begin(), [](const unsigned char character) {
      return static_cast<char>(std::toupper(character));
    });
    return text;
  }

  static const char * state_name(const std::uint8_t state)
  {
    switch (state) {
      case MissionState::IDLE: return "IDLE";
      case MissionState::PATROL: return "PATROL";
      case MissionState::INVESTIGATE: return "INVESTIGATE";
      case MissionState::TRACK: return "TRACK";
      case MissionState::RECOVER: return "RECOVER";
      case MissionState::VERIFY: return "VERIFY";
      case MissionState::ALERT: return "ALERT";
      case MissionState::RETURN_TO_BASE: return "RETURN_TO_BASE";
      case MissionState::PAUSED: return "PAUSED";
      case MissionState::EMERGENCY_STOP: return "EMERGENCY_STOP";
      default: return "UNKNOWN";
    }
  }

  static bool permits_motion(const std::uint8_t state)
  {
    return state == MissionState::PATROL || state == MissionState::INVESTIGATE ||
      state == MissionState::TRACK || state == MissionState::RECOVER ||
      state == MissionState::RETURN_TO_BASE;
  }

  void transition_to(const std::uint8_t next_state, const std::string & detail)
  {
    if (state_ != next_state) {
      RCLCPP_INFO(
        get_logger(), "Mission transition: %s -> %s (%s)", state_name(state_),
        state_name(next_state), detail.c_str());
    }
    state_ = next_state;
    detail_ = detail;
    publish_state();
  }

  void on_reading(const embertrace_interfaces::msg::EnvironmentalReading::SharedPtr reading)
  {
    latest_reading_ = *reading;
    have_reading_ = true;

    if (state_ == MissionState::IDLE || state_ == MissionState::PAUSED ||
      state_ == MissionState::EMERGENCY_STOP || state_ == MissionState::RETURN_TO_BASE ||
      state_ == MissionState::ALERT)
    {
      return;
    }

    detection_count_ = reading->fused_confidence >= investigate_confidence_ || reading->smoke_detected ?
      detection_count_ + 1 : 0;
    confirmation_count_ = reading->fused_confidence >= alert_confidence_ &&
      (reading->temperature_anomaly || reading->visual_confidence >= visual_confirmation_) ?
      confirmation_count_ + 1 : 0;

    switch (state_) {
      case MissionState::PATROL:
        if (detection_count_ >= required_detection_samples_) {
          transition_to(MissionState::INVESTIGATE, "Persistent smoke evidence detected");
        }
        break;
      case MissionState::INVESTIGATE:
        if (reading->fused_confidence >= track_confidence_) {
          transition_to(MissionState::TRACK, "Plume signal strong enough to track");
        }
        break;
      case MissionState::TRACK:
        if (reading->fused_confidence < lost_confidence_) {
          transition_to(MissionState::RECOVER, "Plume signal lost; begin crosswind cast");
        } else if (reading->fused_confidence >= verify_confidence_ && reading->temperature_anomaly) {
          transition_to(MissionState::VERIFY, "High smoke and thermal evidence near source");
        }
        break;
      case MissionState::RECOVER:
        if (reading->smoke_detected && reading->fused_confidence >= track_confidence_) {
          transition_to(MissionState::TRACK, "Plume reacquired");
        }
        break;
      case MissionState::VERIFY:
        if (confirmation_count_ >= required_confirmation_samples_) {
          transition_to(MissionState::ALERT, "Multi-sensor fire criteria satisfied");
          publish_alert();
        } else if (reading->fused_confidence < track_confidence_) {
          transition_to(MissionState::TRACK, "Verification evidence weakened");
        }
        break;
      default:
        break;
    }
  }

  void on_command(
    const std::shared_ptr<embertrace_interfaces::srv::OperatorCommand::Request> request,
    std::shared_ptr<embertrace_interfaces::srv::OperatorCommand::Response> response)
  {
    const std::string command = uppercase(request->command);
    response->accepted = true;

    if (command == "START") {
      detection_count_ = 0;
      confirmation_count_ = 0;
      have_estimate_ = false;
      transition_to(MissionState::PATROL, "Patrol started by operator");
    } else if (command == "PAUSE" && state_ != MissionState::EMERGENCY_STOP) {
      pre_pause_state_ = state_;
      transition_to(MissionState::PAUSED, "Mission paused by operator");
    } else if (command == "RESUME" && state_ == MissionState::PAUSED) {
      transition_to(pre_pause_state_, "Mission resumed by operator");
    } else if (command == "RETURN" && state_ != MissionState::EMERGENCY_STOP) {
      transition_to(MissionState::RETURN_TO_BASE, "Return requested by operator");
    } else if (command == "E_STOP") {
      transition_to(MissionState::EMERGENCY_STOP, "Emergency stop latched");
    } else if (command == "RESET") {
      detection_count_ = 0;
      confirmation_count_ = 0;
      have_estimate_ = false;
      transition_to(MissionState::IDLE, "System reset; waiting for START");
    } else {
      response->accepted = false;
      response->message = "Command is unknown or invalid in the current state";
      return;
    }

    response->message = std::string("Accepted: ") + command;
  }

  void publish_state()
  {
    MissionState message;
    message.header.stamp = now();
    message.header.frame_id = "husky1_odom";
    message.state = state_;
    message.state_name = state_name(state_);
    message.detail = detail_;
    message.motion_allowed = permits_motion(state_);
    message.progress = state_ == MissionState::IDLE ? 0.0F :
      (state_ == MissionState::ALERT || state_ == MissionState::RETURN_TO_BASE ? 1.0F : 0.5F);
    state_publisher_->publish(message);
  }

  void publish_alert()
  {
    if (!have_reading_) {
      return;
    }
    embertrace_interfaces::msg::FireAlert alert;
    alert.header.stamp = now();
    alert.header.frame_id = latest_reading_.sample_pose.header.frame_id;
    alert.severity = embertrace_interfaces::msg::FireAlert::CRITICAL;
    // Only a recent localisation fit may be reported as the source. The robot
    // pose is a fallback location when the source is still unresolved.
    const double estimate_age_s = have_estimate_ ?
      (now() - rclcpp::Time(latest_estimate_.header.stamp)).seconds() : 0.0;
    const bool fresh_estimate = have_estimate_ &&
      latest_estimate_.header.frame_id == latest_reading_.sample_pose.header.frame_id &&
      estimate_age_s >= 0.0 && estimate_age_s <= max_estimate_age_s_ &&
      std::isfinite(latest_estimate_.pose.covariance[0]) &&
      std::isfinite(latest_estimate_.pose.covariance[7]) &&
      latest_estimate_.pose.covariance[0] <=
      max_estimate_stddev_m_ * max_estimate_stddev_m_ &&
      latest_estimate_.pose.covariance[7] <=
      max_estimate_stddev_m_ * max_estimate_stddev_m_;
    if (fresh_estimate) {
      alert.estimated_source.header = latest_estimate_.header;
      alert.estimated_source.pose = latest_estimate_.pose.pose;
    } else {
      alert.estimated_source = latest_reading_.sample_pose;
    }
    alert.confidence = latest_reading_.fused_confidence;
    alert.confirmed = true;
    alert.evidence = "Persistent smoke plus thermal or visual confirmation";
    if (!fresh_estimate) {
      alert.evidence += "; source location unresolved (pose is latest sample)";
    }
    alert.recommended_action = "Review the map, notify the ranger and keep personnel clear";
    alert_publisher_->publish(alert);
  }

  std::uint8_t state_{MissionState::IDLE};
  std::uint8_t pre_pause_state_{MissionState::IDLE};
  std::string detail_{"Starting"};
  int detection_count_{0};
  int confirmation_count_{0};
  int required_detection_samples_{3};
  int required_confirmation_samples_{3};
  double investigate_confidence_{0.22};
  double track_confidence_{0.38};
  double verify_confidence_{0.68};
  double alert_confidence_{0.74};
  double lost_confidence_{0.12};
  double visual_confirmation_{0.55};
  double max_estimate_age_s_{5.0};
  double max_estimate_stddev_m_{4.0};
  bool have_reading_{false};
  bool have_estimate_{false};
  embertrace_interfaces::msg::EnvironmentalReading latest_reading_;
  geometry_msgs::msg::PoseWithCovarianceStamped latest_estimate_;

  rclcpp::Publisher<MissionState>::SharedPtr state_publisher_;
  rclcpp::Publisher<embertrace_interfaces::msg::FireAlert>::SharedPtr alert_publisher_;
  rclcpp::Subscription<embertrace_interfaces::msg::EnvironmentalReading>::SharedPtr
    reading_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    estimate_subscription_;
  rclcpp::Service<embertrace_interfaces::srv::OperatorCommand>::SharedPtr command_service_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};

}  // namespace embertrace_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::MissionManagerNode>());
  rclcpp::shutdown();
  return 0;
}
