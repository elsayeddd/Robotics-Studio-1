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
#include <stdexcept>

#include "embertrace_interfaces/msg/environmental_reading.hpp"
#include "embertrace_interfaces/msg/perception_status.hpp"
#include "embertrace_core/perception_filter.hpp"
#include "embertrace_interfaces/msg/fire_alert.hpp"
#include "embertrace_interfaces/msg/mission_state.hpp"
#include "embertrace_interfaces/srv/operator_command.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

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
    require_thermal_ = declare_parameter<bool>("require_thermal_confirmation", false);
    localise_before_hold_ = declare_parameter<bool>("localise_before_hold", false);
    reading_timeout_s_ = declare_parameter<double>("reading_timeout_s", 2.0);
    if (!std::isfinite(reading_timeout_s_) || reading_timeout_s_ <= 0) {
      throw std::invalid_argument("reading_timeout_s must be finite and positive");
    }
    max_estimate_age_s_ = declare_parameter<double>("max_estimate_age_s", 5.0);
    max_estimate_stddev_m_ = declare_parameter<double>("max_estimate_stddev_m", 4.0);
    require_scout_return_ = declare_parameter<bool>("require_scout_return", false);
    aerial_mission_ = declare_parameter<bool>("aerial_mission", false);
    verification_timeout_s_ = declare_parameter<double>("verification_timeout_s", 15.0);
    verification_retry_s_ = declare_parameter<double>("verification_retry_s", 5.0);
    scout_retry_s_ = declare_parameter<double>("scout_retry_s", 30.0);
    if (!std::isfinite(verification_timeout_s_) || verification_timeout_s_ <= 0 ||
      !std::isfinite(verification_retry_s_) || verification_retry_s_ <= 0 ||
      !std::isfinite(scout_retry_s_) || scout_retry_s_ < 0)
    {throw std::invalid_argument("Verification timing must be finite and positive");}
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
    health_subscription_ = create_subscription<embertrace_interfaces::msg::PerceptionStatus>(
      "/embertrace/perception_status", 10,
      [this](const embertrace_interfaces::msg::PerceptionStatus::SharedPtr status) {
        perception_available_ = status->environment_fresh;
        if (!perception_available_) {detection_count_ = confirmation_count_ = 0;}
        // IR loss suppresses confirmation, while gas-only investigation can continue.
        if (require_thermal_ && !status->thermal_fresh) {confirmation_count_ = 0;}
      });
    scout_subscription_ = create_subscription<embertrace_interfaces::msg::EnvironmentalReading>(
      "/drone1/environment", rclcpp::SensorDataQoS(),
      [this](const embertrace_interfaces::msg::EnvironmentalReading::SharedPtr sample) {
        const double age = (now() - rclcpp::Time(sample->header.stamp)).seconds();
        if (state_ == MissionState::PATROL && now() >= scout_retry_after_ &&
          age >= 0 && age <= reading_timeout_s_ &&
          sample->header.frame_id == "husky1_odom" &&
          (sample->smoke_detected || sample->co2_detected || sample->temperature_anomaly) &&
          sample->fused_confidence >= track_confidence_)
        {
          transition_to(MissionState::INVESTIGATE, "Aerial environmental cue; rover must verify locally");
        }
      });
    return_complete_subscription_ = create_subscription<std_msgs::msg::Empty>(
      "/embertrace/return_complete", 10,
      [this](const std_msgs::msg::Empty::SharedPtr) {
        rover_home_ = state_ == MissionState::RETURN_TO_BASE;
        check_return_complete();
      });
    scout_return_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/drone1/return_complete", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        scout_home_ = msg->data;
        if (aerial_mission_) {rover_home_ = msg->data;}
        check_return_complete();
      });
    search_status_subscription_ = create_subscription<std_msgs::msg::String>(
      "/embertrace/search_status", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        if ((state_ == MissionState::INVESTIGATE || state_ == MissionState::TRACK ||
          state_ == MissionState::RECOVER) &&
          (msg->data == "TIMED_OUT" || msg->data == "SIGNAL_LOST"))
        {
          if (fire_confirmed_) {
            transition_to(MissionState::ALERT, "Fire confirmed; source unresolved at search deadline");
            publish_alert();
            return;
          }
          detection_count_ = confirmation_count_ = 0;
          dismissed_anomaly_ = true;
          scout_retry_after_ = now() + rclcpp::Duration::from_seconds(scout_retry_s_);
          transition_to(MissionState::PATROL, "Investigation ended; resuming patrol");
        }
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
      state_entered_ = now();
      if (next_state == MissionState::RETURN_TO_BASE) {rover_home_ = scout_home_ = false;}
      RCLCPP_INFO(
        get_logger(), "Mission transition: %s -> %s (%s)", state_name(state_),
        state_name(next_state), detail.c_str());
    }
    state_ = next_state;
    detail_ = detail;
    publish_state();
  }

  void check_return_complete()
  {
    // I wait for both vehicles so parking the rover cannot strand the scout.
    if (state_ == MissionState::RETURN_TO_BASE && rover_home_ &&
      (!require_scout_return_ || scout_home_))
    {transition_to(MissionState::IDLE, "Vehicles returned to base; waiting for START");}
  }

  void on_reading(const embertrace_interfaces::msg::EnvironmentalReading::SharedPtr reading)
  {
    const auto current = now();
    if (current < last_clock_) {
      have_reading_ = have_estimate_ = false; detection_count_ = confirmation_count_ = 0;
      verification_retry_after_ = state_entered_ = current;
    }
    last_clock_ = current;
    if (state_ == MissionState::VERIFY &&
      (current - state_entered_).seconds() > verification_timeout_s_)
    {
      confirmation_count_ = 0;
      verification_retry_after_ = current + rclcpp::Duration::from_seconds(verification_retry_s_);
      transition_to(MissionState::TRACK, "Verification timed out; continue local investigation");
      return;
    }
    if (reading->header.stamp.sec < 0) {detection_count_ = confirmation_count_ = 0; return;}
    const double age = (current - rclcpp::Time(reading->header.stamp)).seconds();
    if (age < -0.1 || age > reading_timeout_s_ || !std::isfinite(reading->fused_confidence) ||
      reading->fused_confidence < 0 || reading->fused_confidence > 1 ||
      !std::isfinite(reading->visual_confidence) ||
      reading->header.frame_id.empty() ||
      (have_reading_ && rclcpp::Time(reading->header.stamp) <= last_reading_stamp_))
    {detection_count_ = confirmation_count_ = 0; return;}
    if (have_reading_ && (current - last_reading_stamp_).seconds() > reading_timeout_s_) {
      detection_count_ = confirmation_count_ = 0;
    }
    last_receipt_ = std::chrono::steady_clock::now();
    perception_available_ = true;
    last_reading_stamp_ = rclcpp::Time(reading->header.stamp);
    latest_reading_ = *reading;
    have_reading_ = true;

    if (state_ == MissionState::IDLE || state_ == MissionState::PAUSED ||
      state_ == MissionState::EMERGENCY_STOP || state_ == MissionState::RETURN_TO_BASE ||
      state_ == MissionState::ALERT)
    {
      return;
    }

    if (fire_confirmed_ && usable_estimate()) {
      transition_to(MissionState::ALERT, "Confirmed fire source localised; holding position");
      publish_alert();
      return;
    }
    if (dismissed_anomaly_ && state_ == MissionState::PATROL) {
      if (reading->fused_confidence < investigate_confidence_ && !reading->smoke_detected &&
        !reading->co2_detected && !reading->temperature_anomaly) {dismissed_anomaly_ = false;}
      else {return;}
    }
    detection_count_ = reading->fused_confidence >= investigate_confidence_ ||
      reading->smoke_detected || reading->co2_detected || reading->temperature_anomaly ?
      detection_count_ + 1 : 0;
    confirmation_count_ = confirmation_allowed(!require_thermal_ || reading->thermal_valid,
      reading->smoke_detected || reading->co2_detected,
      reading->temperature_anomaly, reading->visual_confidence, reading->fused_confidence,
      visual_confirmation_, alert_confidence_) ?
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
        } else if (reading->fused_confidence >= verify_confidence_ && reading->temperature_anomaly &&
          current >= verification_retry_after_ && !fire_confirmed_) {
          transition_to(MissionState::VERIFY, "High smoke and thermal evidence near source");
        }
        break;
      case MissionState::RECOVER:
        if ((reading->smoke_detected || reading->co2_detected) && reading->fused_confidence >= track_confidence_) {
          transition_to(MissionState::TRACK, "Plume reacquired");
        }
        break;
      case MissionState::VERIFY:
        if (confirmation_count_ >= required_confirmation_samples_) {
          fire_confirmed_ = true;
          confirmed_confidence_ = reading->fused_confidence;
          if (localise_before_hold_ && !usable_estimate()) {
            // I report detection immediately, then keep collecting spatial evidence
            // until the source fit is usable instead of parking on an unresolved alert.
            publish_alert();
            transition_to(MissionState::TRACK, "Fire confirmed; continuing source localisation");
          } else {
            transition_to(MissionState::ALERT, "Multi-sensor fire criteria satisfied");
            publish_alert();
          }
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

    if (state_ == MissionState::EMERGENCY_STOP && command != "RESET" && command != "E_STOP_RELEASE" && command != "E_STOP") {
      response->accepted = false;
      response->message = "Emergency stop is latched; RESET is required";
      return;
    }
    if (command == "START") {
      detection_count_ = 0;
      confirmation_count_ = 0;
      have_estimate_ = false;
      verification_retry_after_ = now();
      scout_retry_after_ = now();
      dismissed_anomaly_ = false;
      fire_confirmed_ = false; confirmed_confidence_ = 0;
      transition_to(MissionState::PATROL, "Patrol started by operator");
    } else if (command == "PAUSE" && state_ != MissionState::EMERGENCY_STOP) {
      if (state_ != MissionState::PAUSED) {pre_pause_state_ = state_;}
      transition_to(MissionState::PAUSED, "Mission paused by operator");
    } else if (command == "RESUME" && state_ == MissionState::PAUSED) {
      transition_to(pre_pause_state_, "Mission resumed by operator");
    } else if (command == "RETURN" && state_ != MissionState::EMERGENCY_STOP) {
      transition_to(MissionState::RETURN_TO_BASE, "Return requested by operator");
    } else if (command == "E_STOP") {
      transition_to(MissionState::EMERGENCY_STOP, "Emergency stop latched");
    } else if (command == "RESET" ||
      (command == "E_STOP_RELEASE" && state_ == MissionState::EMERGENCY_STOP)) {
      detection_count_ = 0;
      confirmation_count_ = 0;
      have_estimate_ = false;
      verification_retry_after_ = now();
      scout_retry_after_ = now();
      dismissed_anomaly_ = false;
      fire_confirmed_ = false; confirmed_confidence_ = 0;
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
    const auto current = now();
    if (current < last_clock_) {
      have_reading_ = have_estimate_ = false;
      verification_retry_after_ = state_entered_ = current;
      detection_count_ = confirmation_count_ = 0;
      last_reading_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    last_clock_ = current;
    if (state_ != MissionState::IDLE && state_ != MissionState::ALERT &&
      state_ != MissionState::RETURN_TO_BASE && state_ != MissionState::PAUSED &&
      state_ != MissionState::EMERGENCY_STOP &&
      (!have_reading_ || !perception_available_ ||
        (current - last_reading_stamp_).seconds() > reading_timeout_s_ ||
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_receipt_).count() > reading_timeout_s_))
    {
      pre_pause_state_ = state_;
      state_ = MissionState::PAUSED;
      detail_ = "Rover environmental data stale; restore sensors then RESUME";
      detection_count_ = confirmation_count_ = 0;
    }
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

  bool usable_estimate() const
  {
    const double estimate_age_s = have_estimate_ ?
      (now() - rclcpp::Time(latest_estimate_.header.stamp)).seconds() : 0.0;
    return have_reading_ && have_estimate_ &&
      latest_estimate_.header.frame_id == latest_reading_.sample_pose.header.frame_id &&
      estimate_age_s >= 0.0 && estimate_age_s <= max_estimate_age_s_ &&
      std::isfinite(latest_estimate_.pose.covariance[0]) &&
      std::isfinite(latest_estimate_.pose.covariance[7]) &&
      latest_estimate_.pose.covariance[0] >= 0.0 &&
      latest_estimate_.pose.covariance[7] >= 0.0 &&
      std::isfinite(latest_estimate_.pose.pose.position.x) &&
      std::isfinite(latest_estimate_.pose.pose.position.y) &&
      latest_estimate_.pose.covariance[0] <=
      max_estimate_stddev_m_ * max_estimate_stddev_m_ &&
      latest_estimate_.pose.covariance[7] <=
      max_estimate_stddev_m_ * max_estimate_stddev_m_;
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
    const bool fresh_estimate = usable_estimate();
    if (fresh_estimate) {
      alert.estimated_source.header = latest_estimate_.header;
      alert.estimated_source.pose = latest_estimate_.pose.pose;
    } else {
      alert.estimated_source = latest_reading_.sample_pose;
    }
    alert.confidence = std::max(latest_reading_.fused_confidence, confirmed_confidence_);
    alert.confirmed = true;
    alert.evidence = "Persistent local gas/heat evidence with corroboration";
    if (latest_reading_.co2_valid) {alert.evidence += "; CO2=" + std::to_string(latest_reading_.co2_ppm) + " ppm";}
    if (latest_reading_.thermal_valid) {alert.evidence += "; IR peak=" + std::to_string(latest_reading_.thermal_peak_c) + " C";}
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
  double reading_timeout_s_{2.0};
  rclcpp::Time last_reading_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_clock_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<embertrace_interfaces::msg::PerceptionStatus>::SharedPtr health_subscription_;
  rclcpp::Subscription<embertrace_interfaces::msg::EnvironmentalReading>::SharedPtr scout_subscription_;
  double max_estimate_age_s_{5.0};
  double max_estimate_stddev_m_{4.0};
  bool have_reading_{false}, perception_available_{false}, require_thermal_{false};
  std::chrono::steady_clock::time_point last_receipt_;
  bool have_estimate_{false};
  embertrace_interfaces::msg::EnvironmentalReading latest_reading_;
  geometry_msgs::msg::PoseWithCovarianceStamped latest_estimate_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr return_complete_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr scout_return_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr search_status_subscription_;
  bool dismissed_anomaly_{false}, fire_confirmed_{false}, localise_before_hold_{false};
  float confirmed_confidence_{0.0F};
  bool require_scout_return_{false}, aerial_mission_{false}, rover_home_{false}, scout_home_{false};
  double verification_timeout_s_{15.0};
  double verification_retry_s_{5.0};
  double scout_retry_s_{30.0};
  rclcpp::Time scout_retry_after_{0, 0, RCL_ROS_TIME};
  rclcpp::Time verification_retry_after_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_entered_{0, 0, RCL_ROS_TIME};
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
