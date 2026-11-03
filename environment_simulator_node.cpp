/** Simulate local smoke, CO2, scalar temperature and shared wind at measured robot poses.
 * Only simulation/evaluation code knows the source coordinates. Missing odometry,
 * scene transitions and configured dropouts withhold samples instead of inventing
 * clear-air data. Autonomous decisions consume sensor messages, never trial truth.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "embertrace_core/plume_model.hpp"
#include "embertrace_interfaces/msg/environmental_reading.hpp"
#include "embertrace_interfaces/msg/fire_state.hpp"
#include "embertrace_interfaces/msg/trial_truth.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace embertrace_core {

class EnvironmentSimulatorNode : public rclcpp::Node {
 public:
  EnvironmentSimulatorNode() : Node("environment_simulator_node") {
    const auto odometry_topic =
        declare_parameter<std::string>("odometry_topic", "/husky1/odometry");
    const auto output_topic =
        declare_parameter<std::string>("output_topic", "/embertrace/raw_environment");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "husky1_odom");
    const auto publish_rate_hz = declare_parameter<double>("publish_rate_hz", 5.0);
    const auto seed = declare_parameter<int>("random_seed", 42);
    scenario_ = declare_parameter<std::string>("scenario", "fire");
    if (scenario_ != "fire" && scenario_ != "no_fire" && scenario_ != "dust" &&
        scenario_ != "hot_surface" && scenario_ != "visual_decoy" && scenario_ != "dropout") {
      throw std::invalid_argument("Unknown demo scenario");
    }
    require_wind_ = declare_parameter<bool>("require_wind", true);
    odometry_timeout_s_ = declare_parameter<double>("odometry_timeout_s", 1.0);
    altitude_decay_m_ = declare_parameter<double>("altitude_decay_m", 8.0);
    dropout_start_s_ = declare_parameter<double>("dropout_start_s", 25.0);
    dropout_duration_s_ = declare_parameter<double>("dropout_duration_s", 8.0);
    if (!std::isfinite(publish_rate_hz) || publish_rate_hz <= 0 || publish_rate_hz > 100 ||
        !std::isfinite(odometry_timeout_s_) || odometry_timeout_s_ <= 0 ||
        !std::isfinite(altitude_decay_m_) || altitude_decay_m_ <= 0 ||
        !std::isfinite(dropout_start_s_) || dropout_start_s_ < 0 ||
        !std::isfinite(dropout_duration_s_) || dropout_duration_s_ < 0) {
      throw std::invalid_argument("Invalid environmental timing or altitude parameters");
    }
    const auto truth_topic =
        declare_parameter<std::string>("truth_topic", "/embertrace/validation/truth");
    truth_publisher_ = create_publisher<embertrace_interfaces::msg::TrialTruth>(truth_topic, 10);

    PlumeParameters parameters;
    parameters.source_x = declare_parameter<double>("source_x", 12.0);
    parameters.source_y = declare_parameter<double>("source_y", 7.0);
    parameters.emission_strength_ppm = declare_parameter<double>("emission_strength_ppm", 120.0);
    parameters.baseline_smoke_ppm = declare_parameter<double>("baseline_smoke_ppm", 1.0);
    parameters.plume_length_m = declare_parameter<double>("plume_length_m", 14.0);
    parameters.plume_width_m = declare_parameter<double>("plume_width_m", 1.6);
    parameters.plume_expansion = declare_parameter<double>("plume_expansion", 0.05);
    parameters.smoke_noise_stddev = declare_parameter<double>("smoke_noise_stddev", 2.0);
    parameters.ambient_temperature_c = declare_parameter<double>("ambient_temperature_c", 22.0);
    parameters.source_temperature_c = declare_parameter<double>("source_temperature_c", 110.0);
    parameters.thermal_decay_m = declare_parameter<double>("thermal_decay_m", 7.0);
    parameters.wind_direction_rad = declare_parameter<double>("wind_direction_rad", -2.35619449);
    parameters.wind_speed_mps = declare_parameter<double>("wind_speed_mps", 2.5);
    if (scenario_ == "visual_decoy") {
      parameters.emission_strength_ppm = 0.0;
      parameters.source_temperature_c = parameters.ambient_temperature_c;
    } else if (scenario_ == "dust") {
      parameters.source_temperature_c = parameters.ambient_temperature_c;
    } else if (scenario_ == "hot_surface") {
      parameters.emission_strength_ppm = 0.0;
    }
    source_x_ = parameters.source_x;
    source_y_ = parameters.source_y;
    evaluation_radius_ = declare_parameter<double>("evaluation_radius_m", 6);
    if (!std::isfinite(evaluation_radius_) || evaluation_radius_ <= 0) {
      throw std::invalid_argument("evaluation_radius_m must be positive");
    }
    ambient_ = parameters.ambient_temperature_c;
    baseline_ = parameters.baseline_smoke_ppm;
    plume_model_ = std::make_unique<PlumeModel>(parameters, static_cast<std::uint32_t>(seed));
    auto background = parameters;
    background.emission_strength_ppm = 0;
    background.source_temperature_c = background.ambient_temperature_c;
    background_model_ =
        std::make_unique<PlumeModel>(background, static_cast<std::uint32_t>(seed) + 2017U);
    // Reuse only the transport-field geometry: CO2 has its own baseline, emission,
    // noise and random stream. Dust is explicitly NOT a CO2 source.
    auto co2_parameters = parameters;
    co2_parameters.baseline_smoke_ppm = declare_parameter<double>("baseline_co2_ppm", 420);
    co2_parameters.emission_strength_ppm =
        declare_parameter<double>("co2_emission_strength_ppm", 900);
    co2_parameters.smoke_noise_stddev = declare_parameter<double>("co2_noise_stddev_ppm", 5);
    if (scenario_ != "fire" && scenario_ != "dropout" && scenario_ != "no_fire") {
      co2_parameters.emission_strength_ppm = 0;
    }
    if (co2_parameters.baseline_smoke_ppm < 0 || co2_parameters.emission_strength_ppm < 0 ||
        co2_parameters.smoke_noise_stddev < 0 ||
        !std::isfinite(co2_parameters.baseline_smoke_ppm) ||
        !std::isfinite(co2_parameters.emission_strength_ppm) ||
        !std::isfinite(co2_parameters.smoke_noise_stddev)) {
      throw std::invalid_argument("Invalid CO2 field parameters");
    }
    co2_baseline_ = co2_parameters.baseline_smoke_ppm;
    co2_model_ =
        std::make_unique<PlumeModel>(co2_parameters, static_cast<std::uint32_t>(seed) + 1009U);
    co2_parameters.emission_strength_ppm = 0;
    co2_background_ =
        std::make_unique<PlumeModel>(co2_parameters, static_cast<std::uint32_t>(seed) + 3011U);

    wind_subscription_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/embertrace/wind", rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::Vector3Stamped::SharedPtr message) {
          const double stamp = message->header.stamp.sec + message->header.stamp.nanosec * 1e-9;
          const double age = now().seconds() - stamp;
          const double speed = std::hypot(message->vector.x, message->vector.y, message->vector.z);
          if (message->header.frame_id != odom_frame_ || age < -.1 || age > 2 ||
              !std::isfinite(speed) || speed > 100 || stamp <= wind_stamp_) {
            return;
          }
          wind_stamp_ = stamp;
          wind_receipt_ = std::chrono::steady_clock::now();
          wind_seen_ = true;
          const double direction = std::atan2(message->vector.y, message->vector.x);
          plume_model_->set_wind(speed, direction);
          co2_model_->set_wind(speed, direction);
          background_model_->set_wind(speed, direction);
          co2_background_->set_wind(speed, direction);
        });
    fire_subscription_ = create_subscription<embertrace_interfaces::msg::FireState>(
        "/embertrace/fire_state", rclcpp::QoS(1).transient_local(),
        [this](embertrace_interfaces::msg::FireState::SharedPtr message) {
          scene_ready_ = message->rendered;
          if (message->rendered && message->state <= message->BURNT) {
            fire_state_ = message->state;
          }
        });
    fire_state_ = scenario_ == "fire" || scenario_ == "dropout" ? 1 : 0;
    reading_publisher_ = create_publisher<embertrace_interfaces::msg::EnvironmentalReading>(
        output_topic, rclcpp::SensorDataQoS());
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic, rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr message) {
          const auto& p = message->pose.pose.position;
          const double age = (now() - rclcpp::Time(message->header.stamp)).seconds();
          if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
              message->header.frame_id != odom_frame_ || age < -0.1 || age > odometry_timeout_s_) {
            return;
          }
          last_odometry_receipt_ = std::chrono::steady_clock::now();
          latest_odometry_ = *message;
          have_odometry_ = true;
          if (!have_start_) {
            start_ = now();
            have_start_ = true;
          }
        });

    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                               std::bind(&EnvironmentSimulatorNode::publish_reading, this));

    RCLCPP_INFO(get_logger(), "Environmental field ready; waiting for %s", odometry_topic.c_str());
  }

 private:
  void publish_reading() {
    const double current = now().seconds();
    if (last_clock_ >= 0 && current < last_clock_) {
      // A restarted simulation begins a new acquisition epoch.
      wind_seen_ = have_odometry_ = have_start_ = false;
      wind_stamp_ = -1;
    }
    last_clock_ = current;
    if (require_wind_ &&
        (!wind_seen_ || now().seconds() - wind_stamp_ > 2 || now().seconds() < wind_stamp_ - .1 ||
         std::chrono::duration<double>(std::chrono::steady_clock::now() - wind_receipt_).count() >
             2)) {
      return;
    }
    if (!scene_ready_) {
      return;
    }
    if (!have_odometry_ ||
        (now() - rclcpp::Time(latest_odometry_.header.stamp)).seconds() > odometry_timeout_s_ ||
        now() < rclcpp::Time(latest_odometry_.header.stamp)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for fresh robot odometry");
      return;
    }

    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - last_odometry_receipt_)
            .count() > odometry_timeout_s_) {
      return;
    }
    const auto& position = latest_odometry_.pose.pose.position;
    const bool dynamic_source =
        scenario_ == "fire" || scenario_ == "dropout" || scenario_ == "no_fire";
    auto sample = dynamic_source && fire_state_ != 1
                      ? background_model_->sample(position.x, position.y)
                      : plume_model_->sample(position.x, position.y);
    const double attenuation =
        std::exp(-std::max(0.0, position.z) / std::max(0.1, altitude_decay_m_));
    sample.smoke_ppm = baseline_ + (sample.smoke_ppm - baseline_) * attenuation;
    sample.temperature_c = ambient_ + (sample.temperature_c - ambient_) * attenuation;
    const double elapsed = (now() - start_).seconds();
    embertrace_interfaces::msg::TrialTruth truth;
    truth.header.stamp = now();
    truth.scenario = scenario_;
    // This evaluation-only topic is never subscribed to by decision nodes.
    truth.fire_present =
        dynamic_source && fire_state_ == 1 &&
        std::hypot(position.x - source_x_, position.y - source_y_) <= evaluation_radius_;
    truth.sensor_dropout = scenario_ == "dropout" && elapsed >= dropout_start_s_ &&
                           elapsed < dropout_start_s_ + dropout_duration_s_;
    truth_publisher_->publish(truth);
    if (truth.sensor_dropout) {
      return;
    }

    embertrace_interfaces::msg::EnvironmentalReading message;
    message.header.stamp = truth.header.stamp;
    message.header.frame_id = odom_frame_;
    message.sample_pose.header = message.header;
    message.sample_pose.pose = latest_odometry_.pose.pose;
    const auto gas = dynamic_source && fire_state_ != 1
                         ? co2_background_->sample(position.x, position.y)
                         : co2_model_->sample(position.x, position.y);
    const double height_factor =
        std::exp(-std::max(0.0, position.z) / std::max(.1, altitude_decay_m_));
    message.co2_ppm =
        static_cast<float>(co2_baseline_ + (gas.smoke_ppm - co2_baseline_) * height_factor);
    message.co2_valid = true;
    message.smoke_ppm = static_cast<float>(sample.smoke_ppm);
    message.temperature_c = static_cast<float>(sample.temperature_c);
    message.wind_speed_mps = static_cast<float>(sample.wind_speed_mps);
    message.wind_direction_rad = static_cast<float>(sample.wind_direction_rad);
    message.smoke_detected = false;
    message.temperature_anomaly = false;
    message.visual_confidence = 0.0F;
    message.fused_confidence = 0.0F;
    reading_publisher_->publish(message);
  }

  std::string odom_frame_, scenario_;
  double odometry_timeout_s_{1.0}, altitude_decay_m_{8.0};
  double ambient_{22.0}, baseline_{1.0}, dropout_start_s_{25.0}, dropout_duration_s_{8.0};
  bool have_start_{false};
  rclcpp::Time start_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<embertrace_interfaces::msg::TrialTruth>::SharedPtr truth_publisher_;
  bool have_odometry_{false};
  nav_msgs::msg::Odometry latest_odometry_;
  std::unique_ptr<PlumeModel> plume_model_, co2_model_, background_model_, co2_background_;
  std::uint8_t fire_state_{1};
  bool scene_ready_{true}, require_wind_{true}, wind_seen_{false};
  double wind_stamp_{-1};
  double last_clock_{-1};
  std::chrono::steady_clock::time_point wind_receipt_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr wind_subscription_;
  rclcpp::Subscription<embertrace_interfaces::msg::FireState>::SharedPtr fire_subscription_;
  double co2_baseline_{420}, source_x_{12}, source_y_{7}, evaluation_radius_{6};
  std::chrono::steady_clock::time_point last_odometry_receipt_;
  rclcpp::Publisher<embertrace_interfaces::msg::EnvironmentalReading>::SharedPtr reading_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace embertrace_core

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::EnvironmentSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
