#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace embertrace_core {
/** Values match the public MissionState message. */
enum class Platform : std::uint8_t { None = 0, Rover = 1, Drone = 2 };
enum class MissionPhase : std::uint8_t {
  Idle,
  Patrol,
  Investigate,
  Track,
  Recover,
  Verify,
  Alert,
  Return,
  Paused,
  EmergencyStop
};

struct MissionConfig {
  bool rover_enabled{true}, drone_enabled{true}, require_co2{true}, require_thermal{true};
  double drone_wind_limit{5}, rover_wind_limit{12}, wind_hysteresis{.5};
  double wind_alpha{.35}, wind_dwell_s{2}, sensor_timeout_s{2}, wind_timeout_s{2};
  double investigate_confidence{.22}, track_confidence{.38}, verify_confidence{.68};
  double alert_confidence{.74}, lost_confidence{.12}, visual_threshold{.55};
  double search_timeout_s{120}, recovery_timeout_s{20}, verify_timeout_s{15}, verify_retry_s{5};
  double return_timeout_s{120}, alert_hold_s{10}, low_battery_fraction{.2};
  std::size_t detection_samples{3}, confirmation_samples{3}, patrol_loops{2};
};

/** Local evidence for one vehicle. Source position/trial truth never enters decisions. */
struct MissionEvidence {
  double stamp{}, confidence{}, visual{};
  bool smoke{}, heat{}, co2{}, co2_valid{}, thermal_valid{};
};

struct MissionSnapshot {
  MissionPhase phase{MissionPhase::Idle};
  Platform active{Platform::None}, preferred{Platform::None};
  bool motion_allowed{}, manual{}, acknowledged{};
  std::uint64_t epoch{}, alert_sequence{}, warning_sequence{};
  std::string detail{"Waiting for fresh wind and sensors"};
};

/** Pure mission policy used by ROS and regression tests.
 * Acquisition time and elapsed receipt time both matter: repeated packets or a frozen
 * simulator clock cannot maintain movement permission or confirmation indefinitely.
 */
class MissionLogic {
 public:
  explicit MissionLogic(MissionConfig config = MissionConfig{});
  /// Update ambient wind; unsafe excursions stop immediately, deployment waits for dwell.
  void wind(double speed, double stamp, double now, double wall);
  /// Store one ordered local sample; only the selected vehicle advances confirmation.
  bool evidence(Platform robot, const MissionEvidence& reading, double now, double wall);
  /// Mark a required channel unavailable and discard its pending confirmation count.
  void unavailable(Platform robot);
  void battery(Platform robot, double fraction, double now, double wall);
  void loops(Platform robot, std::size_t count, double now, double wall);
  void arrived(Platform robot, double now, double wall);
  void navigation_failed(Platform robot, double now, double wall);
  /// Apply an operator request and explain a rejected state or readiness precondition.
  bool command(std::string text, double now, double wall, std::string& response);
  /// Enforce sensor/receipt expiry and bounded phases even when no samples arrive.
  void tick(double now, double wall);
  MissionSnapshot snapshot() const;
  const MissionConfig& config() const {
    return config_;
  }
  bool fresh(Platform robot, double now, double wall) const;

 private:
  struct Vehicle {
    MissionEvidence reading;
    bool seen{}, valid{};
    double receipt{-1}, battery{1}, battery_receipt{-1};
    std::size_t loops{}, loop_start{};
  };
  static std::size_t index(Platform robot);
  bool clock(double now, double wall);
  bool wind_fresh(double now, double wall) const;
  bool running() const;
  void transition(MissionPhase phase, std::string detail, double now, double wall);
  void clear_evidence();
  MissionConfig config_;
  MissionSnapshot state_;
  std::array<Vehicle, 2> vehicles_;
  MissionPhase resume_phase_{MissionPhase::Patrol};
  Platform wind_choice_{Platform::None}, wind_candidate_{Platform::None};
  bool wind_seen_{false};
  double wind_stamp_{-1}, wind_receipt_{-1}, wind_filtered_{0}, candidate_since_{-1};
  double verify_retry_wall_{-1};
  double last_now_{-1}, phase_since_{0}, phase_wall_{0}, search_wall_{-1};
  std::size_t detections_{0}, confirmations_{0};
};
const char* phase_name(MissionPhase phase);
const char* platform_name(Platform platform);
}  // namespace embertrace_core
