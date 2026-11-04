#include "embertrace_core/mission_logic.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "embertrace_core/perception_filter.hpp"

namespace embertrace_core {
namespace {
bool finite_positive(double value) {
  return std::isfinite(value) && value > 0;
}
}  // namespace
const char* platform_name(Platform p) {
  switch (p) {
    case Platform::Rover:
      return "rover";
    case Platform::Drone:
      return "drone";
    default:
      return "none";
  }
}
const char* phase_name(MissionPhase p) {
  switch (p) {
    case MissionPhase::Idle:
      return "IDLE";
    case MissionPhase::Patrol:
      return "PATROL";
    case MissionPhase::Investigate:
      return "INVESTIGATE";
    case MissionPhase::Track:
      return "TRACK";
    case MissionPhase::Recover:
      return "RECOVER";
    case MissionPhase::Verify:
      return "VERIFY";
    case MissionPhase::Alert:
      return "ALERT";
    case MissionPhase::Return:
      return "RETURN_TO_BASE";
    case MissionPhase::Paused:
      return "PAUSED";
    default:
      return "EMERGENCY_STOP";
  }
}
MissionLogic::MissionLogic(MissionConfig c) : config_(c) {
  for (double v : {c.drone_wind_limit, c.rover_wind_limit, c.wind_dwell_s, c.sensor_timeout_s,
                   c.wind_timeout_s, c.search_timeout_s, c.recovery_timeout_s, c.verify_timeout_s,
                   c.return_timeout_s, c.alert_hold_s, c.verify_retry_s}) {
    if (!finite_positive(v)) {
      throw std::invalid_argument("Mission timing/limits must be positive");
    }
  }
  for (double v :
       {c.investigate_confidence, c.track_confidence, c.verify_confidence, c.alert_confidence,
        c.lost_confidence, c.visual_threshold, c.low_battery_fraction}) {
    if (!std::isfinite(v) || v < 0 || v > 1) {
      throw std::invalid_argument("Invalid mission fraction");
    }
  }
  if (c.rover_wind_limit <= c.drone_wind_limit || !finite_positive(c.wind_alpha) ||
      c.wind_alpha > 1 || !std::isfinite(c.wind_hysteresis) || c.wind_hysteresis < 0 ||
      c.wind_hysteresis >= c.drone_wind_limit || !c.detection_samples || !c.confirmation_samples ||
      !c.patrol_loops ||
      !(c.lost_confidence < c.investigate_confidence &&
        c.investigate_confidence <= c.track_confidence &&
        c.track_confidence < c.verify_confidence && c.verify_confidence <= c.alert_confidence)) {
    throw std::invalid_argument("Invalid wind hysteresis, mission thresholds or sample counts");
  }
}
std::size_t MissionLogic::index(Platform p) {
  if (p != Platform::Rover && p != Platform::Drone) {
    throw std::invalid_argument("Invalid vehicle");
  }
  return p == Platform::Rover ? 0 : 1;
}
bool MissionLogic::clock(double now, double wall) {
  if (!std::isfinite(now) || !std::isfinite(wall) || now < 0 || wall < 0) {
    if (running() || state_.manual) {
      state_.phase = MissionPhase::Paused;
      state_.manual = false;
      state_.detail = "Clock unavailable; motion held";
    }
    wind_seen_ = false;
    clear_evidence();
    return false;
  }
  if (last_now_ >= 0 && now < last_now_) {
    const bool stopped = state_.phase == MissionPhase::EmergencyStop;
    for (auto& vehicle : vehicles_) {
      vehicle = Vehicle{};
    }
    wind_seen_ = false;
    wind_choice_ = wind_candidate_ = Platform::None;
    wind_stamp_ = -1;
    clear_evidence();
    ++state_.epoch;
    state_.active = Platform::None;
    state_.manual = false;
    transition(stopped ? MissionPhase::EmergencyStop : MissionPhase::Idle,
               stopped ? "Emergency stop remains latched after clock reset"
                       : "Clock reset; waiting for START",
               now, wall);
  }
  last_now_ = now;
  return true;
}
void MissionLogic::clear_evidence() {
  detections_ = confirmations_ = 0;
}
void MissionLogic::transition(MissionPhase phase, std::string detail, double now, double wall) {
  state_.phase = phase;
  state_.detail = std::move(detail);
  phase_since_ = now;
  phase_wall_ = wall;
  if (phase == MissionPhase::Patrol) {
    verify_retry_wall_ = -1;
    search_wall_ = -1;
  }
  if (phase == MissionPhase::Investigate && search_wall_ < 0) {
    search_wall_ = wall;
  }
  if (phase == MissionPhase::Paused || phase == MissionPhase::Idle ||
      phase == MissionPhase::EmergencyStop || phase == MissionPhase::Return) {
    clear_evidence();
    state_.manual = false;
  }
}
bool MissionLogic::wind_fresh(double now, double wall) const {
  return wind_seen_ && now >= wind_stamp_ - .1 && now - wind_stamp_ <= config_.wind_timeout_s &&
         wall >= wind_receipt_ && wall - wind_receipt_ <= config_.wind_timeout_s;
}
void MissionLogic::wind(double speed, double stamp, double now, double wall) {
  if (!clock(now, wall)) {
    return;
  }
  if (!std::isfinite(speed) || speed < 0 || speed > 100 || !std::isfinite(stamp) || stamp < 0 ||
      stamp <= wind_stamp_ || now - stamp > config_.wind_timeout_s || stamp - now > .1) {
    wind_seen_ = false;
    wind_choice_ = wind_candidate_ = Platform::None;
    return;
  }
  wind_filtered_ =
      wind_seen_ ? wind_filtered_ + config_.wind_alpha * (speed - wind_filtered_) : speed;
  wind_seen_ = true;
  wind_stamp_ = stamp;
  wind_receipt_ = wall;
  Platform desired;
  // Exceeding a safety limit stops the affected platform immediately. Filtering and
  // dwell only delay deployment or a return to a less restrictive operating mode.
  if (speed > config_.rover_wind_limit) {
    desired = Platform::None;
  } else if (!config_.drone_enabled) {
    desired = Platform::Rover;
  } else if (speed > config_.drone_wind_limit) {
    desired = Platform::Rover;
  } else if (wind_choice_ == Platform::Rover &&
             wind_filtered_ > config_.drone_wind_limit - config_.wind_hysteresis) {
    desired = Platform::Rover;
  } else {
    desired = wind_filtered_ <= config_.drone_wind_limit ? Platform::Drone : Platform::Rover;
  }
  if (desired == Platform::Rover && !config_.rover_enabled) {
    desired = Platform::None;
  }
  if (desired == Platform::None) {
    wind_choice_ = desired;
    wind_candidate_ = desired;
    candidate_since_ = wall;
  } else {
    if (wind_choice_ == Platform::Drone && speed > config_.drone_wind_limit) {
      wind_choice_ = Platform::None;
    }
    if (desired != wind_candidate_) {
      wind_candidate_ = desired;
      candidate_since_ = wall;
    }
    if (wall - candidate_since_ >= config_.wind_dwell_s) {
      wind_choice_ = desired;
    }
  }
  tick(now, wall);
}
bool MissionLogic::fresh(Platform robot, double now, double wall) const {
  if (robot == Platform::None) {
    return false;
  }
  const auto& v = vehicles_[index(robot)];
  return v.valid && v.seen && now >= v.reading.stamp - .1 &&
         now - v.reading.stamp <= config_.sensor_timeout_s && wall >= v.receipt &&
         wall - v.receipt <= config_.sensor_timeout_s &&
         (!config_.require_co2 || v.reading.co2_valid) &&
         (!config_.require_thermal || v.reading.thermal_valid);
}
bool MissionLogic::running() const {
  const auto p = state_.phase;
  return p == MissionPhase::Patrol || p == MissionPhase::Investigate || p == MissionPhase::Track ||
         p == MissionPhase::Recover || p == MissionPhase::Verify || p == MissionPhase::Return;
}
bool MissionLogic::evidence(Platform robot, const MissionEvidence& r, double now, double wall) {
  if (!clock(now, wall) || robot == Platform::None) {
    return false;
  }
  auto& v = vehicles_[index(robot)];
  if (!std::isfinite(r.stamp) || r.stamp < 0 || !std::isfinite(r.confidence) ||
      !std::isfinite(r.visual) || r.confidence < 0 || r.confidence > 1 || r.visual < 0 ||
      r.visual > 1 || now - r.stamp > config_.sensor_timeout_s || r.stamp - now > .1 ||
      (v.seen && r.stamp <= v.reading.stamp)) {
    unavailable(robot);
    return false;
  }
  if (v.seen &&
      (r.stamp - v.reading.stamp > config_.sensor_timeout_s ||
       wall - v.receipt > config_.sensor_timeout_s) &&
      robot == state_.active) {
    clear_evidence();
  }
  v.reading = r;
  v.seen = v.valid = true;
  v.receipt = wall;
  tick(now, wall);
  if (robot != state_.active || !fresh(robot, now, wall) || !running() || state_.manual ||
      state_.phase == MissionPhase::Return) {
    return true;
  }
  const bool detected = r.smoke || r.co2 || r.heat;
  detections_ = detected && r.confidence >= config_.investigate_confidence
                    ? std::min(detections_ + 1, config_.detection_samples)
                    : 0;
  const bool confirmed =
      confirmation_allowed(true, r.smoke || r.co2, r.heat, r.visual, r.confidence,
                           config_.visual_threshold, config_.alert_confidence);
  confirmations_ = confirmed ? std::min(confirmations_ + 1, config_.confirmation_samples) : 0;
  switch (state_.phase) {
    case MissionPhase::Patrol:
      if (detections_ >= config_.detection_samples) {
        ++state_.warning_sequence;
        state_.acknowledged = false;
        transition(MissionPhase::Investigate, "Persistent local environmental anomaly", now, wall);
      }
      break;
    case MissionPhase::Investigate:
      if (r.confidence >= config_.track_confidence) {
        transition(MissionPhase::Track, "Tracking local environmental evidence", now, wall);
      }
      break;
    case MissionPhase::Track:
      if (!detected || r.confidence < config_.lost_confidence) {
        transition(MissionPhase::Recover, "Signal lost; bounded crosswind recovery", now, wall);
      } else if (wall >= verify_retry_wall_ && r.confidence >= config_.verify_confidence &&
                 (r.heat || r.visual >= config_.visual_threshold)) {
        confirmations_ = 0;
        transition(MissionPhase::Verify, "Holding position for corroboration", now, wall);
      }
      break;
    case MissionPhase::Recover:
      if (detected && r.confidence >= config_.track_confidence) {
        transition(MissionPhase::Track, "Local signal reacquired", now, wall);
      }
      break;
    case MissionPhase::Verify:
      if (confirmations_ >= config_.confirmation_samples) {
        ++state_.alert_sequence;
        state_.acknowledged = false;
        transition(MissionPhase::Alert, "Persistent multi-sensor fire evidence", now, wall);
      } else if (!detected || r.confidence < config_.track_confidence) {
        transition(MissionPhase::Track, "Verification evidence weakened", now, wall);
      }
      break;
    default:
      break;
  }
  return true;
}
void MissionLogic::unavailable(Platform robot) {
  if (robot == Platform::None) {
    return;
  }
  vehicles_[index(robot)].valid = false;
  if (robot == state_.active) {
    clear_evidence();
  }
}
void MissionLogic::battery(Platform robot, double fraction, double now, double wall) {
  if (!clock(now, wall) || robot == Platform::None || !std::isfinite(fraction) || fraction < 0 ||
      fraction > 1) {
    return;
  }
  auto& v = vehicles_[index(robot)];
  v.battery = fraction;
  v.battery_receipt = wall;
  if (robot == state_.active && fraction <= config_.low_battery_fraction &&
      (running() || state_.manual) && state_.phase != MissionPhase::Return) {
    transition(MissionPhase::Return, "Low battery; returning to launch position", now, wall);
  }
}
void MissionLogic::loops(Platform robot, std::size_t count, double now, double wall) {
  if (!clock(now, wall) || robot == Platform::None) {
    return;
  }
  auto& v = vehicles_[index(robot)];
  if (count < v.loops) {
    v.loop_start = count;
  }
  v.loops = count;
  if (state_.active == robot && state_.phase == MissionPhase::Patrol && count >= v.loop_start &&
      count - v.loop_start >= config_.patrol_loops) {
    transition(MissionPhase::Return, "Patrol coverage complete; returning to launch position", now,
               wall);
  }
}
void MissionLogic::arrived(Platform robot, double now, double wall) {
  if (clock(now, wall) && robot == state_.active && state_.phase == MissionPhase::Return) {
    transition(MissionPhase::Idle, "Return complete; mission ended", now, wall);
    state_.active = Platform::None;
  }
}
void MissionLogic::navigation_failed(Platform robot, double now, double wall) {
  if (clock(now, wall) && robot == state_.active && running()) {
    resume_phase_ = state_.phase;
    transition(MissionPhase::Paused, "Navigation failed; review path before RESUME", now, wall);
  }
}
bool MissionLogic::command(std::string text, double now, double wall, std::string& response) {
  if (!clock(now, wall)) {
    response = "Invalid clock";
    return false;
  }
  tick(now, wall);
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char c) { return std::toupper(c); });
  if (text == "E_STOP_RELEASE") text = "RESET";
  const auto reject = [&](const char* why) {
    response = why;
    return false;
  };
  if (state_.phase == MissionPhase::EmergencyStop && text != "RESET" && text != "E_STOP") {
    return reject("Emergency stop is latched; RESET is required");
  }
  if (text == "E_STOP") {
    transition(MissionPhase::EmergencyStop, "Emergency stop latched", now, wall);
  } else if (text == "RESET") {
    ++state_.epoch;
    state_.active = Platform::None;
    state_.acknowledged = false;
    transition(MissionPhase::Idle, "Reset complete; waiting for START", now, wall);
  } else if (text == "START" || text == "RESUME") {
    if (text == "RESUME" && state_.phase != MissionPhase::Paused) {
      return reject("Mission is not paused");
    }
    if (text == "START" && running()) {
      return reject("Pause or reset the active mission first");
    }
    if (!wind_fresh(now, wall) || wind_choice_ == Platform::None) {
      return reject("No platform has stable safe wind conditions");
    }
    if (!fresh(wind_choice_, now, wall)) {
      return reject("Selected platform sensors are unavailable");
    }
    const auto& v = vehicles_[index(wind_choice_)];
    if (wall - v.battery_receipt < config_.sensor_timeout_s &&
        v.battery <= config_.low_battery_fraction) {
      return reject("Selected platform battery is too low for deployment");
    }
    const bool changed = state_.active != wind_choice_;
    state_.active = wind_choice_;
    ++state_.epoch;
    clear_evidence();
    state_.manual = false;
    if (text == "START") {
      vehicles_[index(state_.active)].loop_start = v.loops;
    }
    const auto next = text == "RESUME" && !changed ? resume_phase_ : MissionPhase::Patrol;
    transition(next, text == "START" ? "Selected platform patrol started" : "Mission resumed", now,
               wall);
  } else if (text == "PAUSE" && running()) {
    resume_phase_ = state_.phase;
    transition(MissionPhase::Paused, "Paused by operator", now, wall);
  } else if (text == "RETURN" && state_.active != Platform::None) {
    if (!wind_fresh(now, wall) || wind_choice_ != state_.active ||
        !fresh(state_.active, now, wall)) {
      return reject("Return requires safe conditions and fresh selected-platform sensors");
    }
    transition(MissionPhase::Return, "Return requested by operator", now, wall);
  } else if (text == "ACK" && (state_.alert_sequence > 0 || state_.warning_sequence > 0)) {
    state_.acknowledged = true;
  } else if (text == "MANUAL_ON" && state_.active != Platform::None &&
             state_.phase != MissionPhase::Idle && state_.phase != MissionPhase::Return) {
    if (!wind_fresh(now, wall) || wind_choice_ != state_.active ||
        !fresh(state_.active, now, wall)) {
      return reject("Manual control requires safe wind and fresh sensors");
    }
    resume_phase_ = MissionPhase::Patrol;
    transition(MissionPhase::Paused, "Manual control enabled; release controls to stop", now, wall);
    state_.manual = true;
  } else if (text == "MANUAL_OFF" && state_.manual) {
    state_.manual = false;
    state_.detail = "Manual control disabled; RESUME to patrol";
  } else {
    return reject("Command is invalid in the current state");
  }
  response = "Accepted: " + text;
  return true;
}
void MissionLogic::tick(double now, double wall) {
  if (!clock(now, wall)) {
    return;
  }
  state_.preferred = wind_fresh(now, wall) ? wind_choice_ : Platform::None;
  if ((running() || state_.manual) && (!wind_fresh(now, wall) || wind_choice_ != state_.active ||
                                       !fresh(state_.active, now, wall))) {
    resume_phase_ =
        state_.phase == MissionPhase::Return ? MissionPhase::Return : MissionPhase::Patrol;
    transition(MissionPhase::Paused,
               "Wind/platform or sensor availability changed; review then RESUME", now, wall);
    return;
  }
  if (state_.phase == MissionPhase::Alert && wall - phase_wall_ >= config_.alert_hold_s) {
    if (wind_fresh(now, wall) && wind_choice_ == state_.active && fresh(state_.active, now, wall)) {
      transition(MissionPhase::Return, "Alert recorded; automatically returning to launch position",
                 now, wall);
    } else {
      resume_phase_ = MissionPhase::Return;
      transition(MissionPhase::Paused, "Alert recorded; waiting for safe return", now, wall);
    }
  } else if (running() && state_.phase != MissionPhase::Patrol &&
             state_.phase != MissionPhase::Return && search_wall_ >= 0 &&
             wall - search_wall_ > config_.search_timeout_s) {
    clear_evidence();
    transition(MissionPhase::Patrol, "Investigation timed out; resuming coverage", now, wall);
  } else if (state_.phase == MissionPhase::Recover &&
             wall - phase_wall_ > config_.recovery_timeout_s) {
    clear_evidence();
    transition(MissionPhase::Patrol, "No signal after recovery; resuming coverage", now, wall);
  } else if (state_.phase == MissionPhase::Verify &&
             wall - phase_wall_ > config_.verify_timeout_s) {
    clear_evidence();
    verify_retry_wall_ = wall + config_.verify_retry_s;
    transition(MissionPhase::Track, "Verification timed out; gathering new local evidence", now,
               wall);
  } else if (state_.phase == MissionPhase::Return &&
             wall - phase_wall_ > config_.return_timeout_s) {
    resume_phase_ = MissionPhase::Return;
    transition(MissionPhase::Paused, "Return timed out; operator review required", now, wall);
  }
}
MissionSnapshot MissionLogic::snapshot() const {
  auto result = state_;
  const auto p = state_.phase;
  result.motion_allowed = p == MissionPhase::Patrol || p == MissionPhase::Investigate ||
                          p == MissionPhase::Track || p == MissionPhase::Recover ||
                          p == MissionPhase::Return;
  return result;
}
}  // namespace embertrace_core
