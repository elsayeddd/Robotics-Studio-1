#include "embertrace_core/perception_filter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace embertrace_core {
namespace {
double scale(double value, double low, double high) {
  return std::clamp((value - low) / (high - low), 0.0, 1.0);
}
}  // namespace
PerceptionFilter::PerceptionFilter(PerceptionConfig c) : config_(c) {
  for (double v : {c.alpha,
                   c.nominal_period_s,
                   c.smoke_enter,
                   c.smoke_exit,
                   c.smoke_scale,
                   c.baseline_smoke,
                   c.heat_enter,
                   c.heat_exit,
                   c.heat_scale,
                   c.co2_enter,
                   c.co2_exit,
                   c.co2_scale,
                   c.baseline_co2,
                   c.visual_weight,
                   c.visual_threshold,
                   c.confirmation_threshold,
                   c.environment_timeout_s,
                   c.visual_timeout_s,
                   c.thermal_timeout_s,
                   c.future_tolerance_s}) {
    if (!std::isfinite(v)) {
      throw std::invalid_argument("Non-finite perception parameter");
    }
  }
  if (c.alpha <= 0 || c.alpha > 1 || c.nominal_period_s <= 0 || c.baseline_smoke < 0 ||
      c.smoke_exit < c.baseline_smoke || c.smoke_exit >= c.smoke_enter ||
      c.smoke_scale <= c.smoke_enter || c.baseline_co2 < 0 || c.co2_exit < c.baseline_co2 ||
      c.co2_enter <= c.co2_exit || c.co2_scale <= c.co2_enter || c.heat_exit >= c.heat_enter ||
      c.heat_scale <= c.heat_enter || c.visual_weight < 0 || c.visual_weight > 0.5 ||
      c.visual_threshold < 0 || c.visual_threshold > 1 || c.confirmation_threshold < 0 ||
      c.confirmation_threshold > 1 || c.environment_timeout_s <= 0 || c.visual_timeout_s <= 0 ||
      c.thermal_timeout_s <= 0 || c.future_tolerance_s < 0 || !c.warmup_samples ||
      !c.detection_samples) {
    throw std::invalid_argument("Invalid perception thresholds, timing or persistence");
  }
}
void PerceptionFilter::invalidate_environment() {
  initialised_ = smoke_latched_ = heat_latched_ = co2_latched_ = false;
  consecutive_ = smoke_count_ = heat_count_ = co2_count_ = 0;
  sample_co2_valid_ = false;
  last_result_ = {};
}
void PerceptionFilter::reset() {
  visual_history_.clear();
  thermal_history_.clear();
  invalidate_environment();
  have_stamp_ = have_visual_ = have_thermal_ = false;
  last_stamp_ = visual_stamp_ = thermal_stamp_ = last_clock_ = -1;
  receipt_ = visual_receipt_ = thermal_receipt_ = steady_ = -1;
}
bool PerceptionFilter::check_clock(double now) {
  if (!std::isfinite(now) || now < 0) {
    return false;
  }
  if (last_clock_ >= 0 && now < last_clock_) {
    reset();
  }
  last_clock_ = now;
  return true;
}
void PerceptionFilter::tick(double now, double wall) {
  if (!check_clock(now)) {
    invalidate_environment();
    return;
  }
  if (std::isfinite(wall)) {
    // Start receipt clocks if a replay/test begins supplying monotonic time later.
    if (steady_ < 0) {
      receipt_ = visual_receipt_ = thermal_receipt_ = wall;
    }
    steady_ = wall;
  }
  if (initialised_ &&
      (!fresh(now, last_stamp_, config_.environment_timeout_s) ||
       (steady_ >= 0 && receipt_ >= 0 && steady_ - receipt_ > config_.environment_timeout_s))) {
    invalidate_environment();
  }
  if (have_visual_ && (!fresh(now, visual_stamp_, config_.visual_timeout_s) ||
                       (steady_ >= 0 && visual_receipt_ >= 0 &&
                        steady_ - visual_receipt_ > config_.visual_timeout_s))) {
    have_visual_ = false;
    visual_history_.clear();
  }
  if (have_thermal_ && (!fresh(now, thermal_stamp_, config_.thermal_timeout_s) ||
                        (steady_ >= 0 && thermal_receipt_ >= 0 &&
                         steady_ - thermal_receipt_ > config_.thermal_timeout_s))) {
    have_thermal_ = false;
    thermal_history_.clear();
  }
}
bool PerceptionFilter::fresh(double now, double stamp, double timeout) const {
  const double age = now - stamp;
  return stamp >= 0 && std::isfinite(age) && age >= -config_.future_tolerance_s && age <= timeout;
}
bool PerceptionFilter::visual_fresh(double now) const {
  return have_visual_ && fresh(now, visual_stamp_, config_.visual_timeout_s);
}
bool PerceptionFilter::thermal_fresh(double now) const {
  return have_thermal_ && fresh(now, thermal_stamp_, config_.thermal_timeout_s);
}
bool PerceptionFilter::environment_fresh(double now) const {
  return initialised_ && consecutive_ >= config_.warmup_samples &&
         fresh(now, last_stamp_, config_.environment_timeout_s);
}
bool PerceptionFilter::set_visual(double stamp, double confidence, bool valid, double now,
                                  double wall) {
  tick(now, wall);
  if (!valid || !fresh(now, stamp, config_.visual_timeout_s) || !std::isfinite(confidence) ||
      confidence < 0 || confidence > 1 || stamp <= visual_stamp_) {
    have_visual_ = false;
    visual_history_.clear();
    return false;
  }
  visual_stamp_ = stamp;
  visual_receipt_ = steady_;
  visual_history_.push_back({stamp, steady_, confidence, false});
  if (visual_history_.size() > 32) visual_history_.pop_front();
  have_visual_ = true;
  return true;
}
bool PerceptionFilter::set_thermal(double stamp, double peak, bool hotspot, bool valid, double now,
                                   double wall) {
  tick(now, wall);
  if (!valid || !fresh(now, stamp, config_.thermal_timeout_s) || !std::isfinite(peak) ||
      peak < -80 || peak > 1000 || stamp <= thermal_stamp_) {
    have_thermal_ = false;
    thermal_history_.clear();
    return false;
  }
  thermal_stamp_ = stamp;
  thermal_receipt_ = steady_;
  thermal_history_.push_back({stamp, steady_, peak, hotspot});
  if (thermal_history_.size() > 32) thermal_history_.pop_front();
  have_thermal_ = true;
  return true;
}
const PerceptionFilter::Cue* PerceptionFilter::aligned(
    const std::deque<Cue>& history, double sample, double now, double timeout) const {
  // Independent ROS subscriptions can deliver the next image before an older gas
  // sample. Keep bounded causal history instead of making that sample unavailable.
  for (auto it = history.rbegin(); it != history.rend(); ++it) {
    if (it->stamp <= sample + config_.future_tolerance_s && fresh(now, it->stamp, timeout) &&
        (steady_ < 0 || it->receipt < 0 || steady_ - it->receipt <= timeout)) return &*it;
  }
  return nullptr;
}
void PerceptionFilter::latch(double value, double enter, double exit, bool& latched,
                             std::size_t& n) {
  if (latched) {
    if (value <= exit) {
      latched = false;
      n = 0;
    }
  } else {
    n = value >= enter ? std::min(n + 1, config_.detection_samples) : 0;
    latched = n >= config_.detection_samples;
  }
}
std::optional<PerceptionResult> PerceptionFilter::update(const PerceptionSample& s, double now,
                                                         double wall) {
  tick(now, wall);
  const auto reject = [this]() -> std::optional<PerceptionResult> {
    ++rejected_;
    invalidate_environment();
    return std::nullopt;
  };
  if (!std::isfinite(now) || now < 0) {
    return reject();
  }
  for (double v : {s.stamp_s, s.smoke, s.temperature, s.wind_speed, s.wind_direction}) {
    if (!std::isfinite(v)) {
      return reject();
    }
  }
  if (s.smoke < 0 || s.smoke > 10000 || s.temperature < -80 || s.temperature > 1000 ||
      s.wind_speed < 0 || s.wind_speed > 100 ||
      !fresh(now, s.stamp_s, config_.environment_timeout_s) ||
      (have_stamp_ && s.stamp_s <= last_stamp_) || (config_.require_co2 && !s.co2_valid) ||
      (s.co2_valid && (!std::isfinite(s.co2) || s.co2 < 0 || s.co2 > 1000000))) {
    return reject();
  }
  // Receipt gaps as well as sample-time gaps force a new warmup.
  if (initialised_ && s.stamp_s - last_stamp_ > config_.environment_timeout_s) {
    invalidate_environment();
  }
  if (!initialised_) {
    smoke_ = s.smoke;
    temperature_ = s.temperature;
    co2_ = s.co2;
  } else {
    const double a =
        1 - std::pow(1 - config_.alpha, (s.stamp_s - last_stamp_) / config_.nominal_period_s);
    smoke_ += a * (s.smoke - smoke_);
    temperature_ += a * (s.temperature - temperature_);
    if (s.co2_valid) {
      co2_ = sample_co2_valid_ ? co2_ + a * (s.co2 - co2_) : s.co2;
    }
  }
  initialised_ = have_stamp_ = true;
  sample_co2_valid_ = s.co2_valid;
  last_stamp_ = s.stamp_s;
  receipt_ = steady_;
  consecutive_ = std::min(consecutive_ + 1, config_.warmup_samples);
  latch(smoke_, config_.smoke_enter, config_.smoke_exit, smoke_latched_, smoke_count_);
  latch(temperature_, config_.heat_enter, config_.heat_exit, heat_latched_, heat_count_);
  if (s.co2_valid) {
    latch(co2_, config_.co2_enter, config_.co2_exit, co2_latched_, co2_count_);
  } else {
    co2_latched_ = false;
    co2_count_ = 0;
  }
  PerceptionResult r;
  r.valid = environment_fresh(now);
  r.smoke = smoke_;
  r.temperature = temperature_;
  r.co2_valid = s.co2_valid;
  r.co2 = s.co2_valid ? co2_ : 0;
  r.smoke_detected = r.valid && smoke_latched_;
  r.co2_detected = r.valid && co2_latched_;
  const auto* ir = aligned(thermal_history_, s.stamp_s, now, config_.thermal_timeout_s);
  const auto* visual = aligned(visual_history_, s.stamp_s, now, config_.visual_timeout_s);
  r.thermal_valid = ir != nullptr;
  r.thermal_hotspot = ir && ir->hotspot;
  r.thermal_peak_c = ir ? ir->value : 0;
  r.temperature_anomaly = r.valid && (heat_latched_ || r.thermal_hotspot);
  r.visual_valid = visual != nullptr;
  r.visual = visual ? visual->value : 0;
  const bool gas = r.smoke_detected || r.co2_detected;
  if (gas || r.temperature_anomaly) {
    // Smoke and CO2 are correlated combustion evidence; taking max avoids counting them twice.
    const double gas_score =
        std::max(scale(smoke_, config_.baseline_smoke, config_.smoke_scale),
                 s.co2_valid ? scale(co2_, config_.baseline_co2, config_.co2_scale) : 0);
    const double heat_score = std::max(
        scale(temperature_, config_.heat_enter, config_.heat_scale),
        r.thermal_hotspot ? scale(r.thermal_peak_c, config_.heat_enter, config_.heat_scale) : 0);
    const double environmental = .65 * gas_score + .35 * heat_score;
    r.confidence = environmental + config_.visual_weight * r.visual * (1 - environmental);
  }
  const bool sensors_ready = r.valid && (!config_.require_thermal || r.thermal_valid);
  r.confirmed_candidate =
      confirmation_allowed(sensors_ready, gas, r.temperature_anomaly, r.visual, r.confidence,
                           config_.visual_threshold, config_.confirmation_threshold);
  last_result_ = r;
  return r;
}
bool PerceptionFilter::confirmation_candidate(double now) const {
  const auto* ir = aligned(thermal_history_, last_stamp_, now, config_.thermal_timeout_s);
  const auto* vision = aligned(visual_history_, last_stamp_, now, config_.visual_timeout_s);
  if (!environment_fresh(now) || (config_.require_thermal && !ir)) {
    return false;
  }
  const bool heat = heat_latched_ || (ir && ir->hotspot);
  const bool gas = smoke_latched_ || co2_latched_;
  if (!heat && !gas) {
    return false;
  }
  const double gas_score =
      std::max(scale(smoke_, config_.baseline_smoke, config_.smoke_scale),
               sample_co2_valid_ ? scale(co2_, config_.baseline_co2, config_.co2_scale) : 0);
  const double heat_score = std::max(
      scale(temperature_, config_.heat_enter, config_.heat_scale),
      ir && ir->hotspot ? scale(ir->value, config_.heat_enter, config_.heat_scale) : 0);
  const double environment = .65 * gas_score + .35 * heat_score;
  const double visual = vision ? vision->value : 0;
  const double confidence = environment + config_.visual_weight * visual * (1 - environment);
  return confirmation_allowed(true, gas, heat, visual, confidence, config_.visual_threshold,
                              config_.confirmation_threshold);
}
bool confirmation_allowed(bool fresh, bool gas, bool heat, double visual, double confidence,
                          double visual_threshold, double confidence_threshold) {
  return fresh && std::isfinite(visual) && visual >= 0 && visual <= 1 &&
         std::isfinite(confidence) && confidence >= 0 && confidence <= 1 && (gas || heat) &&
         confidence >= confidence_threshold && ((gas && heat) || visual >= visual_threshold);
}
}  // namespace embertrace_core
