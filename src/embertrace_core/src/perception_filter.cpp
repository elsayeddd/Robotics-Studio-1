#include "embertrace_core/perception_filter.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace embertrace_core
{
namespace
{
double scale(double value, double low, double high)
{
  return std::clamp((value - low) / (high - low), 0.0, 1.0);
}
}

PerceptionFilter::PerceptionFilter(PerceptionConfig c) : config_(c)
{
  const double values[] = {c.alpha, c.nominal_period_s, c.smoke_enter, c.smoke_exit,
    c.smoke_scale, c.baseline_smoke, c.heat_enter, c.heat_exit, c.heat_scale,
    c.visual_weight, c.visual_threshold, c.confirmation_threshold,
    c.environment_timeout_s, c.visual_timeout_s, c.future_tolerance_s};
  for (double value : values) {
    if (!std::isfinite(value)) {throw std::invalid_argument("Non-finite perception parameter");}
  }
  if (c.alpha <= 0 || c.alpha > 1 || c.nominal_period_s <= 0 ||
    c.baseline_smoke < 0 || c.smoke_exit < c.baseline_smoke ||
    c.smoke_exit >= c.smoke_enter || c.smoke_scale <= c.smoke_enter ||
    c.heat_exit >= c.heat_enter || c.heat_scale <= c.heat_enter ||
    c.visual_weight < 0 || c.visual_weight > 0.5 || c.visual_threshold < 0 ||
    c.visual_threshold > 1 || c.confirmation_threshold < 0 || c.confirmation_threshold > 1 ||
    c.environment_timeout_s <= 0 || c.visual_timeout_s <= 0 ||
    c.future_tolerance_s < 0 || c.warmup_samples < 1)
  {
    throw std::invalid_argument("Invalid perception thresholds, timing or filter parameters");
  }
}

void PerceptionFilter::reset()
{
  initialised_ = smoke_latched_ = heat_latched_ = have_visual_ = false;
  consecutive_ = 0;
  visual_ = 0;
  visual_stamp_ = -1;
}

bool PerceptionFilter::check_clock(double now_s)
{
  if (!std::isfinite(now_s) || now_s < 0) {return false;}
  if (last_clock_ >= 0 && now_s < last_clock_) {reset();}
  last_clock_ = now_s;
  return true;
}

bool PerceptionFilter::visual_fresh(double now_s) const
{
  const double age = now_s - visual_stamp_;
  return have_visual_ && std::isfinite(age) && age >= -config_.future_tolerance_s &&
         age <= config_.visual_timeout_s;
}

bool PerceptionFilter::environment_fresh(double now_s) const
{
  const double age = now_s - last_stamp_;
  return initialised_ && consecutive_ >= config_.warmup_samples && std::isfinite(age) &&
         age >= -config_.future_tolerance_s && age <= config_.environment_timeout_s;
}

bool PerceptionFilter::set_visual(double stamp, double confidence, bool valid, double now_s)
{
  if (!check_clock(now_s)) {return false;}
  if (!valid) {have_visual_ = false; return false;}
  if (!std::isfinite(stamp) || stamp <= 0 || !std::isfinite(confidence) ||
    confidence < 0 || confidence > 1 || now_s - stamp > config_.visual_timeout_s ||
    stamp - now_s > config_.future_tolerance_s || (have_visual_ && stamp <= visual_stamp_))
  {
    return false;
  }
  visual_stamp_ = stamp;
  visual_ = confidence;
  have_visual_ = true;
  return true;
}

std::optional<PerceptionResult> PerceptionFilter::update(
  const PerceptionSample & s, double now_s)
{
  if (!check_clock(now_s)) {++rejected_; return std::nullopt;}
  const double values[] = {s.stamp_s, s.smoke, s.temperature, s.wind_speed, s.wind_direction};
  for (double v : values) {
    if (!std::isfinite(v)) {++rejected_; reset(); return std::nullopt;}
  }
  if (s.smoke < 0 || s.smoke > 10000 || s.temperature < -80 || s.temperature > 1000 ||
    s.wind_speed < 0 || s.wind_speed > 100 || s.stamp_s <= 0 ||
    now_s - s.stamp_s > config_.environment_timeout_s ||
    s.stamp_s - now_s > config_.future_tolerance_s ||
    (initialised_ && s.stamp_s <= last_stamp_))
  {
    ++rejected_;
    return std::nullopt;
  }
  if (initialised_ && s.stamp_s - last_stamp_ > config_.environment_timeout_s) {
    reset();  // Drop the old latch and require fresh evidence after a dropout.
  }
  if (!initialised_) {
    smoke_ = s.smoke;
    temperature_ = s.temperature;
    initialised_ = true;
  } else {
    const double alpha = 1.0 - std::pow(1.0 - config_.alpha,
      (s.stamp_s - last_stamp_) / config_.nominal_period_s);
    smoke_ += alpha * (s.smoke - smoke_);
    temperature_ += alpha * (s.temperature - temperature_);
  }
  last_stamp_ = s.stamp_s;
  ++consecutive_;
  if (smoke_ >= config_.smoke_enter) {smoke_latched_ = true;}
  if (smoke_ <= config_.smoke_exit) {smoke_latched_ = false;}
  if (temperature_ >= config_.heat_enter) {heat_latched_ = true;}
  if (temperature_ <= config_.heat_exit) {heat_latched_ = false;}
  PerceptionResult r;
  r.valid = environment_fresh(now_s);
  r.smoke = smoke_;
  r.temperature = temperature_;
  r.smoke_detected = r.valid && smoke_latched_;
  r.temperature_anomaly = r.valid && heat_latched_;
  r.visual_valid = visual_fresh(now_s);
  r.visual = r.visual_valid ? visual_ : 0.0;
  if (r.smoke_detected || r.temperature_anomaly) {
    const double environmental = 0.65 * scale(smoke_, config_.baseline_smoke, config_.smoke_scale) +
      0.35 * scale(temperature_, config_.heat_enter, config_.heat_scale);
    r.confidence = environmental + config_.visual_weight * r.visual * (1.0 - environmental);
  }
  r.confirmed_candidate = confirmation_allowed(r.valid, r.smoke_detected,
    r.temperature_anomaly, r.visual, r.confidence,
    config_.visual_threshold, config_.confirmation_threshold);
  return r;
}

bool confirmation_allowed(bool fresh, bool smoke, bool heat, double visual,
  double confidence, double visual_threshold, double confidence_threshold)
{
  return fresh && std::isfinite(visual) && std::isfinite(confidence) &&
         (smoke || heat) && confidence >= confidence_threshold &&
         ((smoke && heat) || visual >= visual_threshold);
}
}  // namespace embertrace_core
