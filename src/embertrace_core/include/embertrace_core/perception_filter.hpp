#pragma once

#include <cstddef>
#include <limits>
#include <optional>

namespace embertrace_core
{
/** Provisional demonstrator parameters. Calibrate on training recordings only. */
struct PerceptionConfig
{
  double alpha{0.4};
  double nominal_period_s{0.2};
  double smoke_enter{12.0}, smoke_exit{8.0}, smoke_scale{45.0}, baseline_smoke{1.0};
  double heat_enter{32.0}, heat_exit{29.0}, heat_scale{65.0};
  double co2_enter{700.0}, co2_exit{600.0}, co2_scale{1500.0}, baseline_co2{420.0};
  double visual_weight{0.15}, visual_threshold{0.55}, confirmation_threshold{0.74};
  double environment_timeout_s{2.0}, visual_timeout_s{1.0}, future_tolerance_s{0.1};
  double thermal_timeout_s{1.0};
  std::size_t warmup_samples{3}, detection_samples{3};
  // Legacy smoke-only recordings remain replayable. Full launch explicitly enables both.
  bool require_co2{false}, require_thermal{false};
};

struct PerceptionSample
{
  double stamp_s{}, smoke{}, temperature{}, wind_speed{}, wind_direction{};
  double co2{420.0};
  bool co2_valid{false};
};

struct PerceptionResult
{
  bool valid{false}, smoke_detected{false}, temperature_anomaly{false};
  bool visual_valid{false}, confirmed_candidate{false};
  double smoke{}, temperature{}, visual{}, confidence{};
  bool co2_valid{false}, co2_detected{false}, thermal_valid{false}, thermal_hotspot{false};
  double co2{}, thermal_peak_c{};
};

/** Shared live/replay algorithm. Never receives source coordinates or trial labels.
 * ROS time checks acquisition age; optional monotonic receipt time detects /clock stalls.
 * Missing required data is unavailable evidence, never a fabricated clean reading.
 */
class PerceptionFilter
{
public:
  explicit PerceptionFilter(PerceptionConfig config = {});
  void reset();  // Explicit full reset; used to begin an independent trial.
  void invalidate_environment();  // Preserve timestamp history to reject replays.
  void tick(double now_s, double steady_s = std::numeric_limits<double>::quiet_NaN());
  bool set_visual(double stamp_s, double confidence, bool valid, double now_s,
    double steady_s = std::numeric_limits<double>::quiet_NaN());
  bool set_thermal(double stamp_s, double peak_c, bool hotspot, bool valid, double now_s,
    double steady_s = std::numeric_limits<double>::quiet_NaN());
  std::optional<PerceptionResult> update(const PerceptionSample & sample, double now_s,
    double steady_s = std::numeric_limits<double>::quiet_NaN());
  bool environment_fresh(double now_s) const;
  bool visual_fresh(double now_s) const;
  bool thermal_fresh(double now_s) const;
  bool confirmation_candidate(double now_s) const;
  std::size_t rejected_samples() const {return rejected_;}
  std::size_t consecutive_samples() const {return consecutive_;}
  const PerceptionConfig & config() const {return config_;}
private:
  bool check_clock(double now_s);
  void latch(double value, double enter, double exit, bool & latched, std::size_t & count);
  bool fresh(double now, double stamp, double timeout) const;
  PerceptionConfig config_;
  bool initialised_{false}, smoke_latched_{false}, heat_latched_{false}, co2_latched_{false};
  bool have_stamp_{false}, have_visual_{false}, have_thermal_{false}, thermal_hotspot_{false};
  double smoke_{}, temperature_{}, co2_{}, last_stamp_{-1.0}, last_clock_{-1.0};
  double visual_stamp_{-1.0}, visual_{0.0}, thermal_stamp_{-1.0}, thermal_peak_{0.0};
  double steady_{-1.0}, receipt_{-1.0}, visual_receipt_{-1.0}, thermal_receipt_{-1.0};
  bool sample_co2_valid_{false};
  std::size_t rejected_{0}, consecutive_{0}, smoke_count_{0}, heat_count_{0}, co2_count_{0};
  PerceptionResult last_result_;
};

bool confirmation_allowed(bool fresh, bool gas, bool heat, double visual,
  double confidence, double visual_threshold = 0.55, double confidence_threshold = 0.74);
}  // namespace embertrace_core
