#pragma once

#include <cstddef>
#include <optional>

namespace embertrace_core
{
struct PerceptionConfig
{
  double alpha{0.25};
  double nominal_period_s{0.2};
  double smoke_enter{12.0}, smoke_exit{8.0}, smoke_scale{45.0}, baseline_smoke{1.0};
  double heat_enter{32.0}, heat_exit{29.0}, heat_scale{65.0};
  double visual_weight{0.15}, visual_threshold{0.55}, confirmation_threshold{0.74};
  double environment_timeout_s{2.0}, visual_timeout_s{1.0}, future_tolerance_s{0.1};
  std::size_t warmup_samples{3};
};

struct PerceptionSample
{
  double stamp_s{}, smoke{}, temperature{}, wind_speed{}, wind_direction{};
};

struct PerceptionResult
{
  bool valid{false}, smoke_detected{false}, temperature_anomaly{false};
  bool visual_valid{false}, confirmed_candidate{false};
  double smoke{}, temperature{}, visual{}, confidence{};
};

// Shared by the live ROS node and the CSV evaluation executable.
class PerceptionFilter
{
public:
  explicit PerceptionFilter(PerceptionConfig config = {});
  void reset();
  bool set_visual(double stamp_s, double confidence, bool valid, double now_s);
  std::optional<PerceptionResult> update(const PerceptionSample & sample, double now_s);
  bool environment_fresh(double now_s) const;
  bool visual_fresh(double now_s) const;
  std::size_t rejected_samples() const {return rejected_;}
  std::size_t consecutive_samples() const {return consecutive_;}
  const PerceptionConfig & config() const {return config_;}
private:
  bool check_clock(double now_s);
  PerceptionConfig config_;
  bool initialised_{false}, smoke_latched_{false}, heat_latched_{false};
  double smoke_{}, temperature_{}, last_stamp_{}, last_clock_{-1.0};
  double visual_stamp_{-1.0}, visual_{0.0};
  bool have_visual_{false};
  std::size_t rejected_{0}, consecutive_{0};
};

bool confirmation_allowed(bool fresh, bool smoke, bool heat, double visual,
  double confidence, double visual_threshold = 0.55, double confidence_threshold = 0.74);
}  // namespace embertrace_core
