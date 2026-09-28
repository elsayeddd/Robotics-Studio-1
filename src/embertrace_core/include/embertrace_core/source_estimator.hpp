#ifndef EMBERTRACE_CORE__SOURCE_ESTIMATOR_HPP_
#define EMBERTRACE_CORE__SOURCE_ESTIMATOR_HPP_

#include <cstddef>
#include <deque>
#include <optional>

namespace embertrace_core
{

struct SearchSample
{
  double x{0.0};
  double y{0.0};
  double smoke_ppm{0.0};
  double temperature_c{0.0};
  double wind_direction_rad{0.0};
};

struct SourceEstimate
{
  double x{0.0};
  double y{0.0};
  // Marginal position covariance in square metres. xy is the off-diagonal term.
  double variance_x{0.0};
  double variance_y{0.0};
  double covariance_xy{0.0};
  std::size_t sample_count{0};
};

struct SourceEstimatorConfig
{
  std::size_t max_samples{160};
  double minimum_spacing_m{0.3};
  double ambient_temperature_c{22.0};
  double baseline_smoke_ppm{1.0};
  double thermal_decay_m{7.0};
  double plume_length_m{14.0};
  double plume_width_m{1.6};
  double plume_expansion{0.05};
  double min_x{-18.5};
  double max_x{18.5};
  double min_y{-18.5};
  double max_y{18.5};
};

// Estimates a source from local measurements only; it has no simulator or map input.
class SourceEstimator
{
public:
  explicit SourceEstimator(SourceEstimatorConfig config = {});
  void clear();
  bool add(const SearchSample & sample);
  std::optional<SourceEstimate> estimate() const;
  std::size_t sample_count() const {return samples_.size();}

private:
  double score(double x, double y) const;

  SourceEstimatorConfig config_;
  std::deque<SearchSample> samples_;
};

}  // namespace embertrace_core

#endif  // EMBERTRACE_CORE__SOURCE_ESTIMATOR_HPP_
