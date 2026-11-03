#include "embertrace_core/source_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace embertrace_core {

SourceEstimator::SourceEstimator(SourceEstimatorConfig config) : config_(std::move(config)) {
  for (double value :
       {config_.minimum_spacing_m, config_.ambient_temperature_c, config_.baseline_smoke_ppm,
        config_.thermal_decay_m, config_.plume_length_m, config_.plume_width_m,
        config_.plume_expansion, config_.min_x, config_.max_x, config_.min_y, config_.max_y}) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("Source estimator parameters must be finite");
    }
  }
  if (config_.max_samples < 5 || config_.max_samples > 10000 || config_.minimum_spacing_m < 0 ||
      config_.thermal_decay_m <= 0 || config_.plume_length_m <= 0 || config_.plume_width_m <= 0 ||
      config_.plume_expansion < 0 || config_.baseline_smoke_ppm < 0 ||
      config_.min_x >= config_.max_x || config_.min_y >= config_.max_y) {
    throw std::invalid_argument("Invalid source estimator geometry/history parameters");
  }
}

void SourceEstimator::clear() {
  samples_.clear();
}

bool SourceEstimator::add(const SearchSample& sample) {
  if (!std::isfinite(sample.x) || !std::isfinite(sample.y) || !std::isfinite(sample.smoke_ppm) ||
      !std::isfinite(sample.temperature_c) || !std::isfinite(sample.wind_direction_rad) ||
      sample.smoke_ppm < 0 || sample.temperature_c < -80 || sample.temperature_c > 1000 ||
      sample.x < config_.min_x || sample.x > config_.max_x || sample.y < config_.min_y ||
      sample.y > config_.max_y) {
    return false;
  }
  if (!samples_.empty() && std::hypot(sample.x - samples_.back().x, sample.y - samples_.back().y) <
                               config_.minimum_spacing_m) {
    // Replace a stationary reading so sensor noise cannot overwhelm spatial evidence.
    samples_.back() = sample;
    return true;
  }
  samples_.push_back(sample);
  if (samples_.size() > config_.max_samples) {
    samples_.pop_front();
  }
  return true;
}

double SourceEstimator::score(const double x, const double y) const {
  double thermal_sum = 0.0;
  double thermal_square_sum = 0.0;
  double smoke_sum = 0.0;
  double smoke_square_sum = 0.0;
  int thermal_count = 0;
  int smoke_count = 0;

  for (const auto& sample : samples_) {
    const double dx = sample.x - x;
    const double dy = sample.y - y;
    const double distance = std::hypot(dx, dy);
    const double temperature_excess = sample.temperature_c - config_.ambient_temperature_c;
    if (temperature_excess > 1.0) {
      // log(T - ambient) = log(unknown source strength) - distance / decay.
      // Profiling out the unknown strength leaves a shape-only residual.
      const double value = std::log(temperature_excess) + distance / config_.thermal_decay_m;
      thermal_sum += value;
      thermal_square_sum += value * value;
      ++thermal_count;
    }

    const double smoke_excess = sample.smoke_ppm - config_.baseline_smoke_ppm;
    if (smoke_excess > 7.0) {
      const double wind_x = std::cos(sample.wind_direction_rad);
      const double wind_y = std::sin(sample.wind_direction_rad);
      const double downstream = dx * wind_x + dy * wind_y;
      const double crosswind = -dx * wind_y + dy * wind_x;
      double log_shape;
      if (downstream >= 0.0) {
        const double width = config_.plume_width_m + config_.plume_expansion * downstream;
        log_shape = -downstream / config_.plume_length_m - 0.5 * std::pow(crosswind / width, 2.0);
      } else {
        log_shape = std::log(0.03) - distance / 1.5;
      }
      // log(smoke - background) = log(unknown emission) + log(plume shape).
      const double value = std::log(smoke_excess) - log_shape;
      smoke_sum += value;
      smoke_square_sum += value * value;
      ++smoke_count;
    }
  }

  // Scores are per observation so adding highly correlated filtered samples does
  // not artificially make the estimate appear certain.
  const double thermal_variance =
      thermal_count >= 3 ? std::max(0.0, thermal_square_sum / thermal_count -
                                             std::pow(thermal_sum / thermal_count, 2.0))
                         : 100.0;
  const double smoke_variance =
      smoke_count >= 3
          ? std::max(0.0, smoke_square_sum / smoke_count - std::pow(smoke_sum / smoke_count, 2.0))
          : 0.0;
  return thermal_variance / (0.12 * 0.12) + smoke_variance / (0.50 * 0.50);
}

std::optional<SourceEstimate> SourceEstimator::estimate() const {
  if (samples_.size() < 5U || config_.thermal_decay_m <= 0.0 || config_.plume_length_m <= 0.0 ||
      config_.plume_width_m <= 0.0 || config_.min_x >= config_.max_x ||
      config_.min_y >= config_.max_y) {
    return std::nullopt;
  }

  const auto informative =
      std::count_if(samples_.begin(), samples_.end(), [this](const SearchSample& s) {
        return s.temperature_c > config_.ambient_temperature_c + 1 ||
               s.smoke_ppm > config_.baseline_smoke_ppm + 7;
      });
  if (informative < 3) {
    return std::nullopt;
  }
  double min_x = samples_.front().x;
  double max_x = min_x;
  double min_y = samples_.front().y;
  double max_y = min_y;
  for (const auto& sample : samples_) {
    min_x = std::min(min_x, sample.x);
    max_x = std::max(max_x, sample.x);
    min_y = std::min(min_y, sample.y);
    max_y = std::max(max_y, sample.y);
  }
  if (std::hypot(max_x - min_x, max_y - min_y) < 2.0) {
    return std::nullopt;  // A stationary sensor cannot localise a source.
  }

  // Exhaustive coarse search avoids committing to the first peak in a noisy plume.
  // A small second pass removes most of the grid quantisation error.
  constexpr double coarse_step = 0.5;
  std::vector<std::pair<std::pair<double, double>, double>> candidates;
  double best_score = std::numeric_limits<double>::infinity();
  double best_x = 0.0;
  double best_y = 0.0;
  for (double x = config_.min_x; x <= config_.max_x; x += coarse_step) {
    for (double y = config_.min_y; y <= config_.max_y; y += coarse_step) {
      const double value = score(x, y);
      candidates.push_back({{x, y}, value});
      if (value < best_score) {
        best_score = value;
        best_x = x;
        best_y = y;
      }
    }
  }
  const double coarse_score = best_score;
  const double coarse_x = best_x;
  const double coarse_y = best_y;
  for (double x = coarse_x - coarse_step; x <= coarse_x + coarse_step; x += 0.1) {
    for (double y = coarse_y - coarse_step; y <= coarse_y + coarse_step; y += 0.1) {
      if (x < config_.min_x || x > config_.max_x || y < config_.min_y || y > config_.max_y) {
        continue;
      }
      const double value = score(x, y);
      if (value < best_score) {
        best_score = value;
        best_x = x;
        best_y = y;
      }
    }
  }

  double sum_w = 0.0;
  double sum_x = 0.0;
  double sum_y = 0.0;
  for (const auto& candidate : candidates) {
    const double weight = std::exp(-0.5 * (candidate.second - coarse_score));
    sum_w += weight;
    sum_x += weight * candidate.first.first;
    sum_y += weight * candidate.first.second;
  }
  const double mean_x = sum_x / sum_w;
  const double mean_y = sum_y / sum_w;
  double variance_x = 0.0;
  double variance_y = 0.0;
  double covariance_xy = 0.0;
  for (const auto& candidate : candidates) {
    const double weight = std::exp(-0.5 * (candidate.second - coarse_score));
    const double dx = candidate.first.first - mean_x;
    const double dy = candidate.first.second - mean_y;
    variance_x += weight * dx * dx;
    variance_y += weight * dy * dy;
    covariance_xy += weight * dx * dy;
  }
  // A half-metre floor acknowledges odometry drift, model mismatch and filtered lag.
  return SourceEstimate{best_x,
                        best_y,
                        std::max(0.25, variance_x / sum_w),
                        std::max(0.25, variance_y / sum_w),
                        covariance_xy / sum_w,
                        samples_.size()};
}

}  // namespace embertrace_core
