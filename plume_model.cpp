#include "embertrace_core/plume_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace embertrace_core {

PlumeModel::PlumeModel(const PlumeParameters& parameters, const std::uint32_t seed)
    : parameters_(parameters),
      random_engine_(seed),
      smoke_noise_(0.0, std::max(1e-12, parameters.smoke_noise_stddev)) {
  for (double value :
       {parameters.source_x, parameters.source_y, parameters.emission_strength_ppm,
        parameters.baseline_smoke_ppm, parameters.plume_length_m, parameters.plume_width_m,
        parameters.plume_expansion, parameters.smoke_noise_stddev, parameters.ambient_temperature_c,
        parameters.source_temperature_c, parameters.thermal_decay_m, parameters.wind_direction_rad,
        parameters.wind_speed_mps}) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("Plume parameters must be finite");
    }
  }
  if (parameters.emission_strength_ppm < 0 || parameters.baseline_smoke_ppm < 0 ||
      parameters.plume_length_m <= 0 || parameters.plume_width_m <= 0 ||
      parameters.plume_expansion < 0 || parameters.smoke_noise_stddev < 0 ||
      parameters.thermal_decay_m <= 0 || parameters.wind_speed_mps < 0) {
    throw std::invalid_argument("Invalid plume scale/noise parameters");
  }
}

void PlumeModel::set_wind(double speed, double direction) {
  if (!std::isfinite(speed) || speed < 0 || speed > 100 || !std::isfinite(direction)) {
    throw std::invalid_argument("Invalid ambient wind");
  }
  parameters_.wind_speed_mps = speed;
  parameters_.wind_direction_rad = direction;
}

PlumeSample PlumeModel::sample_without_noise(const double x, const double y) const {
  if (!std::isfinite(x) || !std::isfinite(y)) {
    throw std::invalid_argument("Invalid sampling position");
  }
  const double dx = x - parameters_.source_x;
  const double dy = y - parameters_.source_y;
  const double distance = std::hypot(dx, dy);
  const double wind_x = std::cos(parameters_.wind_direction_rad);
  const double wind_y = std::sin(parameters_.wind_direction_rad);

  // Positive downstream means that the query point lies in the direction smoke travels.
  const double downstream = dx * wind_x + dy * wind_y;
  const double crosswind = -dx * wind_y + dy * wind_x;

  double plume_contribution = 0.0;
  if (downstream >= 0.0) {
    const double sigma = parameters_.plume_width_m + parameters_.plume_expansion * downstream;
    const double length_term = std::exp(-downstream / parameters_.plume_length_m);
    const double lateral_term = std::exp(-0.5 * std::pow(crosswind / sigma, 2.0));
    plume_contribution = parameters_.emission_strength_ppm * length_term * lateral_term;
  } else {
    // A small source halo avoids an unrealistic hard discontinuity immediately upwind.
    plume_contribution = 0.03 * parameters_.emission_strength_ppm * std::exp(-distance / 1.5);
  }

  PlumeSample result;
  result.smoke_ppm = parameters_.baseline_smoke_ppm + plume_contribution;
  result.temperature_c = parameters_.ambient_temperature_c +
                         (parameters_.source_temperature_c - parameters_.ambient_temperature_c) *
                             std::exp(-distance / parameters_.thermal_decay_m);
  result.wind_speed_mps = parameters_.wind_speed_mps;
  result.wind_direction_rad = parameters_.wind_direction_rad;
  return result;
}

PlumeSample PlumeModel::sample(const double x, const double y) {
  PlumeSample result = sample_without_noise(x, y);
  if (parameters_.smoke_noise_stddev > 0) {
    result.smoke_ppm = std::max(0.0, result.smoke_ppm + smoke_noise_(random_engine_));
  }
  return result;
}

}  // namespace embertrace_core
