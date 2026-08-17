#ifndef EMBERTRACE_CORE__PLUME_MODEL_HPP_
#define EMBERTRACE_CORE__PLUME_MODEL_HPP_

#include <cstdint>
#include <random>

namespace embertrace_core
{

struct PlumeParameters
{
  double source_x{12.0};
  double source_y{7.0};
  double emission_strength_ppm{120.0};
  double baseline_smoke_ppm{1.0};
  double plume_length_m{14.0};
  double plume_width_m{1.6};
  double plume_expansion{0.05};
  double smoke_noise_stddev{2.0};
  double ambient_temperature_c{22.0};
  double source_temperature_c{110.0};
  double thermal_decay_m{7.0};
  double wind_direction_rad{-2.35619449};
  double wind_speed_mps{2.5};
};

struct PlumeSample
{
  double smoke_ppm{0.0};
  double temperature_c{0.0};
  double wind_speed_mps{0.0};
  double wind_direction_rad{0.0};
};

/// Deterministic, low-cost environmental model for simulation and unit tests.
class PlumeModel
{
public:
  explicit PlumeModel(const PlumeParameters & parameters, std::uint32_t seed = 42U);

  PlumeSample sample(double x, double y);
  PlumeSample sample_without_noise(double x, double y) const;

private:
  PlumeParameters parameters_;
  std::mt19937 random_engine_;
  std::normal_distribution<double> smoke_noise_;
};

}  // namespace embertrace_core

#endif  // EMBERTRACE_CORE__PLUME_MODEL_HPP_
