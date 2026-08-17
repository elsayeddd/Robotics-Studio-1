#include <gtest/gtest.h>

#include "embertrace_core/plume_model.hpp"

namespace embertrace_core
{

TEST(PlumeModel, SourceIsStrongerThanFarDownwind)
{
  PlumeParameters parameters;
  parameters.smoke_noise_stddev = 0.0;
  parameters.wind_direction_rad = 0.0;
  PlumeModel model(parameters);

  const auto source = model.sample_without_noise(parameters.source_x, parameters.source_y);
  const auto far = model.sample_without_noise(parameters.source_x + 15.0, parameters.source_y);
  EXPECT_GT(source.smoke_ppm, far.smoke_ppm);
  EXPECT_GT(source.temperature_c, far.temperature_c);
}

TEST(PlumeModel, CentrelineIsStrongerThanCrosswindPoint)
{
  PlumeParameters parameters;
  parameters.smoke_noise_stddev = 0.0;
  parameters.wind_direction_rad = 0.0;
  PlumeModel model(parameters);

  const auto centre = model.sample_without_noise(parameters.source_x + 8.0, parameters.source_y);
  const auto side = model.sample_without_noise(parameters.source_x + 8.0, parameters.source_y + 8.0);
  EXPECT_GT(centre.smoke_ppm, side.smoke_ppm);
}

TEST(PlumeModel, UpwindPointIsNearBaseline)
{
  PlumeParameters parameters;
  parameters.smoke_noise_stddev = 0.0;
  parameters.wind_direction_rad = 0.0;
  PlumeModel model(parameters);

  const auto upstream = model.sample_without_noise(parameters.source_x - 8.0, parameters.source_y);
  EXPECT_NEAR(upstream.smoke_ppm, parameters.baseline_smoke_ppm, 0.2);
}

}  // namespace embertrace_core
