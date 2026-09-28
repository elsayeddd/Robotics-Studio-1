#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "embertrace_core/known_free_grid.hpp"
#include "embertrace_core/plume_model.hpp"
#include "embertrace_core/source_estimator.hpp"

namespace embertrace_core
{

TEST(SourceEstimator, RejectsInvalidAndRepeatedPositionsAndBoundsHistory)
{
  SourceEstimatorConfig config;
  config.max_samples = 5;
  SourceEstimator estimator(config);
  EXPECT_FALSE(estimator.add({
    std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0, 22.0, 0.0}));
  EXPECT_EQ(estimator.sample_count(), 0U);
  EXPECT_TRUE(estimator.add({0.0, 0.0, 2.0, 23.0, 0.0}));
  EXPECT_TRUE(estimator.add({0.1, 0.0, 3.0, 24.0, 0.0}));
  EXPECT_EQ(estimator.sample_count(), 1U);
  for (int i = 1; i < 15; ++i) {
    estimator.add({static_cast<double>(i), 0.0, 10.0, 30.0, 0.0});
  }
  EXPECT_EQ(estimator.sample_count(), 5U);
  estimator.clear();
  EXPECT_EQ(estimator.sample_count(), 0U);
  EXPECT_FALSE(estimator.estimate().has_value());
}

TEST(SourceEstimator, RequiresSpatialEvidence)
{
  SourceEstimator estimator;
  for (int i = 0; i < 10; ++i) {
    estimator.add({0.0, 0.0, 15.0, 35.0, 0.0});
  }
  EXPECT_FALSE(estimator.estimate().has_value());
}

// The test knows the simulator source; the estimator sees only sensor samples.
// Ten independently seeded noise sequences exercise the roadmap's error metric.
TEST(SourceEstimator, TenSeededStandardScenarioTrials)
{
  std::vector<double> errors;
  for (unsigned int seed = 1; seed <= 10; ++seed) {
    PlumeParameters parameters;
    PlumeModel plume(parameters, seed);
    SourceEstimator estimator;
    const double wind_x = std::cos(parameters.wind_direction_rad);
    const double wind_y = std::sin(parameters.wind_direction_rad);
    double filtered_smoke = 0.0;
    double filtered_temperature = 0.0;
    for (int i = 0; i < 40; ++i) {
      const double downstream = 16.0 - 0.38 * i;
      const double lateral = 0.8 * std::sin(0.45 * i + 0.1 * seed);
      const double x = parameters.source_x + downstream * wind_x -
        lateral * wind_y;
      const double y = parameters.source_y + downstream * wind_y +
        lateral * wind_x;
      const auto sample = plume.sample(x, y);
      // Mirror the baseline perception node's 0.25 exponential filter.
      if (i == 0) {
        filtered_smoke = sample.smoke_ppm;
        filtered_temperature = sample.temperature_c;
      } else {
        filtered_smoke = 0.25 * sample.smoke_ppm + 0.75 * filtered_smoke;
        filtered_temperature = 0.25 * sample.temperature_c + 0.75 * filtered_temperature;
      }
      estimator.add({
        x, y, filtered_smoke, filtered_temperature, sample.wind_direction_rad});
    }
    const auto estimate = estimator.estimate();
    ASSERT_TRUE(estimate.has_value());
    errors.push_back(std::hypot(
      estimate->x - parameters.source_x, estimate->y - parameters.source_y));
    EXPECT_GE(estimate->variance_x, 0.25);
    EXPECT_GE(estimate->variance_y, 0.25);
  }
  std::sort(errors.begin(), errors.end());
  const double median = (errors[4] + errors[5]) / 2.0;
  std::cout << "Ten-seed median source error: " << median << " m\n";
  EXPECT_LT(median, 2.0) << "Median source error over ten seeded trials";
}

TEST(SourceEstimator, FollowsChangedSourceWithoutGroundTruthInput)
{
  PlumeParameters parameters;
  parameters.source_x = 4.0;
  parameters.source_y = -3.0;
  parameters.wind_direction_rad = 0.4;
  parameters.smoke_noise_stddev = 0.0;
  PlumeModel plume(parameters);
  SourceEstimator estimator;
  for (int i = 0; i < 35; ++i) {
    const double downstream = 14.0 - 0.35 * i;
    const double lateral = std::sin(0.4 * i);
    const double x = parameters.source_x + downstream * std::cos(parameters.wind_direction_rad) -
      lateral * std::sin(parameters.wind_direction_rad);
    const double y = parameters.source_y + downstream * std::sin(parameters.wind_direction_rad) +
      lateral * std::cos(parameters.wind_direction_rad);
    const auto sample = plume.sample(x, y);
    estimator.add({x, y, sample.smoke_ppm, sample.temperature_c, sample.wind_direction_rad});
  }
  const auto estimate = estimator.estimate();
  ASSERT_TRUE(estimate.has_value());
  EXPECT_LT(std::hypot(estimate->x - 4.0, estimate->y + 3.0), 2.0);
}

// This route is fixed in world coordinates. Changing the source does not move
// the samples toward it, and the simulated field is deliberately different
// from the estimator's configured plume/thermal shape.
TEST(SourceEstimator, LocalisesMovedSourcesOnAnUnchangedSurveyRoute)
{
  std::vector<PlumeParameters> scenarios(3);
  scenarios[0].source_x = 12.0;
  scenarios[0].source_y = 7.0;
  scenarios[1].source_x = 4.0;
  scenarios[1].source_y = -3.0;
  scenarios[1].wind_direction_rad = 0.4;
  scenarios[1].plume_length_m = 10.0;
  scenarios[1].plume_width_m = 2.1;
  scenarios[1].thermal_decay_m = 9.0;
  scenarios[2].source_x = -6.0;
  scenarios[2].source_y = 6.0;
  scenarios[2].wind_direction_rad = -0.7;
  scenarios[2].plume_length_m = 18.0;
  scenarios[2].plume_width_m = 1.3;
  scenarios[2].thermal_decay_m = 5.5;

  for (std::size_t scenario = 0; scenario < scenarios.size(); ++scenario) {
    const auto & parameters = scenarios[scenario];
    PlumeModel plume(parameters, static_cast<unsigned int>(scenario + 17));
    SourceEstimator estimator;
    for (int row = 0; row < 5; ++row) {
      const double y = -8.0 + 6.0 * row;
      for (int col = 0; col < 21; ++col) {
        const int route_col = row % 2 == 0 ? col : 20 - col;
        const double x = -15.0 + 1.5 * route_col;
        const auto sample = plume.sample(x, y);
        estimator.add({
          x, y, sample.smoke_ppm, sample.temperature_c, sample.wind_direction_rad});
      }
    }
    const auto estimate = estimator.estimate();
    ASSERT_TRUE(estimate.has_value()) << "Scenario " << scenario;
    const double error = std::hypot(
      estimate->x - parameters.source_x, estimate->y - parameters.source_y);
    EXPECT_LT(error, 3.0) << "Scenario " << scenario;
  }
}

// Translating every measured position without changing sensor values must
// translate the fit. A fixed or simulator-parameter-derived answer fails this.
TEST(SourceEstimator, TranslationOfEvidenceTranslatesEstimate)
{
  PlumeParameters parameters;
  parameters.smoke_noise_stddev = 0.0;
  PlumeModel plume(parameters);
  SourceEstimator original;
  SourceEstimator translated;
  constexpr double offset_x = -8.0;
  constexpr double offset_y = -5.0;
  for (int i = 0; i < 35; ++i) {
    const double downstream = 14.0 - 0.35 * i;
    const double lateral = std::sin(0.3 * i);
    const double x = parameters.source_x +
      downstream * std::cos(parameters.wind_direction_rad) -
      lateral * std::sin(parameters.wind_direction_rad);
    const double y = parameters.source_y +
      downstream * std::sin(parameters.wind_direction_rad) +
      lateral * std::cos(parameters.wind_direction_rad);
    const auto sample = plume.sample(x, y);
    original.add({x, y, sample.smoke_ppm, sample.temperature_c, sample.wind_direction_rad});
    translated.add({
      x + offset_x, y + offset_y, sample.smoke_ppm, sample.temperature_c,
      sample.wind_direction_rad});
  }
  const auto first = original.estimate();
  const auto second = translated.estimate();
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_NEAR(second->x - first->x, offset_x, 0.2);
  EXPECT_NEAR(second->y - first->y, offset_y, 0.2);
}

TEST(KnownFreeGrid, RejectsUnknownOccupiedAndMapEdges)
{
  KnownFreeGrid grid;
  grid.width = 20;
  grid.height = 20;
  grid.resolution = 0.5;
  grid.origin_x = -5.0;
  grid.origin_y = -5.0;
  grid.data.assign(400, 0);
  EXPECT_TRUE(grid.is_free(0.0, 0.0, 0.5, 20));
  grid.data[10 * 20 + 10] = -1;
  EXPECT_FALSE(grid.is_free(0.0, 0.0, 0.5, 20));
  grid.data[10 * 20 + 10] = 100;
  EXPECT_FALSE(grid.is_free(0.0, 0.0, 0.5, 20));
  EXPECT_FALSE(grid.is_free(-4.9, -4.9, 0.8, 20));
  const auto safe = grid.nearest_free(0.0, 0.0, 2.0, 0.5, 20);
  ASSERT_TRUE(safe.has_value());
  EXPECT_TRUE(grid.is_free(safe->first, safe->second, 0.5, 20));
  EXPECT_FALSE(grid.nearest_free(30.0, 30.0, 2.0, 0.5, 20).has_value());
}

TEST(KnownFreeGrid, HandlesRotatedOrigin)
{
  KnownFreeGrid grid;
  grid.width = 10;
  grid.height = 10;
  grid.resolution = 1.0;
  grid.origin_yaw = 1.5707963267948966;
  grid.data.assign(100, 0);
  // Local (2.5, 3.5) maps to world (-3.5, 2.5).
  grid.data[3 * 10 + 2] = 100;
  EXPECT_FALSE(grid.is_free(-3.5, 2.5, 0.0, 20));
  EXPECT_TRUE(grid.is_free(-4.5, 2.5, 0.0, 20));
}

}  // namespace embertrace_core
