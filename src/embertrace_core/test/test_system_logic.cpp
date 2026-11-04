#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include "embertrace_core/known_free_grid.hpp"
#include "embertrace_core/perception_filter.hpp"
#include "embertrace_core/plume_model.hpp"
#include "embertrace_core/source_estimator.hpp"
#include "embertrace_core/stream_watchdog.hpp"
using namespace embertrace_core;
namespace {
std::size_t checks = 0;
void require(bool ok, const char* reason) {
  ++checks;
  if (!ok) throw std::runtime_error(reason);
}
void run() {
  StreamWatchdog stream(1);
  require(!stream.fresh(1, 100), "Unseen stream was fresh");
  require(stream.observe(1, 1, 100), "First packet rejected");
  require(stream.fresh(1.5, 100.5), "Fresh packet ignored");
  require(!stream.observe(1, 1.5, 100.5), "Duplicate packet refreshed receipt time");
  require(!stream.fresh(1, 102), "Frozen clock bypassed wall timeout");
  require(stream.observe(0, 0, 103), "New clock epoch was not accepted");
  stream.invalidate();
  require(!stream.fresh(0, 103), "Explicit invalidation retained freshness");
  require(!stream.observe(1, std::numeric_limits<double>::quiet_NaN(), 104),
          "Invalid clock accepted");
  PerceptionConfig cfg;
  cfg.require_co2 = true;
  cfg.require_thermal = true;
  cfg.alpha = 1;
  PerceptionFilter filter(cfg);
  for (int i = 0; i < 3; ++i) {
    double t = 1 + i * .2;
    filter.set_thermal(t, 100, true, true, t);
    filter.update({t, 50, 22, 2.5, 0, 1500, true}, t);
  }
  require(filter.confirmation_candidate(1.4), "CO2/IR corroboration not available");
  filter.set_thermal(1.5, 22, false, true, 1.5);
  require(!filter.confirmation_candidate(1.5),
          "A negative IR observation retained cached confirmation");
  filter.set_thermal(2, 100, true, true, 2);
  auto old_sample = filter.update({1.6, 50, 22, 2.5, 0, 1500, true}, 2);
  require(old_sample && old_sample->thermal_valid && !old_sample->thermal_hotspot && !old_sample->confirmed_candidate,
          "Future IR evidence leaked into an older gas acquisition");
  PerceptionConfig aligned_config = cfg;
  aligned_config.warmup_samples = aligned_config.detection_samples = 1;
  PerceptionFilter causal(aligned_config);
  causal.set_thermal(3, 100, true, true, 3);
  causal.set_visual(3, .8, true, 3);
  causal.set_thermal(3.4, 22, false, true, 3.4);
  causal.set_visual(3.4, 0, true, 3.4);
  auto delayed = causal.update({3.2, 50, 22, 2.5, 0, 1500, true}, 3.4);
  require(delayed && delayed->thermal_valid && delayed->thermal_hotspot && delayed->visual == .8,
          "Newer image discarded the valid causal evidence for delayed gas");
  require(delayed->confirmed_candidate, "Causal IR/gas corroboration was lost");
  auto current = causal.update({3.4, 50, 22, 2.5, 0, 1500, true}, 3.4);
  require(current && current->thermal_valid && !current->thermal_hotspot && !current->confirmed_candidate,
          "Historical positive image overrode the current negative image");
  causal.tick(5);
  auto expired = causal.update({5, 50, 22, 2.5, 0, 1500, true}, 5);
  require(expired && !expired->thermal_valid && !expired->confirmed_candidate,
          "Image history bypassed acquisition expiry");
  KnownFreeGrid grid;
  grid.width = grid.height = 20;
  grid.resolution = .5;
  grid.origin_x = grid.origin_y = -5;
  grid.data.assign(400, 0);
  require(grid.is_free(0, 0, .5, 20), "Known free cell rejected");
  grid.data[10 * 20 + 10] = -1;
  require(!grid.is_free(0, 0, .5, 20), "Unknown cell accepted");
  require(!grid.is_free(0, 0, std::numeric_limits<double>::quiet_NaN(), 20),
          "NaN clearance accepted");
  require(!grid.nearest_free(1e300, 1e300, 2, .5, 20), "Far out-of-bounds goal accepted");
  grid.data[10 * 20 + 10] = 100;
  auto safe = grid.nearest_free(0, 0, 2, .5, 20);
  require(safe && grid.is_free(safe->first, safe->second, .5, 20), "Safe goal snapping failed");
  SourceEstimator empty_signal;
  for (int i = 0; i < 10; ++i) empty_signal.add({double(i), 0, 1, 22, 0});
  require(!empty_signal.estimate(), "Clean readings generated a false source estimate");
  SourceEstimatorConfig invalid;
  invalid.thermal_decay_m = 0;
  bool threw = false;
  try {
    SourceEstimator bad(invalid);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  require(threw, "Invalid source model accepted");
  std::vector<double> errors;
  // Source coordinates define only this test's simulator and error metric.
  // The estimator receives the sensor observations and measured sample positions.
  for (unsigned seed = 1; seed <= 10; ++seed) {
    PlumeParameters p;
    PlumeModel plume(p, seed);
    SourceEstimator estimator;
    double smoke = 0, temperature = 0;
    for (int i = 0; i < 40; ++i) {
      double downstream = 16 - .38 * i, lateral = .8 * std::sin(.45 * i + .1 * seed);
      double x = p.source_x + downstream * std::cos(p.wind_direction_rad) -
                 lateral * std::sin(p.wind_direction_rad);
      double y = p.source_y + downstream * std::sin(p.wind_direction_rad) +
                 lateral * std::cos(p.wind_direction_rad);
      auto r = plume.sample(x, y);
      smoke = i == 0 ? r.smoke_ppm : .4 * r.smoke_ppm + .6 * smoke;
      temperature = i == 0 ? r.temperature_c : .4 * r.temperature_c + .6 * temperature;
      estimator.add({x, y, smoke, temperature, r.wind_direction_rad});
    }
    auto fit = estimator.estimate();
    require(fit.has_value(), "Seeded source trial did not produce a fit");
    require(std::isfinite(fit->variance_x) && fit->variance_x >= .25, "Invalid source uncertainty");
    errors.push_back(std::hypot(fit->x - p.source_x, fit->y - p.source_y));
  }
  std::sort(errors.begin(), errors.end());
  double median = (errors[4] + errors[5]) / 2;
  require(median < 2, "Ten-seed median source error exceeds roadmap target");
  std::cout << "Ten-seed numerical source error median: " << median << " m\n";
  PlumeParameters gas;
  gas.baseline_smoke_ppm = 420;
  gas.emission_strength_ppm = 900;
  gas.smoke_noise_stddev = 0;
  PlumeModel co2(gas);
  auto source = co2.sample(gas.source_x, gas.source_y);
  require(std::abs(source.smoke_ppm - 1320) < 1e-6, "Independent CO2 field units are wrong");
  co2.set_wind(7, 0);
  require(co2.sample(0, 0).wind_speed_mps == 7, "Shared wind did not reach environmental model");
}
}  // namespace
int main() {
  try {
    run();
    std::cout << checks << " system logic checks passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
