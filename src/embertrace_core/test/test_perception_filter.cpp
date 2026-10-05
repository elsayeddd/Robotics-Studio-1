#include "embertrace_core/perception_filter.hpp"
#include "embertrace_core/visual_classifier.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>
using embertrace_core::PerceptionFilter;
namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}
void checks()
{
  PerceptionFilter filter;
  for (int i = 0; i < 20; ++i) {
    const double t = 1 + 0.2 * i;
    filter.set_visual(t, 1, true, t);
    const auto r = filter.update({t, 1, 22, 2.5, 0}, t);
    require(r && !r->confirmed_candidate && r->confidence == 0, "Vision alone confirmed a fire");
  }
  filter.reset();
  require(!filter.update({10, 60, 80, 2.5, 0}, 13), "Stale sample accepted");
  require(!filter.update({15, 60, 80, 2.5, 0}, 14), "Future sample accepted");
  require(!filter.set_visual(10, 1, true, 13), "Stale visual accepted");
  for (int i = 0; i < 3; ++i) {
    const double t = 20 + 0.2 * i;
    const auto r = filter.update({t, 60, 80, 2.5, 0}, t);
    require(r && r->valid == (i == 2), "Warmup did not gate evidence");
    if (i == 2) {require(r->confirmed_candidate, "Smoke plus heat did not confirm without camera");}
  }
  require(!filter.update({20.4, 0, 22, 2.5, 0}, 20.5), "Duplicate sample accepted");
  require(!filter.update({20.2, 0, 22, 2.5, 0}, 20.5), "Out-of-order sample accepted");
  require(!filter.environment_fresh(23), "Dropout kept evidence fresh");
  auto recovered = filter.update({24, 1, 22, 2.5, 0}, 24);
  require(recovered && !recovered->valid && !recovered->smoke_detected, "Dropout kept a previous latch");
  filter.update({24.2, 1, 22, 2.5, 0}, 24.2);
  filter.update({24.4, 1, 22, 2.5, 0}, 24.4);
  require(filter.environment_fresh(24.4), "Did not recover after fresh warmup");
  require(!filter.update({24.6, std::numeric_limits<double>::quiet_NaN(), 22, 2.5, 0}, 24.6), "NaN accepted");
  require(!filter.environment_fresh(24.6), "NaN failed to clear the filter");
  auto reset_clock = filter.update({1, 1, 22, 2.5, 0}, 1);
  require(reset_clock && !reset_clock->valid, "Clock rewind did not reset warmup");
  require(!filter.set_visual(1, std::numeric_limits<double>::infinity(), true, 1), "Infinite cue accepted");
  require(!filter.update({1.2, -1, 22, 2.5, 0}, 1.2), "Negative smoke accepted");
  require(!embertrace_core::confirmation_allowed(true, false, false, 1, 1), "Confirmation bypassed evidence gate");
  require(!embertrace_core::confirmation_allowed(false, true, true, 1, 1), "Stale evidence confirmed");
  require(!embertrace_core::confirmation_allowed(true, true, false, 0, 1), "Dust alone confirmed");
  require(!embertrace_core::confirmation_allowed(true, false, true, 0, 1), "Hot surface alone confirmed");
  embertrace_core::PerceptionConfig config;
  config.smoke_exit = config.smoke_enter;
  bool rejected = false;
  try {PerceptionFilter invalid(config);} catch (const std::invalid_argument &) {rejected = true;}
  require(rejected, "Invalid hysteresis configuration accepted");
  std::vector<std::uint8_t> image(8 * 8 * 3, 0);
  for (std::size_t y = 2; y < 6; ++y) {
    for (std::size_t x = 2; x < 6; ++x) {
      const auto i = (y * 8 + x) * 3;
      image[i] = 255; image[i + 1] = 100; image[i + 2] = 10;
    }
  }
  const auto rgb = embertrace_core::flame_colour_score(image, 8, 8, 24, "rgb8", 1);
  require(rgb && *rgb == 1, "Contiguous flame-colour patch missed");
  for (std::size_t i = 0; i < image.size(); i += 3) {std::swap(image[i], image[i + 2]);}
  const auto bgr = embertrace_core::flame_colour_score(image, 8, 8, 24, "bgr8", 1);
  require(bgr && *bgr == 1, "BGR image handling failed");
  require(!embertrace_core::flame_colour_score(image, 8, 8, 1, "rgb8"), "Malformed image stride accepted");
  require(!embertrace_core::flame_colour_score({}, 8, 8, 24, "rgb8"), "Truncated image accepted");
  require(!embertrace_core::flame_colour_score(image, 8, 8, 24, "mono8"), "Unsupported image accepted");
  std::fill(image.begin(), image.end(), 0);
  image[0] = 255; image[1] = 100; image[2] = 10;
  const auto single = embertrace_core::flame_colour_score(image, 8, 8, 24, "rgb8", 1);
  require(single && *single == 0, "Isolated red noise passed cluster gate");
  // Camera expiry must not leave a positive visual observation in a new sample.
  PerceptionFilter camera;
  camera.set_visual(1, 1, true, 1);
  auto out = camera.update({1, 60, 22, 2.5, 0}, 1);
  out = camera.update({1.2, 60, 22, 2.5, 0}, 1.2);
  out = camera.update({1.4, 60, 22, 2.5, 0}, 1.4);
  require(out && out->visual_valid, "Fresh camera ignored");
  out = camera.update({2.2, 60, 22, 2.5, 0}, 2.2);
  require(out && !out->visual_valid && out->visual == 0 && !out->confirmed_candidate, "Expired camera remained positive");
}
}
int main()
{
  try {checks(); std::cout << "Perception regression checks passed\n"; return 0;}
  catch (const std::exception & e) {std::cerr << e.what() << '\n'; return 1;}
}
