#include "embertrace_core/perception_filter.hpp"
#include "embertrace_core/visual_classifier.hpp"
#include "embertrace_core/thermal_detector.hpp"
#include "embertrace_core/image_evidence.hpp"
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>
using embertrace_core::PerceptionFilter;
namespace
{
std::size_t checks_run = 0;
void require(bool condition, const char * message)
{
  ++checks_run;
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

  // C1 full sensor mode: explicit gas validity and live IR are required.
  embertrace_core::PerceptionConfig full_config;
  full_config.require_co2 = full_config.require_thermal = true;
  full_config.alpha = 1;
  PerceptionFilter full(full_config);
  require(!full.update({1, 1, 22, 2.5, 0}, 1), "Missing CO2 accepted in full mode");
  embertrace_core::PerceptionResult result;
  for (int i=0; i<3; ++i) {
    double t=2+i*.2;
    full.set_thermal(t, 100, true, true, t);
    result=*full.update({t, 1, 22, 2.5, 0, 1500, true}, t);
  }
  require(result.valid && result.co2_detected && !result.smoke_detected,
    "CO2 does not have an independent detection channel");
  require(result.thermal_hotspot && result.temperature_anomaly && result.temperature == 22,
    "IR hotspot lost or substituted for local temperature");
  require(result.confirmed_candidate, "CO2 plus IR did not confirm without RGB");
  full.tick(3.8);
  require(!full.confirmation_candidate(3.8), "Expired IR kept a confirmation candidate");
  result=*full.update({3.8, 1, 80, 2.5, 0, 1500, true}, 3.8);
  require(!result.confirmed_candidate, "Missing required IR bypassed confirmation gate");
  full.tick(3.8, 100);
  full.tick(3.8, 103);
  require(!full.environment_fresh(3.8), "Frozen ROS clock retained environmental evidence");
  auto next=full.update({3.9, 1, 22, 2.5, 0, 420, true},3.9,103.1);
  require(next && !next->valid, "Dropout recovery skipped warmup");
  require(!full.update({3.9,1,22,2.5,0,420,true},4,103.2), "Replayed timestamp accepted");
  require(!full.environment_fresh(4), "Rejected sample failed to clear evidence");
  require(!full.update({3.8,1,22,2.5,0,420,true},4.1,103.3), "Older timestamp accepted after rejection");
  require(full.update({0,1,22,2.5,0,420,true},0,103.4).has_value(), "Clock epoch rewind rejected first zero sample");
  PerceptionFilter late;
  for (int i=0;i<10;++i) {late.update({1+i*.2,1,22,0,0},1+i*.2);}
  auto spike=late.update({3,60,80,0,0},3);
  require(spike && !spike->smoke_detected && !spike->temperature_anomaly,
    "Late isolated spike bypassed per-anomaly persistence");
  for (int i=0;i<5;++i) {late.update({3.2+i*.2,60,80,0,0},3.2+i*.2);}
  require(late.environment_fresh(4), "Persistent post-warmup anomaly was not processed");
  // Radiometric decoding, row padding, endianness, cluster rejection and frame gating.
  std::vector<std::uint8_t> ir(4*10,0);
  const auto pixel=[&](int x,int y,int c,bool big=false) {
    unsigned n=static_cast<unsigned>((c+273.15)*100+.5);
    ir[y*10+x*2]=static_cast<std::uint8_t>(big ? n>>8 : n&255);
    ir[y*10+x*2+1]=static_cast<std::uint8_t>(big ? n&255 : n>>8);
  };
  for(int y=0;y<4;++y)for(int x=0;x<4;++x)pixel(x,y,22);
  auto thermal=embertrace_core::thermal_hotspot(ir,4,4,10,"mono16",false);
  require(thermal && !thermal->hotspot && std::abs(thermal->peak_c-22)<.02,"Ambient IR decode failed");
  pixel(1,1,100);
  require(!embertrace_core::thermal_hotspot(ir,4,4,10,"mono16",false)->hotspot,"Single IR noise pixel confirmed");
  pixel(1,2,100);pixel(2,1,100);pixel(2,2,100);
  thermal=embertrace_core::thermal_hotspot(ir,4,4,10,"mono16",false);
  require(thermal && thermal->hotspot && thermal->hot_pixels==4 && std::abs(thermal->peak_c-100)<.02,
    "Connected radiometric hotspot not detected");
  for(int y=0;y<4;++y)for(int x=0;x<4;++x)pixel(x,y,100,true);
  require(embertrace_core::thermal_hotspot(ir,4,4,10,"16UC1",true)->hotspot,"Big-endian IR decode failed");
  require(!embertrace_core::thermal_hotspot(ir,4,4,7,"mono16",false),"Malformed IR row accepted");
  require(!embertrace_core::thermal_hotspot({},4,4,8,"mono16",false),"Truncated IR accepted");
  require(!embertrace_core::thermal_hotspot(ir,4,4,10,"rgb8",false),"RGB passed as radiometric IR");
  std::vector<std::uint8_t> floats(16);
  float celsius=80; std::uint32_t bits; std::memcpy(&bits,&celsius,4);
  for(int pixel=0;pixel<4;++pixel)for(int b=0;b<4;++b)floats[pixel*4+b]=(bits>>(8*b))&255;
  require(embertrace_core::thermal_hotspot(floats,2,2,8,"32FC1",false)->hotspot,
    "Celsius float IR decode failed");
  celsius=std::numeric_limits<float>::quiet_NaN();std::memcpy(&bits,&celsius,4);
  for(int b=0;b<4;++b)floats[b]=(bits>>(8*b))&255;
  require(!embertrace_core::thermal_hotspot(floats,2,2,8,"32FC1",false),"NaN IR image accepted");
  require(!full.update({.2,1,22,0,0,std::numeric_limits<double>::quiet_NaN(),true},.2,103.6),
    "Invalid explicit CO2 accepted");
  embertrace_core::ImageEvidence persistence(2,1);
  require(persistence.observe(0,true,true,0,100) && !persistence.positive(),"Image first frame not gated");
  require(persistence.observe(.2,true,true,.2,100.2) && persistence.positive(),"Consecutive frames missed");
  require(!persistence.observe(.2,true,true,.3,100.3) && !persistence.positive(),"Duplicate image retained evidence");
  persistence.observe(.4,true,true,.4,100.4);
  persistence.observe(.6,true,true,.6,100.6);
  persistence.tick(.6,102);
  require(!persistence.available(),"Frozen clock retained camera evidence");
  require(!persistence.observe(.7,false,true,.7,102.1),"Malformed image contributed evidence");

}
}
int main()
{
  try {checks(); std::cout << checks_run << " perception regression checks passed\n"; return 0;}
  catch (const std::exception & e) {std::cerr << e.what() << '\n'; return 1;}
}
