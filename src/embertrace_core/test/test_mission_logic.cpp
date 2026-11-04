#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "embertrace_core/mission_logic.hpp"
using namespace embertrace_core;
namespace {
std::size_t checks = 0;
void require(bool value, const char* detail) {
  ++checks;
  if (!value) {
    throw std::runtime_error(detail);
  }
}
struct Fixture {
  MissionLogic logic;
  double time{1}, wall{100}, wind_speed{2.5};
  explicit Fixture(bool drone = true, bool rover = true, bool localise = false) : logic(configuration(drone, rover, localise)) {}
  static MissionConfig configuration(bool drone, bool rover, bool localise) {
    MissionConfig c;
    c.localise_before_hold = localise;
    c.localisation_timeout_s = 3;
    c.drone_enabled = drone;
    c.rover_enabled = rover;
    c.wind_dwell_s = .4;
    return c;
  }
  MissionEvidence reading(bool fire = false) {
    return {time, fire ? .95 : 0, 0, fire, fire, fire, true, true};
  }
  void step(bool rover_fire = false, bool drone_fire = false) {
    time += .2;
    wall += .2;
    logic.wind(wind_speed, time, time, wall);
    logic.evidence(Platform::Rover, reading(rover_fire), time, wall);
    logic.evidence(Platform::Drone, reading(drone_fire), time, wall);
  }
  void ready() {
    for (int i = 0; i < 6; ++i) step();
  }
  bool command(const std::string& command) {
    std::string response;
    return logic.command(command, time, wall, response);
  }
};
void test() {
  Fixture f;
  require(!f.command("START"), "Started without wind/sensors");
  f.ready();
  require(f.command("START"), "Low-wind start rejected");
  require(f.logic.snapshot().active == Platform::Drone, "Low wind did not select drone");
  for (int i = 0; i < 15; ++i) f.step(true, false);
  require(f.logic.snapshot().phase == MissionPhase::Patrol,
          "Unselected rover evidence changed drone mission");
  for (int i = 0; i < 15; ++i) f.step(false, true);
  require(f.logic.snapshot().phase == MissionPhase::Alert,
          "Drone local evidence cannot complete fire confirmation");
  require(f.logic.snapshot().alert_sequence == 1, "Alert not counted exactly once");
  require(f.logic.snapshot().warning_sequence == 1,
          "Suspected-anomaly warning was not counted exactly once");
  require(f.command("ACK") && f.logic.snapshot().acknowledged, "Acknowledgement failed");
  for (int i = 0; i < 55; ++i) f.step(false, true);
  require(f.logic.snapshot().phase == MissionPhase::Return,
          "Alert did not trigger automatic return");
  f.logic.arrived(Platform::Rover, f.time, f.wall);
  require(f.logic.snapshot().phase == MissionPhase::Return, "Other robot ended selected return");
  f.logic.arrived(Platform::Drone, f.time, f.wall);
  require(f.logic.snapshot().phase == MissionPhase::Idle, "Return arrival did not end mission");
  Fixture location(true, true, true);
  location.ready();
  require(location.command("START"), "Localisation mission could not start");
  for (int i = 0; i < 15; ++i) location.step(false, true);
  require(location.logic.snapshot().confirmed && location.logic.snapshot().alert_sequence == 1,
          "Confirmed fire was hidden while its source was unresolved");
  require(location.logic.snapshot().phase == MissionPhase::Track && location.logic.snapshot().motion_allowed,
          "Confirmation stopped spatial sampling before source resolution");
  location.time += .2; location.wall += .2;
  location.logic.wind(2.5, location.time, location.time, location.wall);
  auto resolved = location.reading(true); resolved.source_resolved = true;
  location.logic.evidence(Platform::Drone, resolved, location.time, location.wall);
  require(location.logic.snapshot().phase == MissionPhase::Alert && !location.logic.snapshot().motion_allowed,
          "Resolved source did not end localisation motion");
  require(location.logic.snapshot().alert_sequence == 1, "Source update counted a second fire");
  Fixture unresolved(true, true, true);
  unresolved.ready(); unresolved.command("START");
  for (int i = 0; i < 15; ++i) unresolved.step(false, true);
  for (int i = 0; i < 20; ++i) unresolved.step();
  require(unresolved.logic.snapshot().phase == MissionPhase::Alert && unresolved.logic.snapshot().confirmed,
          "Localisation timeout discarded an already confirmed fire");
  require(unresolved.logic.snapshot().alert_sequence == 1, "Unresolved fallback created another incident");
  Fixture rover(false);
  rover.ready();
  require(rover.command("START"), "Rover-only launch could not start");
  require(rover.logic.snapshot().active == Platform::Rover, "Disabled drone was selected");
  for (int i = 0; i < 15; ++i) rover.step(true, false);
  require(rover.logic.snapshot().phase == MissionPhase::Alert, "Rover confirmation failed");
  require(rover.command("E_STOP"), "E-stop rejected");
  require(!rover.command("START") && !rover.command("RESUME") && !rover.command("MANUAL_ON"),
          "E-stop bypassed");
  rover.logic.tick(0, rover.wall + .1);
  require(rover.logic.snapshot().phase == MissionPhase::EmergencyStop,
          "Clock rewind cleared E-stop latch");
  require(rover.command("RESET"), "Reset rejected");
  Fixture drone_only(true, false);
  drone_only.ready();
  require(drone_only.command("START"), "Drone-only launch could not start");
  drone_only.wind_speed = 7;
  for (int i = 0; i < 10; ++i) drone_only.step();
  require(drone_only.logic.snapshot().phase == MissionPhase::Paused &&
              drone_only.logic.snapshot().preferred == Platform::None,
          "Drone-only mission selected an absent rover in high wind");
  require(!drone_only.command("RESUME"), "Drone-only high-wind resume accepted");
  Fixture disabled(false, false);
  disabled.ready();
  require(!disabled.command("START"), "Started with both vehicles disabled");
  require(rover.command("E_STOP") && rover.command("E_STOP_RELEASE"),
          "Legacy E-stop release command was rejected");
  Fixture high;
  high.wind_speed = 7;
  high.ready();
  require(high.command("START"), "High-wind rover start rejected");
  require(high.logic.snapshot().active == Platform::Rover, "High wind selected wrong platform");
  high.wind_speed = 13;
  high.step();
  require(
      high.logic.snapshot().phase == MissionPhase::Paused && !high.logic.snapshot().motion_allowed,
      "Unsafe wind did not stop mission");
  require(!high.command("RESUME"), "Resumed in unsafe wind");
  high.wind_speed = 7;
  for (int i = 0; i < 8; ++i) high.step();
  require(high.command("RESUME"), "Safe stable wind could not resume");
  Fixture switching;
  switching.ready();
  switching.command("START");
  switching.wind_speed = 7;
  switching.step();
  require(switching.logic.snapshot().phase == MissionPhase::Paused,
          "Drone continued above wind limit");
  for (int i = 0; i < 10; ++i) switching.step();
  require(switching.command("RESUME") && switching.logic.snapshot().active == Platform::Rover,
          "Stable high wind did not transfer selected platform");
  Fixture hysteresis;
  hysteresis.wind_speed = 7;
  hysteresis.ready();
  for (int i = 0; i < 20; ++i) {
    hysteresis.wind_speed = i % 2 ? 4.9 : 5.1;
    hysteresis.step();
  }
  require(hysteresis.logic.snapshot().preferred == Platform::Rover,
          "Threshold noise caused platform oscillation");
  Fixture freeze;
  freeze.ready();
  freeze.command("START");
  freeze.logic.tick(freeze.time, freeze.wall + 3);
  require(freeze.logic.snapshot().phase == MissionPhase::Paused,
          "Frozen simulator clock retained motion permission");
  Fixture stale;
  stale.ready();
  stale.command("START");
  auto invalid = stale.reading();
  invalid.stamp -= 3;
  require(!stale.logic.evidence(Platform::Drone, invalid, stale.time, stale.wall),
          "Stale local evidence accepted");
  stale.logic.tick(stale.time, stale.wall);
  require(stale.logic.snapshot().phase == MissionPhase::Paused,
          "Rejected selected evidence did not pause");
  Fixture visual;
  visual.ready();
  visual.command("START");
  for (int i = 0; i < 20; ++i) {
    visual.step();
    auto r = visual.reading();
    r.stamp += .01;
    r.confidence = 1;
    r.visual = 1;
    visual.logic.evidence(Platform::Drone, r, visual.time + .01, visual.wall + .01);
  }
  require(visual.logic.snapshot().alert_sequence == 0, "Vision-only readings confirmed fire");
  for (bool heat_only : {true, false}) {
    Fixture anomaly;
    anomaly.ready();
    anomaly.command("START");
    for (int i = 0; i < 15; ++i) {
      anomaly.time += .2;
      anomaly.wall += .2;
      anomaly.logic.wind(2.5, anomaly.time, anomaly.time, anomaly.wall);
      auto r = anomaly.reading();
      r.confidence = .7;
      r.heat = heat_only;
      r.co2 = !heat_only;
      anomaly.logic.evidence(Platform::Drone, r, anomaly.time, anomaly.wall);
    }
    require(anomaly.logic.snapshot().warning_sequence == 1 &&
                anomaly.logic.snapshot().alert_sequence == 0 && anomaly.command("ACK"),
            "Independent heat/CO2 anomaly failed warning/ACK or falsely confirmed");
  }
  Fixture battery;
  battery.ready();
  battery.command("START");
  battery.logic.battery(Platform::Drone, .19, battery.time, battery.wall);
  require(battery.logic.snapshot().phase == MissionPhase::Return, "Low battery did not return");
  Fixture complete(false);
  complete.ready();
  complete.command("START");
  complete.logic.loops(Platform::Rover, 1, complete.time, complete.wall);
  require(complete.logic.snapshot().phase == MissionPhase::Patrol,
          "Returned before required coverage");
  complete.logic.loops(Platform::Rover, 2, complete.time, complete.wall);
  require(complete.logic.snapshot().phase == MissionPhase::Return,
          "Completed coverage did not return");
  Fixture manual;
  manual.ready();
  manual.command("START");
  require(manual.command("MANUAL_ON") && manual.logic.snapshot().manual, "Manual control rejected");
  require(!manual.logic.snapshot().motion_allowed, "Manual mode retained autonomous permission");
  manual.wind_speed = 20;
  manual.step();
  require(!manual.logic.snapshot().manual, "Unsafe wind retained manual control");
  require(!manual.command("UNKNOWN"), "Unknown command accepted");
  Fixture manual_battery;
  manual_battery.ready();
  manual_battery.command("START");
  manual_battery.command("MANUAL_ON");
  manual_battery.logic.battery(Platform::Drone, .1, manual_battery.time, manual_battery.wall);
  require(manual_battery.logic.snapshot().phase == MissionPhase::Return &&
              !manual_battery.logic.snapshot().manual,
          "Manual control bypassed low-battery return");
  Fixture invalid_clock;
  invalid_clock.ready();
  invalid_clock.command("START");
  invalid_clock.logic.tick(std::numeric_limits<double>::quiet_NaN(), invalid_clock.wall);
  require(invalid_clock.logic.snapshot().phase == MissionPhase::Paused &&
              !invalid_clock.logic.snapshot().motion_allowed,
          "Invalid clock retained motion permission");
  Fixture return_timeout;
  return_timeout.ready();
  return_timeout.command("START");
  return_timeout.command("RETURN");
  for (int i = 0; i < 610; ++i) return_timeout.step();
  require(return_timeout.logic.snapshot().phase == MissionPhase::Paused,
          "Return remained active beyond its deadline");
  Fixture partial;
  partial.ready();
  require(partial.command("START"), "Partial-evidence test could not start");
  auto partial_step = [&]() {
    partial.time += .2;
    partial.wall += .2;
    partial.logic.wind(2.5, partial.time, partial.time, partial.wall);
    auto r = partial.reading(true);
    r.confidence = .70;
    partial.logic.evidence(Platform::Drone, r, partial.time, partial.wall);
  };
  for (int i = 0; i < 10; ++i) partial_step();
  require(partial.logic.snapshot().phase == MissionPhase::Verify,
          "Partial evidence did not enter verification");
  for (int i = 0; i < 80; ++i) partial_step();
  require(partial.logic.snapshot().phase == MissionPhase::Track &&
              partial.logic.snapshot().motion_allowed,
          "Verification timeout did not permit new spatial evidence");
  for (int i = 0; i < 10; ++i) partial_step();
  require(partial.logic.snapshot().phase == MissionPhase::Track,
          "Verification retried before collecting new spatial evidence");
  Fixture low_start;
  low_start.ready();
  low_start.logic.battery(Platform::Drone, .1, low_start.time, low_start.wall);
  require(!low_start.command("START"), "Low-battery platform accepted deployment");
  Fixture error(false);
  error.ready();
  error.command("START");
  error.logic.navigation_failed(Platform::Rover, error.time, error.wall);
  require(error.logic.snapshot().phase == MissionPhase::Paused, "Navigation failure did not pause");
  MissionConfig invalid_config;
  invalid_config.wind_alpha = 0;
  bool throws = false;
  try {
    MissionLogic invalid_logic(invalid_config);
  } catch (const std::invalid_argument&) {
    throws = true;
  }
  require(throws, "Invalid mission configuration accepted");
}
}  // namespace
int main() {
  try {
    test();
    std::cout << checks << " mission policy checks passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
