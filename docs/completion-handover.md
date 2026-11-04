# Completion handover — 2 November 2026

This update completes the perception code paths and connects them to the mission,
both simulated platforms, the operator interface and reproducible evaluation tools.
The November 5 integration merges teammate branch `feature/perception-validation`
at `2d385d5` and navigation assets at `c6faa28` into main. The original November 2
handover used archive `921cdf8`; the current integration uses freshly fetched Git
branches. See [integration review](integration-review.md) for current validation
and the distinction between historical and fresh runtime results.

## Requirements traced to implementation

Reviewed sources: C1 proposal, Development Roadmap, repository descriptions and the
latest supplied C3 technical progress written response. The C3 update changes task
allocation and adds recorded evaluation responsibilities; it does not remove IR,
CO2 or dual-platform operation. RGB machine learning is not a mandatory replacement
for the documented supporting colour detector. Mapping enhancement and terrain
mapping remain extensions to the original MVP.

| Requirement | Implementation in this snapshot | Acceptance still to collect |
| --- | --- | --- |
| R1 waypoint navigation | YAML rover patrol through Nav2; bounded aerial sweep; two-loop completion | Both selected platforms finish routes; paused patrol resumes at nearest waypoint |
| R2 IR hotspot detection | Native radiometric cameras on both models; endian/stride-aware decoding, connected hot regions, consecutive frames | Fortress image conversion, known temperatures, visibility, occlusion and distant/small targets |
| R3 elevated CO2 | Explicit independently simulated concentration, validity, EMA, hysteresis, persistence and gas fusion | Ambient/fire/dust, boundary values, spikes and dropout in recorded Gazebo runs |
| R4 terrain mapping (stretch) | Existing SLAM occupancy map, map-safe rover goals and dashboard map are integrated | This does not implement a new 3D terrain model |
| R5 healthy/burning/burnt trees | Tree service changes visual geometry/material/thermal state and gas emissions together; unavailable scene changes withhold raw samples | Confirm remove/create, all three appearances and coupled readings in both supplied worlds |
| R6 investigate anomaly | Selected vehicle progresses patrol → investigate → track/recover → verify; bounded history and timeouts | Autonomous plume tracking, loss/recovery and obstacle handling |
| R7 location and sensor alert | Suspected-anomaly warning on investigation, followed by a separately gated confirmed alert; platform/id/time, gas/heat/IR/wind, resolved source or explicitly labelled detection-sample fallback | Live source accuracy and alert latency; fallback must not be counted as a resolved source |
| R8 robot/source UI | Qt occupancy map, both robot positions/telemetry, source/sample marker, batteries and stale/disconnected states | Map alignment, frame transforms, stale displays and window responsiveness |
| R9 operator commands | START/PAUSE/RESUME/RETURN/RESET/E_STOP plus ACK; responses and event log | Click/service behaviour and cancellations with Nav2 active |
| R10 manual control (stretch) | Hold/release dashboard buttons; shared command input; selected-platform gate, timeouts and bounds | Release, focus loss, UI exit and E-stop while moving |
| R11 automatic return | Return after two patrol loops, alert hold or low battery; measured launch position; arrival ends mission | Rover return and drone landing; battery threshold and return timeout |
| R12 wind measurement | Configurable simulated stamped wind; filtered policy with dwell/hysteresis and stale-data checks | Real ROS parameter changes and frozen/missing wind |
| R13 wind deployment | ≤5 m/s drone, >5 to ≤12 m/s rover, >12 m/s neither; immediate stop on unsafe change | Boundary/noise/dropout trials; RESUME reselects only after stable safe inputs |
| Roadmap P4 evaluation | Same production filter used by recorder/replay, confusion matrices, availability, missed events, latency and training-only threshold sweep | At least three recorded scenarios; keep negatives and failures |
| C3 recorded integration | Map/TF checks, dual-platform transport test, ten-seed autonomous campaign with retained logs/results | Vasili/Adam collect ten actual autonomous trials and report failures and median source error |

Wind limits are project demonstration assumptions, not certified aircraft/rover
limits. The drone remains the supplied ideal velocity-controlled proxy: takeoff,
patrol, investigation and landing commands are implemented, but rotor dynamics and
a physical Parrot autopilot are outside this simulator model.

## What changed since the October 20 delivery

- Added early suspected-anomaly warnings containing location and sensor values;
  independent IR or CO2 anomalies can warn without being labelled confirmed fire.
- Added a pure mission policy with independent vehicle evidence, wind dispatch,
  mission epochs, E-stop latching, manual control, acknowledgements, completion,
  low-battery return and bounded failure recovery.
- Connected the selected vehicle's local observations to mission confirmation,
  search and the RViz heatmap. An unselected vehicle cannot confirm its mission.
- Added shared wind/battery simulation, real scene-state changes, persistent event
  logging and the expanded public messages/services needed by those features.
- Reworked drone control for ascent, local investigation, two patrol loops and
  return/landing. Added goal/command/sensor watchdogs and flight-envelope guards.
- Connected rover selection to patrol/Nav2 cancellation and the sole velocity
  guard; return now targets the measured launch pose.
- Added a Qt map and both-platform telemetry, stale-data display, alert
  acknowledgement, tree/wind controls and bounded manual commands.
- Hardened time alignment, source-fit input validation, free-space indexing,
  clock-reset behaviour and source covariance handling. IR peaks remain separate
  from local scalar temperature; camera images do not supply a source coordinate.
- Added tests and recording scripts, updated launch/configuration/dependencies,
  and documented setup and acceptance. Comments describe the data, decisions and
  failure behaviour of the implementation.

## Standalone checks from the teammate handover

Run `bash scripts/validate_all.sh` to reproduce the normal standalone suite.

| Check | Observed result | Scope |
| --- | --- | --- |
| Perception regression | 79 checks passed | Fusion, images, CO2, time ordering, freshness and confirmation gates |
| Mission policy regression | 45 checks passed | Selected-platform confirmation, dispatch, unsafe/frozen inputs, battery, return, completion, manual mode and E-stop |
| System logic regression | 41 checks passed | Stream watchdog, cue alignment, stale IR, free-grid boundaries and numerical source fits |
| Python fixtures/replay | 8 tests passed | Scene geometry/states and replay accounting/error handling |
| Address/undefined-behaviour sanitizers | All three pure suites passed | Leak checking disabled in this container |
| C++ compilation flags | C++17, `-Wall -Wextra -Wpedantic -Werror` | ROS-independent sources only |
| Synthetic held-out replay | Six scenarios × ten seeds, 9,000 input events | Synthetic samples and image cues, not rendered Gazebo images |
| Ten numerical source fits | Median horizontal error 0.282843 m | Samples generated directly from the model; not ten autonomous missions |

Synthetic confirmation results are unchanged: fire 830 TP/10 FP/80 FN valid frames,
10/10 events detected, median latency 1.6 s. Dropout has 490 TP/10 FP/100 valid FN
and 340 unavailable frames. Counting unavailable positive frames gives 420 FN and
53.85% end-to-end recall. Each negative scenario (dust, hot surface, no fire,
visual decoy) has zero confirmed-fire FP. These are regression measurements, not
evidence that every real scene is correctly detected.

## VM acceptance sequence

Follow [perception-vm-startup.md](perception-vm-startup.md) for exact installation,
GitHub/branch, build and terminal commands. Do these in order:

1. Pull current main and review the integration notes; no ZIP patch is required.
2. Rebuild all five packages and run all native tests. Fix any build/test failures
   before treating the snapshot as accepted.
3. Inspect native IR/CO2/RGB/odometry and run preflight without navigation.
4. Demonstrate both local confirmation paths using controlled fixtures.
5. Test tree states, wind thresholds, selected-platform gating, stale/frozen
   streams, manual controls, acknowledgements and low-battery return.
6. Enable Nav2; verify map/TF, known-free paths, cancellation and base arrival.
7. Record the perception campaign and inspect confusion/availability/latency
   outputs. Keep training and held-out recordings separate.
8. Record ten autonomous seeded runs, plus negative missions. Inspect every failure
   and every unresolved source; report coverage as well as median error.

No physical IR camera or CO2 sensor is needed for this simulation-only deliverable.
A physical deployment would require hardware drivers, calibration and a separate
validation process; the mathematical fields are not hardware drivers.
