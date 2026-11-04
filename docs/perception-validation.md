# Perception implementation and acceptance — updated 2 November 2026

## Scope review

The updated C3 written response changes task allocation and assigns Vasili the
recorded three-scenario perception evaluation, map/TF trials (with Adam) and ten
seeded autonomous runs (with Adam). It retains the drone/rover proposal. Its
mapping/state-machine discussion is a proposed enhancement after the original
functionality, not a documented removal of IR/CO2 or a mandatory new ML detector.
No sensor scope reduction is recorded. Therefore this update implements C1 R2
(IR-camera hotspots), R3 (CO2) and the perception output for R12 (wind thresholds),
as well as roadmap Priority 4. Terrain mapping remains a separate stretch feature.

The work is based on the uploaded main ZIP, archive commit
`921cdf8168ca6598165e532beaa6de3f1032c797`, not an independently fetched live branch.

## Implemented behaviour

1. Native Fortress `thermal` cameras on rover and drone publish 160x120 `L16`
   radiometric images at 5 Hz. Existing fire geometry has a temperature of 573.15 K;
   its shape/size is unchanged. The decoy retains its appearance but is 295.15 K.
   The hot-surface scenario uses existing rock geometry at 353.15 K, without gas.
2. `thermal_verifier_node` decodes `mono16`/`16UC1` using 0.01 K/unit, or explicitly
   Celsius `32FC1`, including either byte order and padded rows. A hotspot needs
   at least four connected pixels >=50 C, minimum fraction 0.0005, and two
   consecutive qualifying frames. Invalid layouts, non-finite temperatures, stale
   acquisitions and repeated timestamps do not contribute evidence.
3. Environmental simulation publishes explicit `co2_ppm` and validity. CO2 uses
   an independent mathematical plume with background 420 ppm, emission 900 ppm
   and noise 5 ppm (default demonstrator values), with altitude attenuation.
   Dust, visual decoy, hot surface and no-fire scenarios do not emit CO2.
   This is a simulated gas field, not physical gas dispersion or calibrated hardware.
4. Smoke, local temperature and CO2 use elapsed-time EMA filtering. Three consecutive
   filtered threshold exceedances are required for each local anomaly. Hysteresis
   avoids chattering. Warmup requires three fresh samples after startup or loss.
5. Fusion uses the larger of smoke/CO2 scores as gas evidence, avoiding double
   counting correlated combustion indicators. Heat score is the larger of local
   scalar heat and image-derived hotspot heat. The scalar temperature and pose
   remain unmodified by IR peaks, preserving the source estimator's local inputs.
6. RGB connected colour clusters remain optional supporting evidence. It is a
   simulation heuristic, not an ML or validated real-world wildfire detector.
   Colour alone cannot generate fire confidence. Camera streams have timestamp,
   frame-persistence and steady-clock receipt checks.
7. Full launch requires CO2-valid raw samples and fresh IR for fire confirmation.
   Confirmation needs fresh environmental evidence, sufficient confidence and gas
   plus heat or visual corroboration. Thermal-only and CO2-only anomalies can start
   investigation; they do not independently establish a confirmed fire without
   the required corroboration. `mission_manager_node` adds its own confirmation
   count and accepts only the currently selected platform’s local evidence. Both the
   rover and drone can independently complete confirmation when selected.
8. Invalid raw values clear environmental evidence; old acquisition timestamps
   remain rejected until a clock rewind. The wall watchdog also clears stale data
   when `/clock` freezes. Status confirmation is re-evaluated when cameras expire.
9. The status message publishes wind classification: unknown, drone-safe,
   rover-only or unsafe, using configurable 5 and 12 m/s limits. These are project
   assumptions to validate, not certified equipment limits. The mission policy
   consumes stamped shared wind, applies filtering/dwell/hysteresis, selects the
   drone or rover, and pauses on unsafe/stale wind. `platform_select` now launches
   the same complete system with safe demonstration presets.
10. Sensor health monitors IR. Dashboard displays both platforms’ CO2 and IR peak, and shows
    unavailable data instead of presenting an old measurement as current. Alert
    evidence includes CO2/IR values. The existing perception-loss mission pause and
    velocity guard remain in use; frozen-time environmental loss also pauses motion.

## Architecture and interfaces

For each robot:

- Camera image -> RGB verifier -> stamped `VisualCue`.
- Thermal image -> IR verifier -> stamped `ThermalCue`.
- Odometry + independent mathematical gas/temperature models -> raw
  `EnvironmentalReading` (smoke, scalar temperature, wind, CO2).
- Local perception filter + fresh RGB/IR cues -> filtered `EnvironmentalReading`
  and `PerceptionStatus`.
- Mission/search/UI consume the filtered observations; evaluation-only truth goes
  exclusively to the recorder. Perception never subscribes to truth/source positions.

Rover roots are `/embertrace` for environmental/cue/status topics and `/husky1` for
sensor images. Drone roots are `/drone1`. Both raw and output observations preserve
acquisition and sample-pose timestamps. New fields were added to EnvironmentalReading,
PerceptionStatus, MissionState and FireAlert, plus ThermalCue/FireState and SetFireState: rebuild **all packages** after
applying these changes, then source the rebuilt workspace in every terminal.

An IR hotspot is image evidence, not a triangulated source coordinate. Source fitting
remains the search planner's responsibility. C3 map/TF and ten autonomous runs are
integration deliverables, not evidence supplied by pure perception unit tests.

## Main parameters

Defaults live in `embertrace_bringup/config/embertrace.yaml`. Treat them as provisional.

| Parameter | Default | Meaning |
|---|---|---|
| filter_alpha / nominal_period_s | 0.4 / 0.2 s | Time-adjusted EMA |
| warmup_samples / detection_samples | 3 / 3 | Startup/recovery and per-anomaly persistence |
| smoke enter / exit / scale | 12 / 8 / 45 ppm | Smoke hysteresis and confidence |
| temperature enter / exit / scale | 32 / 29 / 65 C | Scalar temperature hysteresis |
| CO2 baseline / enter / exit / scale | 420 / 700 / 600 / 1500 ppm | Explicit gas channel |
| environment_timeout_s | 2 s | ROS acquisition age and steady receipt timeout |
| visual_timeout_s / thermal_timeout_s | 1 / 1 s | Camera evidence expiry |
| future_tolerance_s | 0.1 s | Maximum timestamp allowance |
| visual_weight / alert_confidence | 0.15 / 0.74 | Supporting visual boost / confirmation threshold |
| require_co2 / require_thermal | true in full launch | Missing required evidence cannot confirm |
| IR kelvin_per_unit | 0.01 | Must match Fortress thermal sensor resolution |
| IR threshold / min cluster / required_frames | 50 C / 4 / 2 | Image hotspot rejection gates |
| drone / rover wind limit | 5 / 12 m/s | Perception classification boundaries |

Environmental confidence is `0.65*max(smoke_score, co2_score) + 0.35*heat_score`.
Vision adds `visual_weight*visual*(1-environmental)` only when environmental detection
is present. A score is a heuristic, not a calibrated probability of fire.
`use_thermal:=false` explicitly selects reduced debugging mode and does not satisfy
C1 IR acceptance. Do not use that setting to claim complete perception acceptance.

## Evaluation and test results available here

- **79 perception regression assertions passed**, covering legacy smoke behaviour, independent
  CO2, IR decoding/clusters/byte order/Celsius floats, frame persistence, temporal
  rejections, camera expiry, freeze/recovery and evidence gating.
- **45 mission policy checks, 41 system logic checks and eight Python tests passed**.
  The numerical ten-seed source-fit median error is 0.282843 m; these are not
  autonomous mission trials. See completion-handover.md for the requirement matrix.
- C++ compilation uses C++17 and `-Wall -Wextra -Wpedantic -Werror`.
- AddressSanitizer/UndefinedBehaviorSanitizer check passed (leak detection disabled
  because the execution sandbox cannot support it).
- Six scenarios x ten held-out seeds = 60 synthetic trials / 9,000 input events.
  Exact production filter used; fixtures include synthetic IR/CO2 cues, not Gazebo
  radiometric rendering. Full results are reproducible with the script below.
- Python syntax, package/world XML and unique-key YAML validation passed.
- ROS/ament build, native thermal plugin/bridge, Qt UI and live Gazebo tests have
  **not** been run here: this environment has neither ROS Humble nor Gazebo.

| Synthetic scenario | Confirmed TP | FP | FN (valid frames) | Unavailable | Events detected | Median latency |
|---|---:|---:|---:|---:|---:|---:|
| Fire | 830 | 10 | 80 | 20 | 10/10 | 1.6 s |
| Dropout | 490 | 10 | 100 | 340 | 10/10 | 1.6 s |
| Dust / hot surface / no fire / visual decoy (each) | 0 | 0 | 0 | 20 | no positive events | N/A |

These numbers are synthetic regression results only. In the dropout case, including
missing positive-labelled frames gives 420 FN and 53.85% end-to-end frame recall,
compared with 83.05% recall on valid frames. Filter clearing tails cause the reported
false positives. No failed or unavailable frames are silently omitted from overall
coverage. Dust/hot surfaces produce anomaly flags by design while remaining negative
for confirmed fire; reports separate anomaly and confirmation confusion matrices.

```bash
bash scripts/validate_all.sh
```

The evaluator accepts the old 13-column schema (legacy reduced mode) and new
19-column schema with CO2 and thermal snapshots (full sensor mode). The recorder
writes the full schema and refuses to overwrite a previous file. Acquisition/receipt
snapshots avoid introducing future image observations into an earlier gas sample.
Missing raw packets remain unavailable rows after the truth join deadline.

Trial truth is isolated from autonomy. In the simulator a positive label means a
fire exists and the robot is within configurable `evaluation_radius_m` (default 6 m)
of its horizontal source location. This is a geometric evaluation assumption,
independent of detector outputs and noisy sensor thresholds. Record the label rule
with each report; distant fires are not automatically positive local samples.

Run the Gazebo campaign on the target VM, with all manual launches stopped:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=74 python3 scripts/ros_perception_campaign.py --output log/gazebo-perception-final
```

It records six scenarios for both vehicles, with separate training and held-out
seeds, evaluates default/selected settings and checks that CO2 and IR are actually
present. Its static near-source fixture is a perception test, not an autonomous
search trial. Use a new output directory for each campaign. If headless rendering
needs a display, use `xvfb-run -a` (install xvfb first). The 27-candidate tuning sweep
optimises alpha/smoke/temperature thresholds on training recordings only, with
missing positive frames penalised. CO2/IR thresholds still need their own controlled
boundary/distractor trials. Selected settings are saved, not silently applied to YAML.

Latency is first three-consecutive-confirmation result after labelled onset;
missed events and acquisition availability are reported separately. This is not a
complete simulation of the mission state machine or CPU processing latency. Match
all replay parameters to the live YAML before comparing runs. The CLI supports
alpha, smoke/heat/CO2 entry, exit and scale settings, visual weight and persistence;
other settings use documented defaults.

## Target-machine acceptance still required

1. Clean Humble build of all packages and `colcon test-result --verbose` with no failures.
2. Transport integration test for actual ROS nodes, including required IR loss,
   environmental dropout, colour-only rejection, fire alert and E-stop latching.
3. Native IR images on both vehicles: correct encoding, known ambient/hot temperatures,
   visible vs occluded fire, small/distant hotspots, malformed or interrupted images.
4. CO2 ambient/fire/dust trials: threshold boundaries, noise, temporary spikes,
   dropout recovery and plausible response with distance/altitude.
5. Recorded confusion matrices, availability and detection latency over at least
   three scenarios. Keep negative/failed runs, separate train/test recordings.
6. C3 map/TF trial and ten autonomous seeded runs with Adam; evaluate the source
   estimator separately from local detection. Measure actual operator-alert latency.
7. Demonstrate both platforms through the shared pipeline; test UI stale warnings,
   IR/CO2 values and mission pause/recovery. Exercise the implemented shared-wind
   deployment policy, threshold hysteresis, low-battery return and tree states.

## External references and handover

- Native thermal camera, Kelvin conversion and temperature plugins:
  https://gazebosim.org/api/sensors/6/thermalcameraigngazebo.html
- ROS Humble on Ubuntu 22.04:
  https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- SSH setup:
  https://docs.github.com/en/authentication/connecting-to-github-with-ssh

See `perception-vm-startup.md` for the complete VM/GitHub/build/start/test sequence.
This contribution was developed with AI assistance. Review and understand the code,
record the assistance and retain actual test outcomes using the team's course process.
