# Priority 4: perception validation

## Implementation

The live perception node and offline CSV evaluator use the same C++ filter.
Smoke and temperature have elapsed-time exponential filtering and hysteresis.
The raw acquisition timestamp and pose timestamp are preserved through fusion.
Duplicate, reordered, stale, future and non-finite samples are rejected. A gap
longer than the environment timeout clears the old filters/latches and requires
three fresh samples before publication resumes. Simulation clock rewind also
clears evidence. A watchdog publishes perception status even when input stops.

The camera cue is now stamped and explicitly valid/invalid. The verifier supports
RGB/BGR/RGBA/BGRA, validates image dimensions/stride/buffer size, requires a
contiguous flame-colour patch and two consecutive positive frames. It remains a
simulation colour heuristic. An OpenCV/ML model was not introduced because the
static coloured flame model is sufficient to exercise integration, while no
labelled real-image dataset exists in this repository. Its adequacy for real
wildfire imagery has not been established.

The old Float32 cue topic is still published for compatibility; fusion consumes
`VisualCue`, whose timestamp prevents an old camera detection from remaining
positive indefinitely.

A confirmation requires fresh local environmental evidence and sufficient fused
confidence, plus either both smoke and heat or a fresh visual cue. Vision cannot
raise confidence when neither environmental flag is present. The mission manager
applies three consecutive confirmation samples and independently checks acquisition
age. A dropout pauses the rover's active investigation/patrol, requiring RESUME
after recovery. START cannot bypass a latched E-stop.

Drone measurements use their own filter and topics. They can initiate investigation
and guide a short rover probe, but are never substituted into the rover's local
source fit or fire confirmation. Evaluation truth is isolated from autonomy.

## Parameters

All live settings are in `embertrace_bringup/config/embertrace.yaml`.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `filter_alpha` | 0.4 | EMA update at a nominal 0.2 s interval; adjusted for actual elapsed time |
| `nominal_period_s` | 0.2 | Nominal environmental sample period |
| `smoke_enter_ppm` / `smoke_exit_ppm` | 12 / 8 | Smoke hysteresis |
| `smoke_full_scale_ppm` | 45 | Full smoke confidence scale |
| `temperature_threshold_c` / `temperature_exit_c` | 32 / 29 | Heat hysteresis |
| `temperature_full_scale_c` | 65 | Full heat confidence scale |
| `environment_timeout_s` | 2.0 | Maximum environmental acquisition age and reinitialisation gap |
| `visual_timeout_s` | 1.0 | Maximum usable camera acquisition age |
| `future_tolerance_s` | 0.1 | Small future timestamp allowance |
| `warmup_samples` | 3 | Fresh samples required after initialisation/recovery |
| `visual_weight` | 0.15 | Bounded visual contribution to existing environmental confidence |
| `visual_confirmation` | 0.55 | Minimum visual corroboration |
| `alert_confidence` | 0.74 | Confirmation confidence threshold |
| Camera `minimum_cluster` | 4 | Connected sampled pixels required |
| Camera `required_frames` | 2 | Consecutive positive camera frames |
| Health `sensor_timeout_s` | 2.0 | Required sensor freshness bound |

Environmental confidence is `0.65 * smoke_score + 0.35 * heat_score`. Vision adds
`visual_weight * visual_score * (1 - environmental_confidence)` only when smoke
or heat is detected. Smoke-only or heat-only evidence without vision can trigger
investigation but cannot independently confirm fire.

## Reproduce the standalone evaluation

No ROS installation is needed for the pure filter/regression suite:

```bash
./scripts/validate_perception.sh
```

The script compiles the live C++ filter with warnings treated as errors, generates
60 synthetic trials (10 seeds for each of six scenarios), writes a confusion
matrix/latency CSV and runs regression tests. Outputs go to ignored `log/`.
These fixtures are not recorded Gazebo trials or physical sensor calibration.

### Tuning process

A grid search evaluated alpha = 0.15/0.25/0.4, smoke enter = 10/12/15 ppm and heat
enter = 30/32/35 C on seeds 42–51. The cost was `4*FP + 2*FN` on valid classified
frames. Alpha 0.4 and the retained 12 ppm / 32 C thresholds shared the minimum
training cost of 484. The baseline alpha 0.25 at those thresholds scored 556.
The original conservative entry thresholds were retained because lower thresholds
had no training advantage. Independent seeds 142–151 were used for the report
below. Recorded Gazebo tuning is still required before priority 4's runtime
acceptance is signed off.

```bash
python3 scripts/generate_perception_trials.py log/training.csv --seed-start 42
python3 scripts/tune_perception.py log/training.csv \
  --evaluator log/perception-validation/evaluate_perception \
  --output log/threshold_sweep.csv
```

The utility also accepts recorded training CSV with the same schema. It does not
automatically edit live parameters. Verify a selected candidate on a distinct
acceptance dataset before editing YAML.

### Held-out synthetic results

Filter alpha 0.4, entry thresholds 12 ppm / 32 C, three-sample confirmation, 5 Hz
environmental samples, seeds 142–151. Positive events span 6–24 s and ramp over
two seconds. Dropout suppresses input from 12–18 s. Frame labels become negative
immediately when an event ends, so a short EMA clearing tail contributes FP frames.

| Scenario | TP | FP | TN | FN | Invalid/unavailable | Valid-frame detection rate | Availability | Detected events | Median / p95 latency |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| Fire | 817 | 10 | 560 | 93 | 20 | 89.78% | 98.67% | 10/10 | 1.8 / 2.0 s |
| Dropout | 477 | 10 | 560 | 113 | 340 | 80.85% | 77.33% | 10/10 | 1.8 / 2.0 s |
| Dust | 0 | 0 | 1480 | 0 | 20 | N/A | 98.67% | No positive event | N/A |
| Hot surface | 0 | 0 | 1480 | 0 | 20 | N/A | 98.67% | No positive event | N/A |
| Clear | 0 | 0 | 1480 | 0 | 20 | N/A | 98.67% | No positive event | N/A |
| Visual decoy | 0 | 0 | 1480 | 0 | 20 | N/A | 98.67% | No positive event | N/A |

Fire/dropout valid negative-frame false-positive rate is 1.75%; the other
scenarios have 0%. This is not a claim of 100% frame sensitivity. Event-level
10/10 detection and frame-level detection rates answer different questions.
Latency is from labelled positive-event onset to the first debounced confirmation,
not mission travel time or end-to-end ranger response time.

Warmup/dropout/rejected rows are listed separately, never silently counted as TN.
Availability is valid classified rows divided by all rows. Detection rate is
TP/(TP+FN); false-positive rate is FP/(FP+TN). NA is reported for empty denominators.
The evaluator reports onset event count, detected event count, median latency and
nearest-rank p95 latency per scenario. Each trial starts with a clean filter.

### CSV schema

```text
trial,scenario,now_s,stamp_s,smoke_ppm,temperature_c,wind_speed_mps,wind_direction_rad,visual_stamp_s,visual_confidence,visual_valid,environment_available,fire_present
```

Times are seconds in one clock domain; `now_s` is reception/evaluation time,
`stamp_s` is acquisition time. Boolean columns are 0/1. Each trial must be ordered,
with a distinct trial ID; a clock rewind starts a new filter segment. The recorder
joins raw samples and labels by exact acquisition timestamp and keeps missing
samples visible. If no labelled data is received it fails rather than producing
an apparently successful empty report.

## ROS and environment tests

`colcon test` includes the existing plume/search/patrol tests, the new C++
perception regression and a ROS transport integration test. The ROS test sends
actual image/environment messages and checks vision-only suppression, smoke-only
suppression with expired vision, multisensor alerting, E-stop latching and dropout
pause. It uses separate processes for camera, fusion and mission nodes.

The repository CI also builds every ROS package and runs the complete Gazebo
rover/drone smoke check under Xvfb. That short check covers sensor readiness,
map/TF/action availability, actual movement of both vehicles and E-stop zero
commands. It does not replace recorded perception acceptance or ten seeded source
localisation trials. CI and WSL acceptance status are recorded in the pull request.

### Recorded campaign protocol

`scripts/ros_perception_campaign.py` launches a fresh server-mode world for every
scenario and seed, with isolated Gazebo transport. The rover is stationary at
(10.5, 5.5); the drone hovers at (12, 7, 8). These are explicit test-fixture poses
used to make local evidence observable, not inputs to autonomy's source fit.
Trials cover fire, no_fire, dust, hot_surface, visual_decoy and dropout. The
dropout fixture starts at 4 simulated seconds and lasts 4 seconds; the interactive
demo retains its longer 25 s / 8 s schedule.

Each trial records both vehicles for at least 12 simulated seconds after the
first labelled observations. Rover seed 42 / drone seed 43 supply training data;
142 / 143 supply held-out data. The threshold sweep uses only rover training
observations and prefers the existing defaults when costs tie. Held-out reports
include results with both defaults and the selected candidate for each vehicle.
The campaign requires a visible rover camera cue in the visual-decoy fixture,
missing observations during dropout, rover detection in fire/dropout and zero
rover confirmations in the four negative scenarios.

The recorder snapshots visual evidence at raw-sample reception and joins truth
by the same acquisition timestamp. It preserves missing observations and their
reception times while waiting for a join, and writes observations in acquisition
order. A ROS regression checks that a later image cannot influence an earlier
record and that a dropout row cannot be overtaken or silently omitted.

These trials measure near-source local perception in the mathematical field.
They complement the independent synthetic transient/noise suite. They do not
measure patrol coverage, source-search travel time or real-world sensor accuracy.

## Remaining acceptance evidence

The implementation and synthetic evaluation are supplied. Priority 4 acceptance
still needs recorded Gazebo trials covering at least three scenarios, measured
confusion matrices/latencies, and a documented tuning/held-out split. The tools
and scenario switches above support that campaign. Physical sensor performance,
real-image false alarms and high-fidelity drone flight are outside this demo's
validated scope.
