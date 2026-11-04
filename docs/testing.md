# Testing and acceptance

The November 2 standalone results and requirement matrix are in
[completion-handover.md](completion-handover.md). See [integration-review.md](integration-review.md) for native results and
remaining acceptance for the freshly merged branches. Run the following layers
on the target Ubuntu 22.04/Humble/Fortress VM and retain failures.

## 1. Standalone logic and fixtures

```bash
bash scripts/validate_all.sh
```

This compiles the actual pure C++ perception, mission and numerical libraries,
runs scene/replay Python checks, and generates six-scenario synthetic metrics.
Expected current result: 79 perception + 61 mission + 45 system checks and eight
Python tests. Synthetic fixtures are not Gazebo camera or autonomous trial evidence.

## 2. Clean native build and unit/transport tests

Stop other EmberTrace launches first. Source Humble and the new workspace.

```bash
./scripts/build.sh
source install/setup.bash
ROS_DOMAIN_ID=73 colcon test --event-handlers console_direct+
colcon test-result --verbose
```

The suite includes existing plume, search, YAML patrol and Nav2 action tests, the
pure regressions, scene/replay checks, and ROS perception, mission, search and motion transport tests. The latter test
real perception/mission nodes with controlled messages, both platform confirmation
paths, wind selection, stale/IR loss, vision-only rejection and E-stop. ROS tests
include a 250 ms delayed environmental stream and confirmed-source refinement,
stable incident IDs, reset isolation, bounded fallback and unsafe-wind holds. They
are serialised to avoid shared-topic collisions. They do not launch Gazebo. The dual-platform image fixture allows five seconds
of host scheduling jitter; dedicated expiry tests retain production limits.

## 3. Native scene/sensor/controller integration

Use [perception-vm-startup.md](perception-vm-startup.md) for terminal-by-terminal
commands, preflight, IR inspection, wind/tree/battery controls and map/TF checks.
The smoke script starts its own launch; do not run it over a manual demo.

```bash
ROS_DOMAIN_ID=74 python3 scripts/ros_demo_smoke.py --platform combined
ROS_DOMAIN_ID=74 python3 scripts/ros_demo_smoke.py --platform rover
ROS_DOMAIN_ID=74 python3 scripts/ros_demo_smoke.py --platform drone
```

It checks native sensors, map/TF/Nav2 when present, selected-vehicle motion,
pause/resume, measured return and E-stop. Automatic mode also checks that the
unselected vehicle remains held. The launch log stays in log/demo-smoke. Use
`LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a` if a virtual display is needed.

The scene check exercises real healthy → burning → burnt replacements and
requires the native IR and local gas readings to follow those changes:

```bash
ROS_DOMAIN_ID=76 LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=2 xvfb-run -a \
  python3 scripts/ros_scene_integration_test.py --output log/scene-integration-01
```

## 4. Recorded perception evidence

```bash
ROS_DOMAIN_ID=74 python3 scripts/ros_perception_campaign.py --output log/gazebo-perception-01
```

This launches six near-source scenarios for both platforms with separate training
and held-out seeds. It records actual inputs, checks CO2/IR presence, tunes only
on training data and reports confusion matrices, availability, missed events and
latency. At least three recorded scenarios are required; all six are preferred.
Use a new output directory each run. A failed/negative trial remains evidence.

## 5. Autonomous mission evidence

```bash
ROS_DOMAIN_ID=75 python3 scripts/run_mission_campaign.py --output log/autonomous-fire-01 --trials 10
ROS_DOMAIN_ID=75 python3 scripts/run_mission_campaign.py --output log/autonomous-clear-01 --trials 2 --scenario no_fire --sim-timeout 1200 --wall-timeout 1800
```

Trials alternate drone/rover selection and retain individual launch logs and JSON.
Record completion/return, path length, recoveries, local-exposure alert latency,
false alerts and resolved source error. Missing estimates must be reported, not
silently removed to make a median look better. The source-fit numerical test's
0.282843 m median does not establish the roadmap's autonomous <2 m acceptance.
Manually inspect collisions and map/TF, which the metrics script cannot certify.

Include image occlusion, tree transitions, wind threshold noise, missing/frozen
streams, battery return, goal cancellation, manual release/focus loss, alert ACK,
RESET and E-stop. Repeat in the dense world after the small world passes.

`run_mission_campaign.py` records controller and harness revisions separately.
It deduplicates incident IDs, records the first confirmation timestamp and keeps
source-resolution latency separate. `early_detection` means confirmation before
entry into the evaluation radius; remote IR may legitimately do that. Detection
latency is zero if already confirmed on entry and otherwise starts at that local
exposure. Missing exposure/estimates remain explicit. Use `ros_patrol_trials.py`
for the stricter rover two-loop check: no skipped stops, no false critical alerts
and automatic measured return, without an operator RETURN request.

Mission trials use independent Gazebo partitions and DDS domains. `--domain-base`
reserves one domain per trial (default: ROS_DOMAIN_ID, otherwise 20). This keeps
old simulation clocks/readiness packets from contaminating the next fresh world.
Readiness status carries the acquisition timestamp/frame; a native regression
checks delayed samples against that contract.

The environmental publisher regression freezes ROS time, advances it and resets
the epoch. It requires exactly one raw/truth acquisition for each timestamp;
consumer replay rejection and monotonic expiry remain tested separately. The
rover patrol check records physical visits in route order, including the final
base closure, as well as loop counters and automatic return.
