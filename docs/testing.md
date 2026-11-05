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
Expected current result: 79 perception + 61 mission + 45 system checks and thirteen
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
also run the real wind/battery simulator with the manager: live parameter changes,
depletion/low-battery return, invalid-setting rejection and loss-of-input holds.
A native search-map fixture also checks a bounded surveyed detour when the first upwind view is unknown, minimum probe travel beyond the controller arrival radius, and holds on entirely unknown or occupied maps. A native patrol fixture verifies route-frame precedence over a configured map alias and return to home captured from a custom odometry topic. They are serialised to avoid shared-topic collisions. They do not launch Gazebo. The dual-platform image fixture allows five seconds
of host scheduling jitter; dedicated expiry tests retain production limits.

## 3. Native scene/sensor/controller integration

Use [perception-vm-startup.md](perception-vm-startup.md) for terminal-by-terminal
commands, preflight, IR inspection, wind/tree/battery controls and map/TF checks.
The smoke script starts its own launch; do not run it over a manual demo.

```bash
ROS_DOMAIN_ID=74 python3 scripts/ros_demo_smoke.py --platform combined
ROS_DOMAIN_ID=75 python3 scripts/ros_demo_smoke.py --platform rover
ROS_DOMAIN_ID=76 python3 scripts/ros_demo_smoke.py --platform drone
```

It checks native sensors, map/TF/Nav2 when present, selected-vehicle motion,
pause/resume, measured return and E-stop. Automatic mode also checks that the
unselected vehicle remains held. The launch log stays in log/demo-smoke. Use
`LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a` if a virtual display is needed.

For visible tests, add `--gui` to the mission or rover patrol recorder and
`--world-file embertrace_forest_dense.sdf` to select the larger 70 × 70 m forest.
The default remains the original 40 × 40 m forest. The dense scene preserves the
original patrol and source area, with additional trees and landmarks outside it.

On WSLg, visible launches use a software Ogre2 sensor server under Xvfb and a
separate GPU Ogre viewer on the desktop. This avoids the Ogre2/D3D12 texture-copy
failure seen on this machine while keeping IR resolution, sensor rates, physics
and vehicle limits unchanged. `gui_render_engine:=ogre2` selects the normal
combined renderer for machines with compatible drivers. Headless tests keep their
existing rendering environment. Close the launch with Ctrl-C when finished.

CI passes `--max-sensor-holds 2` for the operator smoke check. If rendering stalls
IR acquisition, it must first observe zero commands and less than 0.2 m stopped
drift, pass fresh continuous preflight, then issue an explicit RESUME. Each such
recovery is recorded; navigation failures and E-stop cannot use this path. The
default smoke check permits no unexpected hold. Autonomous acceptance always
fails on any pause and sends no recovery commands.

The scene check exercises real healthy → burning → burnt replacements and
requires the native IR and local gas readings to follow those changes:

```bash
ROS_DOMAIN_ID=77 LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=2 xvfb-run -a \
  python3 scripts/ros_scene_integration_test.py --output log/scene-integration-01
```

## 4. Recorded perception evidence

```bash
ROS_DOMAIN_ID=40 python3 scripts/ros_perception_campaign.py --output log/gazebo-perception-01
```

This launches six near-source scenarios for both platforms with separate training
and held-out seeds. It records actual inputs, checks CO2/IR presence, tunes only
on training data and reports confusion matrices, availability, missed events and
latency. At least three recorded scenarios are required; all six are preferred.
Use a new output directory each run. A failed/negative trial remains evidence.

The perception campaign reserves twelve DDS domains from `--domain-base`
(ROS_DOMAIN_ID, otherwise 40), one per world, and records source/harness revisions.
Default detector settings must detect fire/dropout and reject the four negative
scenarios on both platforms. CI smoke modes also use separate domains.

## 5. Autonomous mission evidence

```bash
ROS_DOMAIN_ID=20 python3 scripts/run_mission_campaign.py --output log/autonomous-fire-01 --trials 10
ROS_DOMAIN_ID=35 python3 scripts/ros_patrol_trials.py --output log/rover-patrol-01
ROS_DOMAIN_ID=36 python3 scripts/run_mission_campaign.py --output log/drone-patrol-01 --platform drone --scenario no_fire --trials 1 --sim-timeout 1200 --wall-timeout 1800
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

The forest launch allows 300 wall seconds for return, including turns and slower
software rendering. Required sensor/wind freshness limits still apply throughout.
Resolved source alerts require at most 4 m standard deviation per horizontal axis.
These are bounded mission settings; localisation accuracy is scored separately
against hidden truth by the campaign recorder.
The planner and manager use the same uncertainty limit. The native quality-gate
regression requires additional spatial probing when the fit is too broad, rather
than holding indefinitely at a coarse estimate. Platform fixtures wait for actual
sensor readiness within a bounded deadline before attempting their mission cases.

Autonomous trials run preflight before START, including active Nav2/first-path
checks for the rover and a two-second continuous readiness window. Preflight also
requires the event logger's mission and alert subscriptions, which are created
after its CSV has opened successfully. An unexpected launched subsystem exit
fails the trial. Negative missions require two patrol cycles, physical visits in
route order and a measured return; landing is checked in three dimensions.
Completed missions with unresolved sources remain failed acceptance results.
The campaign defaults allow 480 simulation seconds and 900 wall seconds per fire
trial, including startup. Full negative patrols need the longer explicit budgets
shown above.

The November 5 dashboard interaction check also exercised the actual window on
an isolated X11 display with controlled ROS telemetry. It verified map markers,
both sensor panels, ACK, every operator command, Return/Reset confirmation and
cancellation, all three tree buttons and the wind parameter control. Manual
rover/drone axes returned to zero on button release, connection loss and focus
loss. Screenshots and the action/assertion log remain in
`log/dashboard-interaction`; the production controller's acceptance is covered
separately by the DDS and Gazebo tests above.

Rover preflight also requires both costmaps to subscribe to the published
`/husky1/scan`; a healthy sensor topic alone does not establish collision-layer
routing. Full rover trial recorders independently evaluate the oriented local
footprint against static tree trunks, record minimum clearance and fail on overlap.
This geometry is used only by observers, not motion planning.
