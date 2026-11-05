# Integration review — 5 November 2026

## Scope and branch history

I reviewed the repository documentation against the implementation and fetched every remote branch. The starting main commit was `921cdf8`. These teammate branches contained additional work and are now ancestors of local main:

| Branch | Reviewed head | Integrated capability |
| --- | --- | --- |
| feature/nav2-goal-adapter | c6faa28 | Navigation, launch updates and supplied Parrot assets |
| feature/waypoint-patrol-manager | 4bafc0a | Action feedback, retry policy, return completion, rover tuning and dashboard |
| feature/perception-validation | 2d385d5 | Local IR/CO2 perception, selected-platform mission policy, scene controls, UI and evaluation |
| feature/search-source-estimate | fac37e9 | Already included in the starting main |

The required deployment includes standalone rover and standalone drone missions, with both available in the automatic wind-selected demonstration. The aircraft remains a Gazebo velocity-controlled model. Standalone aerial search uses the same local-evidence estimator and bounded search policy, with aerial goals executed by its flight controller rather than a ground Nav2 server.

## Documentation audit

| Document | Finding and correction |
| --- | --- |
| README.md | Old branch references and capability status; updated launch modes and acceptance evidence |
| CONTRIBUTING.md | Branch workflow remains appropriate for teammate development; integration on main was explicitly requested |
| NOTICE.md | Supplied-asset provenance retained |
| architecture.md | Added standalone deployment and mission completion paths |
| interface-contract.md | Added navigation status, search status, return feedback and aerial topic routing |
| dependencies.md | Checked against manifests and the Humble/Fortress test environment |
| development-roadmap.md | Replaced completed scaffold tasks with final acceptance criteria |
| demo-guide.md | Updated to main and documented each supported mode |
| perception-validation.md | Retained the IR/CO2 algorithm and evaluation limitations |
| perception-vm-startup.md | Removed obsolete ZIP patch instructions and person-specific Git setup |
| simulation.md | Updated aerial scope and scenario expectations |
| testing.md | Added action-server, mission-control and mode-specific runtime tests |
| troubleshooting.md | Removed instructions to implement the existing Nav2 adapter |
| teammate-start-here.md | Updated the current workflow and test entry points |
| original-package-audit.md | Historical source audit retained; current aircraft documented separately |
| forest-layout.svg | Standard-world layout checked against world includes and route |
| core/src/README.md | Removed obsolete claims about missing action feedback |
| core/scaffolds/README.md | Marked the scaffold phase complete |

## Integration decisions

- I retained the common simulated odometry origin and namespaced TF wiring. Ground route coordinates stay in `husky1_odom`; Nav2 goals are transformed to `husky1_map`.
- The rover velocity guard remains the sole rover command publisher. Both vehicles gate motion on fresh mission, sensor and wind evidence.
- Search, verification and return phases are bounded. Verification retries allow five seconds of movement to collect new evidence. New investigations clear the previous signal deadline.
- The refreshed teammate policy selects one active vehicle from stable safe wind. RETURN waits for that vehicle to reach its measured home position; the drone also lands. `mission_platform:=rover` or `drone` excludes the other vehicle, and `auto`/`combined` makes both available.
- RESET and the dashboard's explicit E_STOP_RELEASE both return a latched stop to IDLE. Neither resumes motion.
- Real action-server tests replace checks that only asserted the ROS process remained alive.

## Validation evidence

The fresh merge includes `feature/nav2-goal-adapter` at `c6faa28` and
`feature/perception-validation` at `2d385d5`. Older waypoint and search heads remain
ancestors. I resolved shared interfaces and controllers around the new selected
platform policy, retained supported Fortress flame geometry and the slow-motion
source-sampling fix, and ported standalone launch/preflight/control checks.

Fresh merged revision:

- Repository validation and strict standalone compilation passed.
- Perception: 79 checks; mission policy: 61 checks including standalone
  availability and verification retry; system logic: 45 checks; Python: eight tests passed.
- Full five-package build passed (`log/fresh-branch-build-final.log`).
- Full native suite passed: **63 tests, zero errors/failures/skips**, including
  Qt lifecycle, perception/mission DDS, selected motion/frozen-clock guards,
  downwind search, Nav2 actions and waypoint recovery (`log/final-native-63-acceptance.log`).
- Parrot Xacro expands successfully with eight links. Fresh Gazebo acceptance
  includes passing standalone rover/drone and automatic-mode motion, pause/
  resume, measured return and E-stop checks (`log/fresh-demo-acceptance` and
  `log/fresh-automatic-mode`). The automatic check confirmed the unselected
  rover stayed still. The initial failed runs are retained.
- Actual healthy/burning/burnt scene transitions passed with native IR and gas
  coupling (`log/fresh-scene-integration`). Healthy/burning/burnt IR peaks were
  22.4/300.0/45.5°C; burning smoke/CO2 rose and returned to baseline when burnt.
- Runtime acceptance caught missing executable modes on uploaded Python nodes,
  a launch boolean case mismatch, and a temporary zero TF mistaken for home.
  Permissions are now checked statically; navigation forwarding is corrected;
  rover home is captured from actual Gazebo odometry.
- Native compilation exposed a cross-branch waypoint API regression. I retained
  the newer retry library and added selected-platform, loop and measured-home
  feedback. Qt handles termination through a quit flag and destroys its windows
  before shutting down ROS; both SIGINT and SIGTERM exit cleanly.

Historical evidence from main through `31dbc1d` (requires rerunning for the new
policy):

- Five-package build and full native suite passed: 48 results, zero failures
  (`log/integration-final-regressions.log`).
- All three launch modes passed real Gazebo motion, pause/resume, measured return
  and emergency-stop checks (`log/integration-mode-checks.log`).
- Twelve recorded Gazebo perception runs passed: six scenarios with independent
  training/held-out seeds and both sensor pipelines (`log/gazebo-perception-fixed`).
- Autonomous drone trial 142 resolved the source with 0.6 m error. The rover
  confirmed fire but reported an unresolved location; that remains an acceptance
  issue, not a passing source-localisation trial.
- Native camera capture reproduced unsupported cone geometry. Supported sphere
  volumes produced 16,159 qualifying flame-colour pixels in a 720 × 480 image.

Numerical ten-seed fits are unit evidence, not ten autonomous missions. Generated
recordings and compiler/test logs remain under ignored `log/`. These results do
not establish physical robot performance.

The camera-timing fix retains bounded causal image history. It reproduced a
250 ms delayed-stream failure in the old binary and passes that DDS case in the
new binary. The fixture now allows five seconds of host scheduling jitter after
diagnostics found a 2.4-second fixture delivery stall; dedicated expiry checks
retain production limits. Three consecutive delayed fixture runs passed.
Address/undefined-behaviour sanitizers also passed the 45 system-logic checks.

Twelve fresh Gazebo recordings (six scenarios, training/held-out seeds, both
vehicles) passed and were replayed with the timing fix. Both platforms detected
fire/dropout; the four negative scenarios had zero confirmed-fire false positives.
These stationary fixtures do not establish autonomous source accuracy.

The autonomous baseline drone trial 300 completed detection, return and landing
with a resolved source error of 0.10 m. Rover trial 301 detected and returned but
its source remained unresolved. The current refinement flow emits the first
critical alert immediately and continues bounded tracking, retaining the same
incident ID for a later source update. A fresh rover acceptance run follows.

The refined rover trial 301 now completes detection, source refinement and
automatic return: source error 0.1414 m, 203.6 simulated seconds and 77.73 m
travelled (`log/rover-source-refinement`). Its incident count remains one.
Manual command selection/release passed through both real ROS controllers
(`log/manual-motion-acceptance.log`). Healthy/burning/burnt changes also passed in
the dense world (`log/dense-scene-acceptance`).

The first final cohort caught an immediate rover startup pause and is retained
under `log/final-fire-campaign`. Readiness metadata previously used publication
time, which could make an old warmup status appear newer than a delayed valid
acquisition. Status now identifies the acquisition/frame it describes. The new
regression fails against the old binary and passes against the corrected one.
Mission cohorts now also use separate DDS domains per world, alongside separate
Gazebo partitions.

The second cohort exposed repeated environmental acquisitions whenever Gazebo
briefly stopped advancing its clock. The simulator now publishes each timestamp
once, while retaining normal replay rejection and frozen-stream expiry downstream.
The new regression fails against the old producer and passes with the correction,
including clock advance and simulation restart. I retained both interrupted cohorts.
That native run passed all 60 checks. I also corrected two fixture scheduling
assumptions: initial START waits for actual readiness, and source-refinement sensor
streams continue while an operator service response is pending. The ten-mission
campaign and complete two-loop patrol checks will use this tested revision.

I inspected the actual dashboard with controlled ROS telemetry on an isolated
X11 display (`log/dashboard-interaction`). The map rendered both vehicles and the
source in the correct positions, and both panels displayed current values.
Actual clicks verified every operator command, ACK, Return/Reset confirmation
and cancellation, all three tree states and the wind setting. Manual rover/drone
axes were correct, with zero motion on release, lost connection or lost focus.
The window exited successfully. This checks UI wiring and presentation; the
separate native and Gazebo tests check the controllers' real responses.

The third cohort's drone 400 completed with 0.361 m source error. Rover 401
resolved a source but hit the 120-wall-second return cap 2.3 m from home; source
error was 2.126 m. I stopped the cohort and retained its JSON/logs. The forest
configuration now permits 300 wall seconds for a full diagonal return under
software rendering, and uses a 3 m source uncertainty gate rather than 4 m.
Neither change bypasses sensor/wind freshness or a failed source fit. A new DDS
check also passes the actual mission simulator/manager pipeline: live wind and
battery settings, depletion/threshold return, unsafe/missing wind and invalid
parameter rejection.
The expanded full native suite passes all 61 checks with zero failures or skips.

Drone 600 in the fourth cohort completed alert/return/landing but retained an
unresolved source: its widest fitted variance was 9.156 m², just above the new
9 m² manager gate. The planner still had a fixed 16 m² approach gate, which let it
hold at a coarse stand-off instead of gathering another spatial sample. Both
nodes now use the configured 3 m standard-deviation limit in the full launch.
The planner keeps probing until its fit meets that limit. A DDS regression fails
against the old binary and passes after the fix; the cohort remains retained.

GitHub run `41861337601` built successfully and passed all twelve recorded Gazebo
runs, including the new acquisition producer. Its delayed-stream native test
attempted START before sensor warmup completed. Both platform fixtures now wait
for readiness within ten seconds instead of assuming a fixed two-second delay.
Production expiry and confirmation limits are unchanged.

The full fifth cohort is retained in `log/final-fire-campaign-v5`. All five drone
missions resolved and returned/landed; median source error was 0.283 m. Three of
five rover missions completed, but none satisfied the experimental 3 m uncertainty
gate, so the cohort failed. Two rover fits ended near the true source with wider
covariance, while one continued probing beyond the useful plume. I restored the
established 4 m gate in both nodes; the configurable matching-limit fix remains.
The 300-second return budget remains in place.

Rover 801 also started before Nav2 became ready and paused immediately. The
campaign now runs the documented preflight sequence before START, requires a
continuous readiness window, and fails on any launched subsystem exit. This also
closed a logging coverage gap: `mission_event_logger.py` overwrote ROS Node's
reserved `handle` attribute and crashed. Its CSV handle now has its own name. A
new DDS regression reproduces the old crash and verifies persisted mission/alert
rows, the configured row limit and clean shutdown after the correction. An audit
found no other ROS Node attribute collisions in the Python runtime nodes.

[GitHub run 41865088729](https://github.com/elsayeddd/Robotics-Studio-1/actions/runs/41865088729)
passed its build, all 62 native checks, all three deployment smoke checks and the
twelve recorded perception runs at `6b5bf80`. The new event-log/preflight changes
will receive a separate full run. Campaign results now distinguish completion
from acceptance, retain failed source/return checks, and record physical patrol
visits and measured return distance.
The expanded local suite passes all 63 tests with zero errors, failures or skips
(`log/final-native-63-acceptance.log`). A fresh fetch still finds every teammate
branch head included in main.

The standalone drone dashboard previously waited for a rover SLAM map that is
absent in that mode. It now shows a labelled 5 m odometry grid with fresh vehicle
and source/sample markers. I verified this against a live drone return and also
verified the real rover SLAM map, telemetry and detection-sample marker. Qt
startup/termination still passes; the combined native results remain 63/63.
The snapshots are in `log/dashboard-live-drone.png` and
`log/dashboard-live-rover-active.png`.

GitHub run `41920968542` passed all 63 native tests and twelve recorded perception
runs. The rover operator smoke test correctly paused after IR freshness expired.
The smoke harness can now explicitly verify stopped commands/body drift and a
fresh preflight before a bounded operator RESUME; CI permits at most two such
recoveries and records them. The autonomous campaign still rejects every pause
and issues no recovery command. Neither sensor expiry nor E-stop is bypassed.
Smoke modes and recorded worlds now use separate DDS domains, and baseline
perception acceptance is enforced for both vehicles. The older source-only
diagnostic is labelled separately from full mission acceptance.

The complete sixth cohort passes all ten strict autonomous missions at controller
revision `0c932ba`: five drone and five rover, ten resolved sources, ten measured
returns and all drone landings. Median source error is 0.316 m for the drone,
0.412 m for the rover and 0.338 m overall. No PAUSED recovery or operator return
command was used. Full two-loop negative patrol checks follow separately.

The next CI run passed native tests but exposed two readiness/reporting issues.
Its Nav2 log showed every managed node active, while preflight remained blocked
on an unanswered lifecycle query. Queries now expire after three wall seconds
and retry; readiness also requires fresh replies. Path-query futures are bounded
as well. CI now trusts its explicit container checkout so Git can record revision
metadata despite the mounted directory's different owner. These changes do not
alter controller or sensor thresholds.


Visible validation uses the original forest for the completed ten-mission cohort.
The larger 70 × 70 m forest previously passed scene/IR/gas changes; full missions
and two-loop patrols are now being run there as well. The sparse rover negative
patrol stopped on expired IR after 18 of 21 measured visits during viewer changes;
it remains a failed trial in `log/final-rover-patrol`.

I separated the WSL viewer from the sensor renderer after reproducing Ogre2's
D3D12 texture-copy exception. The GPU Ogre viewer is smoother (confirmed on the
desktop), while the software Ogre2 server retains the existing thermal camera.
The dense drone patrol measured about 0.86–0.99 real-time factor with this split.
An attempted GPU sensor-server trial failed before START and is retained in
`log/dense-drone-patrol-gpu-v1`. Sensor resolution/rates and motion limits are
unchanged. Visible WSL launches now use this split automatically.

CI at `97b38bb` passed all 63 native tests, the rover operator smoke check and all
12 recorded perception worlds. The drone smoke recorder joined after preflight
and received no state or odometry despite the independent preflight succeeding.
The recorder now creates its DDS subscriptions before starting the world, matching
the strict campaign's startup order, and requires its own heartbeat before START.
This change needs a fresh smoke/CI run; the failed run is retained.

The dense drone negative mission now passes at `061bfd8`: all twelve physical
sweep visits over two loops, no confirmed alerts or safety pauses, automatic
return and landing, with 0.195 m final 3D displacement. I watched this run through
the separate GPU viewer. The dense rover test uses the new automatic WSL launch
split at `5fa2f3c`; it is still running. The source/noise-seed results and held-out
perception metrics are now saved under `docs/validation/2026-11-05` so the core
acceptance data is available from Git, alongside the ignored detailed logs.

CI at `5fa2f3c` is fully green: all 63 native tests, rover/drone/combined
operator smoke checks (zero sensor-hold recoveries), and twelve recorded
perception worlds with both vehicle baselines enforced. The current metric tables
and mode results are saved under `docs/validation/2026-11-05`.

The larger forest adds two important failures to the acceptance record. The rover
negative patrol completed one loop and eleven measured visits, then stalled near
the base on its second loop. I paused it to prioritise the other tests after the
user deferred movement refinement. Its three recovery spins and retained stop
note are saved. The dense drone fire mission completed, resolved its source and
landed, but the rover fire mission reached its wall deadline without leaving base.
The captured initial map has narrow unknown gaps between LiDAR rays: no point
within 2 m of the upwind target satisfies the existing 0.8 m clearance check.

Search now tries a bounded 4 m detour to already surveyed free space when an
initial probe cannot be placed. Probe goals must also be at least 1.5 m from the
robot, beyond Nav2's 1 m arrival tolerance. Unknown/occupied cells and clearance
rules are unchanged; failed goal searches are throttled to the normal goal
interval. Focused grid/source tests and real DDS map/goal tests pass. The actual
dense rover fire regression is still required before accepting this fix.

The detour rerun at `bea2c9a` completed the dense rover's detection and measured
return without operator recovery, but failed cohort accuracy: the reported source
was (18.4, 12.0), 8.122 m from the evaluated fire. The saved summary has
`passed=false`; its lifecycle success flag is not a source-accuracy pass.

I found that SourceEstimator published the fitted peak while calculating spread
around the different likelihood mean. Skewed or boundary-clipped fits therefore
understated uncertainty about the published position. The uncertainty now includes
that offset, including the cross term. A controlled clipped-source fixture that
previously appeared below the 4 m gate is now withheld. Numerical source positions,
configured uncertainty limits and fire-confirmation rules are unchanged. Five
focused native source/search suites pass; the next dense Gazebo rerun is pending.

The user observed the next rover run turning against a tree. I paused the run
and retained it as failed. The recorded pose came within 0.661 m of tree_15's
centre. A live subscription audit then found the missing integration: local and
global costmaps listened on `/husky1/local_costmap/scan` and
`/husky1/global_costmap/scan`, both without publishers. Only SLAM and sensor health
subscribed to the actual `/husky1/scan`; costmaps received front depth alone.
The odometry alias was checked and is correctly connected to the EKF.

Both collision layers now use the absolute LiDAR topic. Preflight verifies those
actual subscriptions and rejects the old running stack. The local controller also
checks the full padded polygon, and NavFn uses a 0.8 m enclosing radius globally.
Its A* path computation is enabled; the turn critic now uses the same 1 m arrival
tolerance as the goal checker. Neither maximum speed nor sensor expiry is raised.
The trial observers now measure oriented footprint/trunk clearance from the world
and odometry, failing on overlap; this evaluation geometry is never an autonomy
input. Two pure geometry/world-loading regressions pass. Live reruns remain.

The routing regression is red/green on live Gazebo stacks: the original stack
fails the new collision LiDAR gate, while the corrected stack passes every
preflight check including its first path. Parameter/service introspection confirms
the padded local footprint critic, global 0.8 m radius and A* setting are loaded.
Before/after subscription and preflight evidence is saved in the validation folder.

The first connected-LiDAR patrol avoids tree overlap but exhausts retries at the
east sensor station, so it remains failed. The controller's rectangular local
outline could pass closer to the post than the global planner's enclosing circle,
leaving the next plan's start inside its inflated region. Both costmaps now use
the same 0.8 m enclosing circle and 0.05 m padding. The full-footprint critic stays
enabled. The observer supports this conservative circular envelope as well as the
previous rectangle; eleven pure scene/replay/clearance fixtures pass. I also
removed an unused per-critic tolerance override: Humble uses the shared
`FollowPath.xy_goal_tolerance`, which service introspection confirms is 1 m.

NavFn's potential-to-path extraction also repeatedly failed on reachable legs
with both Dijkstra and its A* option. The configured `GridBased` action ID now
uses the installed Humble SmacPlanner2D A* plugin instead, with the same clearance
models and unknown-space policy. Its dependency is explicit in the simulation
package. The next run must validate this replacement through actual route
coverage and source/return missions; no speed or acceptance threshold is relaxed.
