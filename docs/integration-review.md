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
- Full native suite passed: **59 tests, zero errors/failures/skips**, including
  Qt lifecycle, perception/mission DDS, selected motion/frozen-clock guards,
  downwind search, Nav2 actions and waypoint recovery (`log/readiness-native-acceptance.log`).
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
  feedback. Qt shutdown now checks the ROS context before spinning.

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
Gazebo partitions. A fresh ten-seed cohort will use that controller revision.
