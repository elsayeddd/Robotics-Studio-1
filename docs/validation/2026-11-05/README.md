# Integration validation — 5 November 2026

I merged the latest teammate branches into main and tested the selected rover and
selected drone as independent missions. The current aircraft is the documented
velocity-controlled simulation proxy. These results cover the simulator and ROS
integration; they do not establish physical sensor or flight performance.

| Check | Recorded result | Evidence |
| --- | --- | --- |
| Five-package native build and regression suite | 63 tests, zero failures/errors/skips | Local `log/final-native-63-acceptance.log`; [CI at 5fa2f3c](https://github.com/elsayeddd/Robotics-Studio-1/actions/runs/41932033248), native test step |
| Standard 40 × 40 m forest, autonomous fire missions | 10/10 accepted, five per vehicle; every source resolved and vehicle returned; all drones landed | [Complete results](standard-forest-fire-missions.json) |
| Source position accuracy in those missions | Median 0.338 m overall, 0.316 m drone, 0.412 m rover | Same complete results; five noise seeds per vehicle, fixed source location |
| Dense 70 × 70 m forest, drone negative patrol | Two loops, all 12 sweep visits, no alerts/pauses, automatic return and landing; final 3D displacement 0.195 m | [Complete result](dense-forest-drone-patrol.json) |
| Recorded native perception | Six scenarios × training/held-out worlds; both vehicles checked; fire/dropout events detected and four negative scenarios have zero confirmed false positives | [Rover held-out metrics](recorded-perception-husky1.csv), [drone held-out metrics](recorded-perception-drone1.csv) |
| Dynamic scene changes | Healthy/burning/burnt IR and gas changes pass in both worlds | Local `log/fresh-scene-integration/result.json`, `log/dense-scene-acceptance/result.json` |
| Operator dashboard | Actual START/PAUSE/RESUME/RETURN/RESET/E-stop/ACK/manual controls, confirmation cancellation, tree/wind controls and zero release/focus/disconnection commands pass | Local `log/dashboard-interaction/result.json`; live rover/drone screenshots and Qt lifecycle tests |
| Shared wind and battery | Live simulator parameter changes, depletion/low-battery return, invalid-setting rejection and unsafe/missing-input holds pass | Local `log/mission-inputs-acceptance.log` and native suite |

The dense rover negative patrol completed one loop and 11/21 measured visits,
then stalled near the base on the second loop. I issued PAUSE to move on to the
remaining tests, as requested. This run is incomplete and is not accepted:
[result](dense-forest-rover-incomplete-result.json) and
[operator stop/recovery counts](dense-forest-rover-incomplete-operator-stop-note.json).
Planner/path-following refinement was explicitly deferred until the other tests
finish. No controller configuration was changed during this run.

The larger-forest fire cohort accepted the drone mission, including a resolved
source and automatic landing, but the rover remained at base and reached its wall
deadline. [Both retained results](dense-forest-fire-missions-before-detour.json)
show this as one of two accepted missions. The captured initial map had no
clearance-safe cell within 2 m of the upwind target. A bounded detour to surveyed
space now passes the real DDS regression, which fails against the previous
planner. Its full Gazebo rerun and a fresh CI run are pending.

[CI at 5fa2f3c](https://github.com/elsayeddd/Robotics-Studio-1/actions/runs/41932033248)
is fully green: 63 native tests, all three mode smoke checks and twelve recorded
perception worlds. The [mode results](ci-mode-checks.json) show zero unexpected
sensor-hold recoveries in every mode and no unselected rover motion in automatic
mode. The held-out CSV tables above are from this run. This CI result precedes
the new search-detour change.

The strict mission recorder sends START, then observes autonomy. It sends no
operator RETURN or RESUME, rejects every safety pause, requires a resolved source
for fire trials and measures the final home position. Negative trials additionally
require the configured route visits in order and two completed loops. Nav2's own
bounded recovery is part of the rover controller and is recorded separately.

Detection timing uses the first confirmed incident, while source-refinement
timing uses the later resolved update of that same incident. The evaluation's
local-exposure label is a 6 m radius; confirmations before that radius are labelled
early and have zero clipped local-exposure latency. The full results retain the
timestamps, early flags, source uncertainty and recovery counts. Ground truth is
used only for recording/evaluation.

The dense forest preserves the original patrol/source area and adds surrounding
geometry. Running a mission there tests the larger scene and its sensor/rendering
load; the current route does not cover every part of the 70 × 70 m footprint.

Failed cohorts remain in the ignored local `log/` directory. In particular, the
sparse rover negative patrol stopped on expired IR after 18/21 visits during
viewer changes, and the first dense GPU sensor-server trial failed before START
with an Ogre2/D3D12 texture-copy exception. Neither is counted as accepted. WSL
visible launches now retain software-rendered Ogre2 sensors and use a separate
GPU Ogre viewer. The user confirmed smoother movement; the dense drone server
measured approximately 0.86–0.99 real-time factor with this setup. Camera
resolution/rates, physics and motion limits were unchanged.
