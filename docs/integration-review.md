# Integration review — 4 November 2026

## Scope and branch history

I reviewed the repository documentation against the implementation and fetched every remote branch. The starting main commit was `921cdf8`. These teammate branches contained additional work and are now ancestors of local main:

| Branch | Reviewed head | Integrated capability |
| --- | --- | --- |
| feature/nav2-goal-adapter | bf05d73 | Navigation and launch updates |
| feature/waypoint-patrol-manager | 4bafc0a | Action feedback, retry policy, return completion, rover tuning and dashboard |
| feature/perception-validation | 001d7a5 | Radiometric IR, CO2, persistence, stream health and evaluation |
| feature/search-source-estimate | fac37e9 | Already included in the starting main |

The required deployment includes standalone rover and standalone drone missions, as well as the coordinated demonstration. The aircraft remains a Gazebo velocity-controlled model. Standalone aerial search uses the same local-evidence estimator and bounded search policy, with aerial goals executed by its flight controller rather than a ground Nav2 server.

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
- A search timeout now returns control to patrol. A dismissed anomaly must clear before it can trigger another investigation.
- Coordinated RETURN waits for both vehicles. Standalone aerial RETURN completes from the aircraft's measured home position.
- RESET and the dashboard's explicit E_STOP_RELEASE both return a latched stop to IDLE. Neither resumes motion.
- Real action-server tests replace checks that only asserted the ROS process remained alive.

## Validation evidence

- Full five-package Humble/Fortress build passed (`log/integration-build.log`).
- Full ROS test suite passed: **45 results, zero errors or failures**, including
  all 79 pure perception assertions (`log/integration-final-tests.log`).
- Native camera capture reproduced the invisible-decoy defect. Fortress logged
  unsupported cone geometry. Supported flame volumes now yield 16,159 qualifying
  flame-colour pixels in a 720 x 480 rover image, without changing detector thresholds
  (`log/camera-debug-spheres-result.log`). The first mesh experiment rendered white
  and was discarded; failed captures and trials remain in `log/`.
- The first recorded campaign stopped at the visual-decoy acceptance check. A fresh
  complete campaign is running with the repaired geometry; mode-specific motion
  checks and autonomous source-localisation acceptance are still in progress.

Final runtime results are recorded here as acceptance runs complete. Generated compiler logs, recordings and test output remain under ignored `log/`; they must not be confused with source-controlled evidence or physical robot validation.
