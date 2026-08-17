# Development roadmap

## Definition of the baseline

The repository provides a complete simulation shell and runnable node scaffolding. “Outstanding” means the team must develop, integrate or validate it on the target ROS machine before claiming the final system is complete.

## Priority 0: first-machine acceptance

**Suggested owner:** integration lead  
**Estimate:** 0.5–1 day

- Install the declared dependencies on Ubuntu 22.04.
- Run `colcon build --symlink-install`.
- Correct any distro-specific package/API differences.
- Launch Gazebo and confirm `/husky1/odometry`, `/husky1/scan` and `/husky1/camera/image` update.
- Record exact installation fixes in the README.

**Acceptance:** clean build from a fresh clone and a screenshot or log of all three sensor topics.

## Priority 1: Nav2 goal adapter

**Suggested owner:** navigation member  
**Estimate:** 3–5 days

- Create `nav2_goal_adapter_node.cpp` from the instructions in `embertrace_core/src/README.md`.
- Subscribe to `/embertrace/navigation_goal`.
- Send `nav2_msgs/action/NavigateToPose` goals to `/husky1/navigate_to_pose`.
- Publish goal status or expose it to the mission manager.
- Cancel on `RETURN`, `PAUSED` and `EMERGENCY_STOP`.
- Reject stale or out-of-bounds goals.

**Acceptance:** goal marker appears, Husky reaches it without collision, and E-stop cancels movement within one second.

## Priority 2: waypoint patrol manager

**Suggested owner:** autonomy member  
**Estimate:** 2–4 days

- Load waypoints from `patrol_waypoints.yaml`.
- Send them in sequence while state is `PATROL`.
- Pause the patrol when mission state changes to investigation.
- Resume at the nearest sensible waypoint after a dismissed false alarm.
- Return to the base waypoint on `RETURN`.

**Acceptance:** the rover completes two loops and can leave/resume patrol cleanly.

## Priority 3: search and source estimate

**Suggested owner:** decision-making member  
**Estimate:** 4–6 days

- Replace the baseline one-step surge/cast rule with a documented algorithm.
- Maintain a bounded sample history.
- Estimate source location and uncertainty without reading ground truth.
- Constrain goals to known free space.
- Define signal-loss and timeout behaviour.

**Acceptance:** median source error below 2.0 m across at least ten seeded trials in the standard scenario.

## Priority 4: perception validation

**Suggested owner:** perception member  
**Estimate:** 3–5 days

- Tune smoke/temperature filters and thresholds using recorded trials.
- Measure false-positive and detection rates.
- Add dropout/staleness handling.
- Decide whether the simple visual verifier is sufficient or replace it with an OpenCV/ML method.
- Ensure vision cannot confirm a fire without smoke or heat evidence.

**Acceptance:** report confusion matrix and latency over at least three scenarios.

## Priority 5: operator interface refinement

**Suggested owner:** UI member  
**Estimate:** 2–4 days

- Add a 2D map panel or embed RViz if time permits.
- Show connection health and stale-data warnings.
- Require confirmation for reset/return if appropriate.
- Add alert acknowledgement and mission event logging.
- Improve layout without changing the interface contract.

**Acceptance:** a new user can start, pause, return and stop the mission without using a terminal.

## Priority 6: testing and final evidence

**Suggested owner:** testing/documentation member  
**Estimate:** ongoing, then 3 days

- Add unit tests for perception thresholds and state transitions.
- Create deterministic launch tests using fixed random seeds.
- Record mission time, source error, path length, false alarms and recovery rate.
- Run at least ten repetitions of the main trial.
- Record known failure modes rather than hiding failed runs.

## Stretch goals

- dynamic wind changes;
- two fire sources;
- false plumes from dust or machinery;
- battery-aware return to base;
- real camera classifier;
- aerial scout feeding initial search bounds;
- terrain slopes and wheel slip;
- bag-file replay mode for repeatable perception testing.

