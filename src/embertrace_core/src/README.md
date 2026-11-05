# C++ node development notes

Source headers and inline comments explain the runtime responsibilities and failure behaviour.
The current integration is described in docs/completion-handover.md. Keep one
executable per node; pure policy/numerical libraries also run without ROS.

## Recommended ownership

| File | Purpose | Suggested owner |
| --- | --- | --- |
| `plume_model.cpp` | Ground-truth mathematical environment | Simulation member |
| `environment_simulator_node.cpp` | Robot-position sampling and raw readings | Simulation member |
| `camera_verifier_node.cpp` | Visual fire evidence | Perception member |
| `perception_node.cpp` | Filtering and evidence fusion | Perception member |
| `mission_manager_node.cpp` | Authoritative decision state machine | Autonomy member |
| `search_planner_node.cpp` | Source estimation and map-safe search goals | Autonomy member |
| `heatmap_node.cpp` | RViz operator visualisation | UI/testing member |
| `patrol_manager_node.cpp` | Waypoint patrol and return goals | Autonomy member |
| `nav2_goal_adapter_node.cpp` | Nav2 action bridge and result status | Navigation member |

## Nav2 adapter baseline

The adapter subscribes to `/embertrace/navigation_goal`, transforms goals into
`husky1_map`, and sends them to `/husky1/navigate_to_pose` when Nav2 is ready.
It cancels goals on mission-state changes or when motion is disallowed, rejects
stale and out-of-bounds goals, and publishes result strings on
`/embertrace/navigation_status`. The patrol manager uses action results together
with measured TF proximity in the configured route frame, retries bounded failures
and announces return completion. Native mock-action and waypoint regressions pass;
five standard-forest rover fire missions have resolved their sources and returned.
Full two-loop negative patrol results are tracked in
`docs/validation/2026-11-05/README.md`.

## Priority 3: search and source estimate

The search planner keeps at most 160 spatially separated environmental samples. It
fits candidate source positions on a 0.5 m grid, then refines the best candidate
at 0.1 m resolution. For each candidate it removes the unknown source strength
from the logarithmic thermal decay and wind-aligned Gaussian plume equations,
then scores the remaining spatial residuals. It publishes the best candidate
on `/embertrace/source_estimate` as
`geometry_msgs/msg/PoseWithCovarianceStamped` in the sample frame. The x/y
uncertainty includes likelihood spread and displacement of its mean from the
reported peak, with a 0.5 m standard deviation floor; it does not include a measured SLAM error. The estimator never reads
`source_x` or `source_y`. The field-shape values in
`embertrace.yaml` are tunable model assumptions, not source coordinates.

When smoke is present, a sufficiently constrained estimate guides short
investigation goals. Before enough evidence exists, the rover probes upwind.
On signal loss it casts on alternating sides of the last detection with a
bounded increasing sweep. Goal publication stops after 20 s without smoke or
120 s of searching. A reading older than 3 s is ignored. During `VERIFY`,
the estimate continues updating, but the planner issues no motion goal. For the rover, no goal
is published until `/husky1/map` and the odom-to-map transform are available.
Aerial goals use the configured bounded flight layer and the local LiDAR stop instead.
The map check rejects unknown/occupied cells and requires 0.8 m robot
clearance. It validates the endpoint; Nav2 remains responsible for route
planning and live obstacle avoidance. If an initial probe has no clearance-safe cell within the normal 2 m snap radius, search can use a surveyed detour within 4 m of that target. Probe goals stay at least 1.5 m from the robot so Nav2 can gather another view before stopping. Unknown/occupied cells remain excluded. Search timeouts only withhold planner
goals; mission-state timeout policy belongs to the mission manager.

The full launch uses only a recent estimate with at most 4 m standard deviation
per axis for the resolved alert location.
The planner uses the same configured uncertainty gate and continues probing while
the fit is too broad. An unresolved confirmed alert retains the original
confirmation sample location and sensor evidence. The uncertainty remains available
on `/embertrace/source_estimate`.

Build and run the focused unit tests from the repository root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to embertrace_core
colcon test --packages-select embertrace_core --ctest-args -R test_search_estimation
colcon test-result --verbose
```

`test_search_estimation` checks bounded history, invalid readings, stationary
samples, unknown/occupied/rotated maps, source localisation over ten seeded
standard-field trials with the baseline perception filter, a fixed survey route
with moved sources and changed plume parameters, and translation of the same
evidence to another location. The seeded test
checks median error below 2.0 m, but it supplies a controlled track through
the plume. Separate strict Gazebo missions now exercise the Nav2 adapter and
patrol manager; see `docs/validation/2026-11-05/README.md` for actual mission
coverage and measured errors.

For a runtime check after building, launch with `use_nav2:=true` so SLAM
publishes `/husky1/map`, select the rover with `wind_speed_mps:=7.0`, then send START:

```bash
ros2 launch embertrace_bringup embertrace_demo.launch.py use_nav2:=true wind_speed_mps:=7.0
ros2 service call /embertrace/operator_command embertrace_interfaces/srv/OperatorCommand "{command: 'START'}"
ros2 topic echo /embertrace/source_estimate
ros2 topic echo /embertrace/navigation_goal
```

Expect source estimates after at least five separated samples, covariance in
square metres, and search goals stamped in `husky1_map`. With
`use_nav2:=false` there is no SLAM map, so estimates may still appear while
navigation goals are withheld. Move out of the plume and check that goals
cease after `signal_loss_timeout_s`; pause or switch to `VERIFY` to check
that motion goals stop immediately. With `use_nav2:=true`, inspect
`/embertrace/navigation_status` and verify that cancellation stops motion.
