# C++ node development notes

Each source file starts with its completed baseline and `TODO(student)` work. Keep one executable per file until the team is comfortable with ROS composition.

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
`/embertrace/navigation_status`. The patrol manager currently advances from
map-frame odometry proximity; action-result sequencing and full rover acceptance
remain to be validated on the target ROS machine.

## Priority 3: search and source estimate

The search planner keeps at most 160 spatially separated environmental samples. It
fits candidate source positions on a 0.5 m grid, then refines the best candidate
at 0.1 m resolution. For each candidate it removes the unknown source strength
from the logarithmic thermal decay and wind-aligned Gaussian plume equations,
then scores the remaining spatial residuals. It publishes the best candidate
on `/embertrace/source_estimate` as
`geometry_msgs/msg/PoseWithCovarianceStamped` in the sample frame. The x/y
covariance is a relative likelihood spread with a 0.5 m standard deviation
floor; it does not include a measured SLAM error. The estimator never reads
`source_x` or `source_y`. The field-shape values in
`embertrace.yaml` are tunable model assumptions, not source coordinates.

When smoke is present, a sufficiently constrained estimate guides short
investigation goals. Before enough evidence exists, the rover probes upwind.
On signal loss it casts on alternating sides of the last detection with a
bounded increasing sweep. Goal publication stops after 20 s without smoke or
120 s of searching. A reading older than 3 s is ignored. During `VERIFY`,
the estimate continues updating, but the planner issues no motion goal. No goal
is published until `/husky1/map` and the odom-to-map transform are available.
The map check rejects unknown/occupied cells and requires 0.8 m robot
clearance. It validates the endpoint; Nav2 remains responsible for route
planning and live obstacle avoidance. Search timeouts only withhold planner
goals; mission-state timeout policy belongs to the mission manager.

The mission manager uses only a recent estimate with less than 4 m standard deviation per axis for the alert location.
If no estimate exists, the alert evidence says the source is unresolved and the
reported pose is the latest sample location. The uncertainty remains available
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
the plume. The roadmap's full rover-trial acceptance still needs integrated
runs with the Nav2 adapter and patrol manager.

For a runtime check after building, launch with `use_nav2:=true` so SLAM
publishes `/husky1/map`, then send `START` and teleoperate through the plume:

```bash
ros2 launch embertrace_bringup embertrace_demo.launch.py use_nav2:=true
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
