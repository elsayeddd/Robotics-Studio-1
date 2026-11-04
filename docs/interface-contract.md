# ROS interface contract

This is the integration contract for the November 5 snapshot. Rebuild all packages
after applying message changes. Use message frames and acquisition timestamps;
do not substitute publication time or an image hotspot for a source coordinate.

## Per-vehicle sensing

Use `/embertrace` as the rover perception root and `/drone1` as the drone root.
Images/odometry use robot roots `/husky1` and `/drone1` respectively.

| Suffix or topic | Type | Meaning |
| --- | --- | --- |
| `/{robot}/odometry` | nav_msgs/Odometry | Measured pose in shared simulation frame `husky1_odom` |
| `/{robot}/camera/image` | sensor_msgs/Image | RGB image |
| `/{robot}/thermal/image` | sensor_msgs/Image | Radiometric mono16/16UC1, 0.01 K/unit; not an RGB substitute |
| `{root}/raw_environment` | EnvironmentalReading | Independent noisy local smoke, scalar temperature, CO2 and shared wind |
| `{root}/visual_cue` | VisualCue | Acquisition header, confidence and validity |
| `{root}/thermal_cue` | ThermalCue | Acquisition header, valid persistent hotspot, peak C and cluster measures |
| `{root}/environment` | EnvironmentalReading | Filtered local readings, gas/heat flags, IR summary and fused confidence |
| `{root}/perception_status` | PerceptionStatus | Acquisition health, camera freshness, CO2 validity, confirmation candidate and rejections |
| `{root}/sensors_ready` | std_msgs/Bool | Fresh required bridge streams and raw environment |
| `/{robot}/battery` | sensor_msgs/BatteryState | Simulated fraction in percentage, with stamped availability |
| `/{robot}/patrol_loops` | std_msgs/UInt32 | Completed route loops; policy counts from the mission's start baseline |
| `/{robot}/mission_navigation_status` | std_msgs/String | RETURNED, ABORTED or TIMEOUT for the selected platform |
| `{root}/validation/truth` | TrialTruth | Evaluation-only exposure/dropout label; never an autonomy input |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | Per-stream sensor freshness |

Scalar `temperature_c` is the local environmental measurement. It is not replaced
by the camera's peak. Gas confidence uses the stronger smoke/CO2 score, avoiding
addition of correlated gas channels. RGB alone cannot confirm a fire. Legacy
Float32 visual cue topics remain for compatibility but are not fusion inputs.

## Shared mission and navigation

| Topic | Type | Meaning |
| --- | --- | --- |
| `/embertrace/wind` | geometry_msgs/Vector3Stamped | Ambient vector in `husky1_odom`; acquisition and receipt freshness required |
| `/embertrace/mission_state` | MissionState | Sole authoritative policy state, selected/preferred platform, epoch, permissions, acknowledgement |
| `/embertrace/active_environment` | EnvironmentalReading | Selected platform's local readings for search/heatmap; no unselected evidence mixing |
| `/embertrace/navigation_goal` | geometry_msgs/PoseStamped | Rover patrol/search goal; Nav2 adapter transforms into map and bounds-checks |
| `/drone1/navigation_goal` | geometry_msgs/PoseStamped | Bounded aerial investigation goal in shared odometry frame |
| `/embertrace/navigation_status` | std_msgs/String | Rover action adapter status for patrol/retry/failure handling |
| `/embertrace/source_estimate` | geometry_msgs/PoseWithCovarianceStamped | Local-history fit, covariance and sample frame; no simulator truth |
| `/embertrace/fire_alert` | FireAlert | Suspected warning or confirmed event with id/platform, sensor values and source_resolved flag |
| `/embertrace/sample_markers` | visualization_msgs/MarkerArray | Bounded selected-platform sample history for RViz |
| `/embertrace/manual_cmd_vel` | geometry_msgs/Twist | Operator input, effective only for selected platform in manual mode |
| `/husky1/autonomy_cmd_vel` | geometry_msgs/Twist | Nav2 input to rover guard |
| `/husky1/cmd_vel` | geometry_msgs/Twist | Sole guarded rover output to Gazebo |
| `/drone1/cmd_vel` | geometry_msgs/Twist | Guarded aerial controller output to Gazebo |
| `/embertrace/mission_event` | std_msgs/String | Actual policy transitions/acknowledgements, persisted by CSV logger |
| `/embertrace/fire_state` | FireState | Latched scene state; rendered=false suspends simulated raw samples |
| `/husky1/map` | nav_msgs/OccupancyGrid | SLAM occupancy map; map-safe rover goal selection and dashboard |

`confirmed=false` denotes a suspected-anomaly warning, not a confirmed fire.
A new warning is sent when an investigation begins; confirmation has its own
persistence gate. ACK applies to either kind. An alert with `source_resolved=false` locates the measured detection sample, not a
fitted fire source. Consumers and metrics must preserve this distinction. Mission
state and alerts use transient-local QoS. Environmental/image streams use sensor
QoS. Mission epochs invalidate old goals, commands, fits and confirmation evidence.

## Services and commands

`/embertrace/operator_command` uses `embertrace_interfaces/srv/OperatorCommand`.
Invalid or unsafe commands return accepted=false with a reason.

| Command | Behaviour |
| --- | --- |
| START | Select stable safe wind platform and begin patrol with fresh sensors |
| PAUSE | Stop active autonomy; retain a resume state and clear confirmation counts |
| RESUME | Re-check wind/sensors; restore same-platform phase or start newly selected platform patrol |
| RETURN | Return selected platform to measured launch pose if conditions permit |
| RESET | Stop, increment epoch and clear emergency/mission state to IDLE |
| E_STOP_RELEASE | Alias for RESET, used by the dashboard's explicit release action |
| E_STOP | Latch stop; START/RESUME/manual cannot clear it |
| ACK | Acknowledge a recorded alert without changing its sensor evidence |
| MANUAL_ON | Hold autonomy and enable bounded, selected-platform operator commands |
| MANUAL_OFF | Disable manual commands; explicit RESUME is required |

`/embertrace/set_fire_state` uses `embertrace_interfaces/srv/SetFireState` with state
healthy, burning or burnt. It is simulation-only. The scene controller and sensor
simulators use it; perception/mission/search do not read the source scene state.
Use `/mission_simulator` ROS parameters to set wind and initial battery fraction.

## Frames

All transforms are carried on `/husky1/tf` and `/husky1/tf_static`. `husky1_map`
is the SLAM frame. `husky1_odom` is the shared simulation odometry frame, based on
Fortress world-position odometry. Robot body/camera frames are `husky1_base_link`,
`husky1_camera_link`, `drone1_base_link` and `drone1_camera_link`. The dashboard
transforms poses into the actual map frame, including its origin yaw. Verify these
origins with the map/TF acceptance trial; do not assume a SLAM origin is world zero.

## Confirmed incident updates

`FireAlert.alert_id` identifies an incident. Confirmation is published immediately,
even when `source_resolved` is false. In the full launch, the selected vehicle can
remain in TRACK to gather spatial evidence for at most `localisation_timeout_s`
(default 120 seconds). A usable estimate ends that movement and updates the same
alert ID; it does not count as another fire. The original confirmation sample and
sensor values remain available in the fallback alert. If localisation expires,
the confirmed incident remains recorded with an explicitly unresolved location,
then proceeds through alert hold and return. Wind, battery, sensor and operator
motion guards retain priority. Consumers counting incidents must deduplicate IDs.

`PerceptionStatus.header` identifies the latest accepted environmental acquisition
and its frame. It is not the publication clock. This lets the mission adapter
discard late warmup/unavailable statuses after a newer valid reading has arrived.
Expiry still uses current acquisition age and monotonic receipt time.
