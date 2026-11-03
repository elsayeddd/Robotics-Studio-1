# ROS interface contract

Treat this document as the integration agreement. Discuss changes before renaming a topic, service, frame or field.

## Topics

| Name | Type | Publisher | Subscribers | Purpose |
| --- | --- | --- | --- | --- |
| `/husky1/odometry` | `nav_msgs/msg/Odometry` | Gazebo bridge | Environment simulator, localisation | Raw simulated odometry |
| `/husky1/odom` | `nav_msgs/msg/Odometry` | EKF | Nav2 and future consumers | Filtered robot odometry |
| `/husky1/map` | `nav_msgs/msg/OccupancyGrid` | SLAM Toolbox | Search planner, Nav2, RViz | Known free cells for goal selection |
| `/husky1/scan` | `sensor_msgs/msg/LaserScan` | Gazebo bridge | SLAM/Nav2/RViz | Obstacle perception |
| `/husky1/camera/image` | `sensor_msgs/msg/Image` | Gazebo bridge | Camera verifier/RViz | RGB image |
| `/embertrace/raw_environment` | `embertrace_interfaces/msg/EnvironmentalReading` | Environment simulator | Perception | Noisy local simulated sensors |
| `/embertrace/visual_fire_cue` | `std_msgs/msg/Float32` | Camera verifier | Perception | Visual confidence from 0 to 1 |
| `/embertrace/environment` | `embertrace_interfaces/msg/EnvironmentalReading` | Perception | Mission, search, UI, heatmap | Filtered/fused evidence |
| `/embertrace/mission_state` | `embertrace_interfaces/msg/MissionState` | Mission manager | Search, UI, adapter | Authoritative state |
| `/embertrace/navigation_goal` | `geometry_msgs/msg/PoseStamped` | Search planner | Future Nav2 adapter, RViz | Next short map-frame investigation goal in known free space |
| `/embertrace/source_estimate` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Search planner | Mission manager, RViz | Source position and x/y covariance from local sensor history; sample frame |
| `/embertrace/fire_alert` | `embertrace_interfaces/msg/FireAlert` | Mission manager | UI, logging | Confirmed or suspected alert |
| `/embertrace/sample_markers` | `visualization_msgs/msg/MarkerArray` | Heatmap | RViz | Colour-coded sample history |

## Service

### `/embertrace/operator_command`

Type: `embertrace_interfaces/srv/OperatorCommand`

Accepted command strings:

| Command | Intended behaviour |
| --- | --- |
| `START` | Begin or restart the patrol from idle |
| `PAUSE` | Hold the mission without clearing evidence |
| `RESUME` | Resume the state stored before pause |
| `RETURN` | Request return to base |
| `RESET` | Clear alert/emergency state and return to idle |
| `E_STOP` | Latch emergency stop immediately |

Unknown commands must be rejected with `accepted=false` and an explanatory message.

## Frames

| Frame | Meaning |
| --- | --- |
| `husky1_map` | SLAM/Nav2 global frame |
| `husky1_odom` | Locally continuous odometry frame |
| `husky1_base_link` | Robot body frame |
| `husky1_base_scan` | LiDAR frame |
| `husky1_camera_link` | Camera frame |

Environmental sample poses and source estimates use `husky1_odom`. Fortress OdometryPublisher reports world positions; the simulation EKF preserves that origin by fusing x/y/yaw. Both vehicle environmental streams use this shared simulation frame. Patrol waypoints are in that frame, not the arbitrary SLAM map origin. The search planner transforms candidate goals into `husky1_map` before checking the occupancy map and publishing them. Consumers must use the message header frame rather than assuming all poses share a frame.



## Added perception and aerial integration interfaces

| Topic | Type | Purpose |
| --- | --- | --- |
| `/embertrace/visual_cue` | `embertrace_interfaces/msg/VisualCue` | Timestamped valid rover camera cue; legacy Float32 still published |
| `/embertrace/perception_status` | `embertrace_interfaces/msg/PerceptionStatus` | Environment/vision freshness, confirmation candidate and rejection counts |
| `/embertrace/sensors_ready` | `std_msgs/msg/Bool` | Required rover sensor freshness |
| `/husky1/autonomy_cmd_vel` | `geometry_msgs/msg/Twist` | Nav2/teleop input to sole rover command guard |
| `/husky1/cmd_vel` | `geometry_msgs/msg/Twist` | Guarded rover command to Gazebo |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Per-stream vehicle sensor freshness |
| `/drone1/odometry` | `nav_msgs/msg/Odometry` | Scout pose in shared `husky1_odom` |
| `/drone1/cmd_vel` | `geometry_msgs/msg/Twist` | Guarded aerial sweep/return velocity |
| `/drone1/imu` | `sensor_msgs/msg/Imu` | Scout inertial data |
| `/drone1/scan` and `/drone1/scan/points` | `LaserScan` / `PointCloud2` | Scout obstacle scan and cloud |
| `/drone1/camera/image` | `sensor_msgs/msg/Image` | Downward RGB image |
| `/drone1/camera/depth/image` and `/drone1/camera/depth/points` | `Image` / `PointCloud2` | Downward depth data |
| `/{husky1,drone1}/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB-D calibration |
| `/drone1/raw_environment` | `embertrace_interfaces/msg/EnvironmentalReading` | Independent noisy altitude-attenuated scout samples |
| `/drone1/visual_cue` | `embertrace_interfaces/msg/VisualCue` | Independent stamped scout visual evidence |
| `/drone1/environment` | `embertrace_interfaces/msg/EnvironmentalReading` | Independent filtered scout evidence; investigation cue only |
| `/drone1/perception_status` | `embertrace_interfaces/msg/PerceptionStatus` | Scout perception freshness |
| `/drone1/sensors_ready` | `std_msgs/msg/Bool` | Required scout sensor freshness |
| `/{embertrace,drone1}/validation/truth` | `embertrace_interfaces/msg/TrialTruth` | Evaluation-only local observability/dropout label; autonomy never subscribes |

`VisualCue` has a header, confidence and validity. `PerceptionStatus` has a
publication header, environment/visual freshness, confirmation candidate,
rejected sample count and consecutive fresh sample count. EnvironmentalReading
retains its existing fields and now preserves the raw acquisition timestamp.

All vehicle transforms share `/husky1/tf` and `/husky1/tf_static` for this
simulation. Drone frames are `drone1_base_link` and `drone1_camera_link`.
The drone's environmental pose is its measured position, not a source estimate.
The aerial hint is checked for acquisition age and never supplies fire ground
truth to the rover. The source estimator continues using rover samples only.

## October 20 full perception sensors

- `/husky1/thermal/image`, `/drone1/thermal/image`: SensorDataQoS Image, radiometric
  mono16 / 16UC1, 0.01 K per unit. Not an RGB display image.
- `/embertrace/thermal_cue`, `/drone1/thermal_cue`: SensorDataQoS ThermalCue with
  acquisition header, explicit validity, persistent hotspot, peak C and cluster data.
- EnvironmentalReading now also includes explicit CO2 concentration/validity/detection
  and IR validity/hotspot/peak. Scalar `temperature_c` stays the local field measurement.
- PerceptionStatus includes CO2 validity, IR freshness and wind classification constants.
- Full launch requires CO2-valid raw data and fresh IR before confirmation. Legacy
  recordings and explicit reduced debug mode remain separate from C1 acceptance.
- Message changes require rebuilding every dependent package and re-sourcing all shells.
