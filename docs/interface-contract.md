# ROS interface contract

Treat this document as the integration agreement. Discuss changes before renaming a topic, service, frame or field.

## Topics

| Name | Type | Publisher | Subscribers | Purpose |
| --- | --- | --- | --- | --- |
| `/husky1/odometry` | `nav_msgs/msg/Odometry` | Gazebo bridge | Environment simulator, localisation | Raw simulated odometry |
| `/husky1/odom` | `nav_msgs/msg/Odometry` | EKF | Nav2 and future consumers | Filtered robot odometry |
| `/husky1/scan` | `sensor_msgs/msg/LaserScan` | Gazebo bridge | SLAM/Nav2/RViz | Obstacle perception |
| `/husky1/camera/image` | `sensor_msgs/msg/Image` | Gazebo bridge | Camera verifier/RViz | RGB image |
| `/embertrace/raw_environment` | `embertrace_interfaces/msg/EnvironmentalReading` | Environment simulator | Perception | Noisy local simulated sensors |
| `/embertrace/visual_fire_cue` | `std_msgs/msg/Float32` | Camera verifier | Perception | Visual confidence from 0 to 1 |
| `/embertrace/environment` | `embertrace_interfaces/msg/EnvironmentalReading` | Perception | Mission, search, UI, heatmap | Filtered/fused evidence |
| `/embertrace/mission_state` | `embertrace_interfaces/msg/MissionState` | Mission manager | Search, UI, adapter | Authoritative state |
| `/embertrace/navigation_goal` | `geometry_msgs/msg/PoseStamped` | Search planner | Future Nav2 adapter, RViz | Next short investigation goal |
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

All project-level sample poses currently use `husky1_odom`. When the Nav2 integration becomes stable, the team may transform them into `husky1_map`, but the change must be made consistently.

