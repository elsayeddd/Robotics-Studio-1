# One environmental sample taken at the robot's current pose.
# Raw and filtered messages share this type; consumers distinguish them by topic.

std_msgs/Header header
geometry_msgs/PoseStamped sample_pose

float32 smoke_ppm
float32 temperature_c
float32 wind_speed_mps
float32 wind_direction_rad

bool smoke_detected
bool temperature_anomaly
float32 visual_confidence
float32 fused_confidence


# Explicit simulated CO2: independent background/enrichment/noise, not renamed smoke.
float32 co2_ppm
bool co2_valid
bool co2_detected
# IR image evidence stays separate from the scalar local temperature used for source fitting.
bool thermal_valid
bool thermal_hotspot
float32 thermal_peak_c
