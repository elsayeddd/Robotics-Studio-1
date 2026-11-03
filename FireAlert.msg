# Operator-facing fire alert. The estimate must be generated from robot evidence,
# never copied from simulation ground truth.

uint8 INFO=0
uint8 WARNING=1
uint8 CRITICAL=2

std_msgs/Header header
uint8 severity
geometry_msgs/PoseStamped estimated_source
float32 confidence
# False for a suspected-anomaly warning; true only after corroborated confirmation.
bool confirmed
string evidence
string recommended_action

# Sensor values are explicit so an operator need not parse the evidence text.
uint64 alert_id
uint8 platform
bool source_resolved
float32 smoke_ppm
float32 temperature_c
float32 co2_ppm
bool co2_valid
float32 thermal_peak_c
bool thermal_valid
float32 wind_speed_mps
