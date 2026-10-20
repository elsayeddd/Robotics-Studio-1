std_msgs/Header header
bool environment_fresh
bool visual_fresh
bool confirmation_candidate
uint64 rejected_samples
uint32 consecutive_samples
bool co2_valid
bool thermal_fresh
# Wind classification is a perception output; deployment policy consumes it separately.
uint8 WIND_UNKNOWN=0
uint8 WIND_DRONE_SAFE=1
uint8 WIND_ROVER_ONLY=2
uint8 WIND_UNSAFE=3
uint8 wind_condition
