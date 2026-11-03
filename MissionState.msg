# Authoritative mission state published only by mission_manager_node.

uint8 IDLE=0
uint8 PATROL=1
uint8 INVESTIGATE=2
uint8 TRACK=3
uint8 RECOVER=4
uint8 VERIFY=5
uint8 ALERT=6
uint8 RETURN_TO_BASE=7
uint8 PAUSED=8
uint8 EMERGENCY_STOP=9

std_msgs/Header header
uint8 state
string state_name
string detail
float32 progress
bool motion_allowed

# Selected platform owns local sensing, source fitting and movement for this epoch.
uint8 PLATFORM_NONE=0
uint8 PLATFORM_ROVER=1
uint8 PLATFORM_DRONE=2
uint8 active_platform
uint8 preferred_platform
uint64 mission_epoch
bool manual_control
bool alert_acknowledged
