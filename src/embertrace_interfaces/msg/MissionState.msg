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

