# Operator-facing fire alert. The estimate must be generated from robot evidence,
# never copied from simulation ground truth.

uint8 INFO=0
uint8 WARNING=1
uint8 CRITICAL=2

std_msgs/Header header
uint8 severity
geometry_msgs/PoseStamped estimated_source
float32 confidence
bool confirmed
string evidence
string recommended_action

