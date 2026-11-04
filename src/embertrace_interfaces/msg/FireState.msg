# Simulation state; consumed by sensor simulators and the evaluation recorder only.
uint8 HEALTHY=0
uint8 BURNING=1
uint8 BURNT=2
std_msgs/Header header
string tree_name
uint8 state
bool rendered
