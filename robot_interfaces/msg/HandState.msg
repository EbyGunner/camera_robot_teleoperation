# robot_interfaces/msg/HandState.msg
bool detected
string hand_type  # "left" or "right"
geometry_msgs/Point position  # x,y,z (z=0 for 2D)
bool is_closed