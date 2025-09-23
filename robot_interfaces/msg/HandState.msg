# HandState.msg

# Detection status
bool detected

# Hand type (left or right)
uint8 HAND_TYPE_LEFT=0
uint8 HAND_TYPE_RIGHT=1
uint8 hand_type

# 3D position relative to initial detection point
# x: horizontal movement (-left, +right)
# y: vertical movement (-down, +up)  
# z: distance/size scaling (1.0 = initial size)
geometry_msgs/Point position

# 3D orientation relative to initial hand orientation
# Represented as quaternion for smooth interpolation
geometry_msgs/Quaternion orientation

# Hand state (open/closed)
bool is_closed