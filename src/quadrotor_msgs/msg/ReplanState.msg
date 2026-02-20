# all information for a replan

std_msgs/Header header

int16 state
float64[] ini_state
nav_msgs/Odometry target
quadrotor_msgs/OccMap3d occmap

# other temporal variables
float64[] path
builtin_interfaces/Time replan_stamp