# Teleop controller state, published by arts_teleop.
# Captures clutch/scaling/frame state for recording and display.

std_msgs/Header header

bool    clutch_engaged          # true = master motion maps to slave motion
bool    camera_control_active   # true = master controls camera arm instead

float64 scaling_factor          # e.g. 3.0 for 3:1 master-to-slave

# Active instrument (which arm the master is controlling)
string  active_arm              # "psm_left" | "psm_right" | "ecm"
