# <mtm>/stylus_state: one VerseGrip packet (decision 18). Published when the pen sends new
# data (~50 Hz), stamped on receipt by the bridge.

std_msgs/Header header
geometry_msgs/Quaternion orientation
bool[] buttons                  # a, b, c, in that order
float64 grip                    # Hall-sensor grip value as the Haply service reports it
float64 battery_level           # 0..1
