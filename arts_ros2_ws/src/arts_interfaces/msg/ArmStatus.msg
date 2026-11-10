# <arm>/status (latched, on change): one status message per arm (decision 18), merging the UR
# driver's separate safety mode, robot mode, program-running and speed-scaling topics.

std_msgs/Header header
string model                    # "ur5e" | "ur5"
uint8 safety_mode               # values of ur_dashboard_msgs/SafetyMode (1 = NORMAL, 5 = SAFEGUARD_STOP, ...)
uint8 robot_mode                # values of ur_dashboard_msgs/RobotMode (7 = RUNNING, ...)
bool program_running            # the External Control program is running on the arm
float64 speed_scaling           # the UR's speed scaling, 0..1
bool controller_active          # the ARTS servo controller is active in the arm's controller manager
