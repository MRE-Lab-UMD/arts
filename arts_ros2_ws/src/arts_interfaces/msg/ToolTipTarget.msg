# The one way to move an arm (decision 18): <arm>/tool_tip_target.
# Every reference source writes it: teleop follower, script, policy follower.
# A quick test is one message with only the pose set: the arm moves there, jaw unchanged.

std_msgs/Header header          # frame_id: the arm's base frame (<arm>_base_link) or empty for it;
                                # stamp: when the target was produced (traces a command to its cause)
geometry_msgs/Pose pose         # tool-tip pose target, metres
geometry_msgs/Twist velocity    # tool-tip feedforward, m/s and rad/s; used only if velocity_valid
float64 jaw                     # jaw opening target, rad (0 = closed); used only if jaw_valid
bool velocity_valid             # false: zero feedforward (the controller still reaches the pose)
bool jaw_valid                  # false: the jaw holds. Flags, not zero defaults: jaw 0 means closed.
