# <arm>/tool_tip_state: the controller's forward kinematics of the tool tip (decision 18).
# A convenience and a recorded record; /tf carries the same pose (<arm>_tool_tip, whose
# <arm>_tool0 -> <arm>_tool_tip edge the controller publishes on /tf_static).

std_msgs/Header header          # stamp: the joint_states sample it was computed from; frame_id: <arm>_base_link
geometry_msgs/Pose pose         # pose of <arm>_tool_tip (the configured tip; tool0 + offset until
                                # the instrument chain is in the description)
geometry_msgs/Twist velocity    # m/s, rad/s
float64 jaw                     # rad, 0 = closed; NaN when the instrument has no jaw or no drive
float64 jaw_velocity            # rad/s; NaN as above
