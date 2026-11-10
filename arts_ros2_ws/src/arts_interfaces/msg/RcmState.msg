# <arm>/rcm_state (every control cycle): the remote centre of motion as the servo controller
# holds it (decision 17 task layer, decision 18 names).
#
# deviation is the KINEMATIC RCM deviation: the minimum distance from `point` to the
# INFINITE shaft line, from forward kinematics of the MEASURED joints. Same definition as the
# open laparoscopic platform's reported metric, so the two numbers compare directly. It is a
# self-consistency measure (shaft vs the commanded point), not the distance to the physical
# cannula; that needs the calibrated cannula and is reported separately.

std_msgs/Header header          # frame_id = <arm>_base_link; stamp = the control cycle
bool enabled                    # the RCM constraint is active in the task-layer QP
geometry_msgs/Point point       # the RCM point in header.frame_id; NaN when none is set
float64 deviation               # m, point to shaft line (measured joints); NaN when no point
float64 insertion_depth         # m, tip beyond the point along the shaft (negative: not inserted)
bool constraint_ok              # false when the QP failed this cycle and the arm is holding
