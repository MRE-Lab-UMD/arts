# <arm>/tool_info (latched): which instrument is mounted and which model files describe it.
# Set by the bringup's <psm>_tool argument (decision 13); read by the esu node to route energy.

std_msgs/Header header
string name                     # catalog key, e.g. "large_needle_driver"; "none" for an empty adapter
string part_number              # manufacturer part number, from the instrument label; empty if unknown
string family                   # e.g. "da_vinci_x"
uint8 ENERGY_NONE=0
uint8 ENERGY_MONOPOLAR=1
uint8 ENERGY_BIPOLAR=2
uint8 energy_type
string[] joint_names            # instrument joints present in <arm>/joint_states
string[] actuator_names         # discs present in <arm>/tool_actuator_states
string kinematics_file          # package path of the DH + coupling file in use; empty if none
string kinematics_sha256        # so a recording names the exact numbers it was made with
