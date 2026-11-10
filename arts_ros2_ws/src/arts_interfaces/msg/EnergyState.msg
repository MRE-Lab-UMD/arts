# /esu/device_state (the board's outputs as decoded by the esu node: arms empty) and /esu/state
# (the same, with the arm each active output feeds) (decision 18, 2026-11-03 amendment).
# The board reports which generator-side contacts it drives, inferred from its pedal inputs and
# the commands it accepted; it measures neither the contacts nor RF output, and there is no
# current sensing at the instrument. The stamp is the PC's receipt time of the board's message.

std_msgs/Header header
bool monopolar_cut              # CUT contact of the monopolar output driven
bool monopolar_coag             # COAG contact of the monopolar output driven
bool bipolar                    # the bipolar output driven

uint8 SOURCE_FOOTSWITCH=0       # a pedal changed it
uint8 SOURCE_SOFTWARE=1         # a software command changed it
uint8 SOURCE_SYNC=2             # a snapshot at (re)connection, not an event
uint8 SOURCE_TAKEOVER=3         # a pedal press ended software control
uint8 source

bool software_enabled           # the board accepts software commands (cleared by any pedal press)
bool board_connected            # the board's micro-ROS session is up
bool timing_suspect             # this message arrived later than the event (see the esu node)
string[] active_arms            # the arm each active output feeds; empty in device_state
