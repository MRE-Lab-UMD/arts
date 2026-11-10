# /esu/info (latched): the ESU, the board, and how each output was mapped to an arm (decision 18).
# Republished whenever the board (re)connects, so a recording names the board it was made with.

std_msgs/Header header
string esu_model
string board                    # ESU-TAP board revision as the board reports it, e.g. "v2"
string board_firmware           # firmware version as the board reports it, e.g. "3.2.0-bench"
string[] channels               # the board's channels, e.g. ["cut", "coag", "bipolar"]
bool pedal_connected            # the firmware reads its pedal inputs
uint32 lease_ms                 # a software command expires this long after its last renewal
uint32 boot_id                  # the board's boot counter (0 = unknown); a change is a reboot
uint32 dropped_events           # state messages the board could not deliver since boot
uint8[] outputs                 # parallel arrays, one entry per mapped output: EnergyCommand.MONOPOLAR | BIPOLAR,
string[] arms                   # the arm it feeds,
string[] mapping_source         # and what decided it: "tool_info" | "config"
