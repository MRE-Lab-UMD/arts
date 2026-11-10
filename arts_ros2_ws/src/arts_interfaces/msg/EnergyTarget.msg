# /esu/target: a request for energy (decision 18). Address it by arm; the esu node resolves
# which ESU output feeds that arm from the arms' tool_info. Addressing by output is for bench tests.
# activate=true is a hold-to-run request: keep publishing it (>= 10 Hz) while energy is wanted;
# the esu node switches the output off when it goes stale or activate=false arrives.

std_msgs/Header header
string arm                      # "psm_left" | "psm_right"; empty to address an output directly
uint8 output                    # EnergyCommand.MONOPOLAR | BIPOLAR; used only when arm is empty
uint8 mode                      # EnergyCommand.CUT | COAG (BLEND is refused; ignored for BIPOLAR)
bool activate
uint8 source                    # EnergyCommand.SOURCE_*
