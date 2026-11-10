# /esu/command: what the esu node sent to the ESU-TAP board, after its checks (decision 18).
# Published when a channel is switched, not on every lease renewal. The board's pedals never
# appear here: they act on the board directly and are seen in /esu/state.

std_msgs/Header header
uint8 DEACTIVATE=0
uint8 ACTIVATE=1
uint8 action

uint8 MONOPOLAR=0
uint8 BIPOLAR=1
uint8 output                    # the ESU output this command drives

uint8 CUT=0
uint8 COAG=1
uint8 BLEND=2                   # never sent: blend is chosen on the ESU front panel and fires on CUT
uint8 mode                      # CUT | COAG for MONOPOLAR; COAG for BIPOLAR (one contact)

uint8 SOURCE_SOFTWARE=0
uint8 SOURCE_FOOTSWITCH=1
uint8 source                    # who asked for it (EnergyTarget.source)

string arm                      # the arm the output was resolved to ("" when addressed by output)
builtin_interfaces/Duration expiry   # the board drops the command this long after its last renewal
