# Episode boundary marker for the recording pipeline.
# Published on footswitch press or software command to mark episode start/end
# in MCAP recordings. arts_datasets uses these to segment recordings into
# training episodes for LeRobot export.

std_msgs/Header header

uint8 START=0
uint8 END=1
uint8 CHECKPOINT=2
uint8 mark_type

string label                    # optional human-readable label (e.g. "peg_transfer_1")
uint32 episode_id               # monotonically increasing episode counter
