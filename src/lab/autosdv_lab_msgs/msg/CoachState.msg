# The coach board as the pursuit planner sees it: one target, in the ground
# plane of base_link, predicted forward to header.stamp.
#
# Published by coach_tracker at a fixed rate whether or not the board is seen;
# `valid` says whether the numbers mean anything. A consumer must not act on
# range / range_rate / bearing when valid is false.

std_msgs/Header header      # stamp = time the state is predicted to; frame_id = base_link
bool    valid               # false: no accepted detection within the tracker timeout
float32 range               # m, base_link origin to board centre, ground plane
float32 range_rate          # m/s, positive = board pulling away
float32 bearing             # rad, positive = left
float32 age                 # s since the last accepted detection; -1 if there has never been one
