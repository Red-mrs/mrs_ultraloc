# One array per publish tick, holding every target that had a fresh bearing and
# a fresh range at that moment. Empty arrays are not published, so the absence
# of messages means nothing was fused rather than that fusion is running.
std_msgs/Header header
mrs_ultraloc/FusionTarget[] targets
