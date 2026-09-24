# UVDAR signal id of the blinker the bearing was decoded from. This is the id
# the tracker reports, i.e. the UWB address translated through the configured
# uwb_uvdar_id_pairs.
int32 id

# UWB address of this vehicle, as reported by the ranging module that measured
# the range. Echoed so a consumer can tell which module the range came from
# when several are fitted.
uint32 own_id

# Position of the other vehicle, expressed in FusionTargetArrayStamped.header.frame_id.
geometry_msgs/Vector3 position

# Range as reported by the UWB module, in metres. This is the norm of position
# only when the bearing and the range describe the same line of sight.
float64 distance

# Unit bearing the position was built from, in FusionTargetArrayStamped.header.frame_id.
# Kept separately so a downstream estimator can use the direction and the range
# as two measurements instead of inverting this message.
geometry_msgs/Vector3 bearing

# Row-major 3x3 covariance of position, in m^2. First-order propagation of the
# bearing covariance scaled by the range, plus a radial term from the range
# noise. Rank deficient by construction: nothing constrains the direction
# perpendicular to both the bearing and the image-plane uncertainty.
float64[9] covariance

# True when the bearing came from a tracker prediction rather than a detection
# associated in that frame.
bool bearing_predicted

# Stamp of the bearing batch the bearing was taken from.
builtin_interfaces/Time bearing_stamp

# Stamp of the range message the distance was taken from.
builtin_interfaces/Time range_stamp
