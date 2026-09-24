#!/usr/bin/env python3
"""Publishes bearings to a target flying the scripted path, in place of the UVDAR chain.

Stands in for `uvdar_core`'s bearing endpoint: it publishes
`uvdar_core/msg/BearingObservationArrayStamped` on the topic the real endpoint uses, so
the fusion node cannot tell the difference. The detector, the tracker and the camera are
all absent.

    ros2 run mrs_ultraloc sim_uvdar_target.py --ros-args -p signal_id:=3

Usually started through sim_fusion.launch.py, which also starts the UWB side and the
fusion node and gives both sides the same trajectory.

Why the detector is not in the loop
-----------------------------------
The obvious version of this node renders black frames with bright dots on them and lets
the real FIMD detector, tracker and bearing endpoint produce the message. That was the
intention, and it does not work here, for two reasons found by trying:

* the detector's default `backend: gpu` cannot initialise on this machine (amdgpu fails
  to open a render node) and then publishes *nothing*, without an error, which is
  indistinguishable from a detection failure; and
* with `backend: cpu`, filled blobs of every size tried - 1 px, and 2 to 14 px at grey
  255 on black - all came back as `points_seen: []`, so getting a detection at all needs
  the blob geometry tuned against `radius_module.hpp`'s boundary and interior offsets and
  against the intrinsics in `camera/ocam.yaml`.

Both are solvable and neither is this repository's problem, so the detector is left out
and the bearing is injected where the endpoint would put it. What that does *not* cover:
detection of synthetic dots, the tracking association gate, and the cyclic Hamming
sequence matching in `signal_matcher.hpp` - none of which this node touches. A real
`id` is what the tracker decodes from a blink pattern; here it is a parameter.
"""

import glob
import os
import sys

# An installed node is run by path, so sys.path[0] is <prefix>/lib/mrs_ultraloc and this
# package's own Python directory is not on it - the sourced setup files normally put it
# there through a PYTHONPATH hook, and when that hook is present none of this is needed.
# It is here for the other case, running the script straight out of a checkout, where
# there is no prefix at all and `sim_ultraloc` is the sibling of `scripts/`.
#
# Both layouts put the module one directory up from this file: `<pkg>/scripts/` above is
# the package, and `<prefix>/lib/mrs_ultraloc/` above is `<prefix>/lib`, which is where
# ament's Python install directory lands. The glob covers whatever versioned `python3.x`
# suffix sysconfig picked, so no Python version is hard-coded.
_here = os.path.dirname(os.path.abspath(__file__))
_parent = os.path.dirname(_here)
for _candidate in [_parent] + sorted(glob.glob(os.path.join(_parent, 'python*'))):
    if os.path.isdir(os.path.join(_candidate, 'sim_ultraloc')) and _candidate not in sys.path:
        sys.path.insert(0, _candidate)
        break

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402

from uvdar_core.msg import BearingObservation, BearingObservationArrayStamped  # noqa: E402

from sim_ultraloc.sim_parameters import (  # noqa: E402
    declare_float, declare_int, declare_string, declare_trajectory_parameters,
    read_float, read_int, read_rate, read_string, trajectory_from_parameters,
)
from sim_ultraloc.sim_trajectory import OPTICAL_FRAME, flatten, tangent_covariance  # noqa: E402


class SimUvdarTarget(Node):
    """The bearing side of the simulation."""

    def __init__(self):
        super().__init__('sim_uvdar_target')

        declare_string(self, 'topic', '/uav/bearing/camera_0/observations')
        declare_float(self, 'publish_rate_hz', 60.0)
        # The id the tracker would decode from this vehicle's blinker sequence, i.e. its
        # index into `tracking.sequences`. This is the UVDAR half of the fusion's
        # uwb_uvdar_id_pairs, so it has to be mentioned there or the bearing is looked up
        # for a pair that never arrives.
        declare_int(self, 'signal_id', 0)
        # Only echoes back what the fusion ignores; carried because the real endpoint
        # fills it and a consumer that logs it would otherwise see 0 for everything.
        declare_int(self, 'track_id', 1)
        declare_string(self, 'camera_frame', OPTICAL_FRAME)
        # Angular 1-sigma of the bearing, spread over the tangent plane. Filled into the
        # message rather than left empty: an all-zero covariance reads to the fusion as
        # "no covariance", and it then substitutes its own bearing_sigma_rad, so a sim
        # that left it out would silently stop controlling the angular uncertainty it is
        # supposed to be setting.
        declare_float(self, 'bearing_sigma_rad', 0.02)
        # Force the flag the tracker raises when it is coasting through an occlusion and
        # extrapolating. Deterministic on purpose - a random flag would make a failing
        # run impossible to reproduce - and the fusion's `bearing_predicted` output is
        # otherwise unreachable.
        declare_int(self, 'predicted', 0)
        declare_trajectory_parameters(self)

        self.topic = read_string(self, 'topic')
        rate_hz = read_rate(self, 'publish_rate_hz')
        self.signal_id = read_int(self, 'signal_id')
        self.track_id = read_int(self, 'track_id')
        self.camera_frame = read_string(self, 'camera_frame')
        self.bearing_sigma_rad = read_float(self, 'bearing_sigma_rad')
        self.predicted = bool(read_int(self, 'predicted'))

        if self.bearing_sigma_rad < 0.0:
            raise ValueError(f'bearing_sigma_rad cannot be negative, got {self.bearing_sigma_rad}')

        self.trajectory = trajectory_from_parameters(self)

        # Reliable, matching the fusion's input QoS. The endpoint's own profile is not
        # the constraint here - the fusion is the only consumer that matters, and a
        # best-effort publisher would match and then drop, which reads as staleness.
        self.publisher = self.create_publisher(
            BearingObservationArrayStamped, self.topic,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))

        self.timer = self.create_timer(1.0 / rate_hz, self.publish_bearings)

        self.get_logger().info(
            f'sim UVDAR target id {self.signal_id} on \'{self.topic}\' at {rate_hz:g} Hz, '
            f'frame \'{self.camera_frame}\', bearing sigma {self.bearing_sigma_rad:.3f} rad'
            + (' (flagged predicted)' if self.predicted else ''))

    def publish_bearings(self):
        # One clock read for both the geometry and the stamp, so the message never
        # claims a time at which the target was not quite where the message says it was.
        # Two reads would be microseconds apart - too small to matter physically, but it
        # is the difference between `normalize(position) == p(bearing_stamp)/|p(...)|`
        # being exact and being exact to a few millimetres, and a sim is the one place
        # where exact is free.
        #
        # This is the clock the fusion ages the message against, and the same clock the
        # UWB side samples its distance from, so the two agree on where the target was at
        # a given instant without ever talking to each other.
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9

        bearing = self.trajectory.bearing_at(t)
        if bearing is None:
            # The target is at the camera. There is no direction to report and no
            # observation to make, so publish nothing rather than a zero vector - the
            # fusion skips zero bearings, and a real tracker would have lost the target.
            return

        batch = BearingObservationArrayStamped()
        batch.header.stamp = now.to_msg()
        batch.header.frame_id = self.camera_frame

        observation = BearingObservation()
        observation.id = self.signal_id
        observation.track_id = self.track_id
        observation.bearing.x, observation.bearing.y, observation.bearing.z = bearing
        observation.covariance = flatten(tangent_covariance(bearing, self.bearing_sigma_rad))
        observation.predicted = self.predicted
        batch.observations.append(observation)

        self.publisher.publish(batch)


def main(args=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = None
    try:
        node = SimUvdarTarget()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
