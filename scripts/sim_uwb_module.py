#!/usr/bin/env python3
"""Publishes UWB range reports to a target flying the scripted path.

Stands in for this vehicle's UWB module. It publishes `uwb_driver/msg/UwbRangeStamped`
on the topic the real driver uses, so the fusion node cannot tell the difference and
neither `uwb_driver` nor a module is needed.

    ros2 run mrs_ultraloc sim_uwb_module.py --ros-args \
        -p this_address:=0x1234 -p peer_address:=0x2345

Usually started through sim_fusion.launch.py, which also starts the bearing side and
the fusion node and gives both sides the same trajectory.

What is deliberately *not* simulated
------------------------------------
The serial link. The real driver opens a USB-serial device, speaks LLCP a byte at a
time, and republishes what it decoded - so a fake at the USB level would be testing
uwb_driver's byte handling rather than anything in this repository, and building one
means a kernel USB gadget. Faking at the driver's output topic tests the fusion, which
is what this is for, and leaves the driver's own behaviour untested. Worth knowing if
you go looking for coverage of the driver and find none here.

Consistency with the bearing side
---------------------------------
`distance` is the distance to the same trajectory `sim_uvdar_target.py` publishes the
bearing of, sampled at this node's own clock. The fusion pairs a bearing with a range by
their timestamps, not by wall-clock coincidence, so the two samples it combines describe
the target a few milliseconds apart. That is what two real sensors do, and it is why
nothing downstream should assume the fused `position` is where the target is at the
moment of publication. The relations that do hold are asserted in test/test_sim_pipeline.py.
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

from uwb_driver.msg import UwbRange, UwbRangeStamped  # noqa: E402

from sim_ultraloc.sim_parameters import (  # noqa: E402
    declare_float, declare_int, declare_string, declare_trajectory_parameters,
    read_float, read_int, read_rate, read_string, trajectory_from_parameters,
)
from sim_ultraloc.sim_trajectory import SmoothWiggle  # noqa: E402

# The modules report millimetres over the wire and the driver converts to metres, so
# quantising here too keeps the sim from being smoother than the thing it replaces. A
# consumer that distinguished 1.2345678 m from 1.234 m would be relying on precision the
# hardware does not have.
RANGE_QUANTISATION_M = 1e-3


class SimUwbModule(Node):
    """The range side of the simulation."""

    def __init__(self):
        super().__init__('sim_uwb_module')

        declare_string(self, 'topic', '/uav/uwb/distance')
        declare_float(self, 'publish_rate_hz', 42.0)
        declare_int(self, 'this_address', 0x1234)
        declare_int(self, 'peer_address', 0x2345)
        # The real driver declares frame_id too and defaults to exactly this. Nothing
        # in the fusion reads it, but a downstream consumer that filters on it works
        # against the sim unchanged.
        declare_string(self, 'frame_id', 'uwb')
        # 1-sigma multipath-style error on the reported range. Zero keeps the range
        # exact, which is what makes the fusion's own arithmetic checkable.
        declare_float(self, 'range_noise_sigma_m', 0.0)
        declare_trajectory_parameters(self)

        self.topic = read_string(self, 'topic')
        rate_hz = read_rate(self, 'publish_rate_hz')
        self.this_address = self._read_address('this_address')
        self.peer_address = self._read_address('peer_address')
        self.frame_id = read_string(self, 'frame_id')
        self.range_noise_sigma_m = read_float(self, 'range_noise_sigma_m')
        if self.range_noise_sigma_m < 0.0:
            raise ValueError(f'range_noise_sigma_m cannot be negative, got {self.range_noise_sigma_m}')

        if self.this_address == self.peer_address:
            # peerAddress() resolves the peer as "whichever endpoint of the exchange is
            # not us", so equal addresses leave the fusion no peer to key the range to
            # and every message is discarded with a throttled warning. Better to say so
            # at startup than to simulate that and let it look like a fusion bug.
            raise ValueError(f'this_address and peer_address must differ, both are 0x{self.this_address:04X}')

        self.trajectory = trajectory_from_parameters(self)

        # Its own wiggle, on a seed well clear of the three the trajectory uses (the
        # orbit's noise_seed plus one per axis), so range error and position error are
        # uncorrelated and do not conspire to keep the reported range matching the true
        # distance. Correlated errors would look like a better sensor than the rig is.
        self.range_noise = SmoothWiggle(
            self.range_noise_sigma_m, self.trajectory.period_sec,
            seed=self.trajectory.noise_seed + 100)

        # Reliable, because the fusion's subscription is reliable. A best-effort
        # publisher still matches a reliable subscriber in DDS, so it would appear to
        # work and then drop messages silently, which reads as fusion staleness. Depth
        # well above the driver's 1, because a shallow queue plus a busy subscriber shows
        # up as missing ranges and ruling that out is the point of a bench rig.
        self.publisher = self.create_publisher(
            UwbRangeStamped, self.topic,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))

        self.timer = self.create_timer(1.0 / rate_hz, self.publish_range)

        near, far = self.trajectory.range_bounds()
        self.get_logger().info(
            f'sim UWB module 0x{self.this_address:04X} ranging to 0x{self.peer_address:04X} '
            f'on \'{self.topic}\' at {rate_hz:g} Hz, {near:.2f}-{far:.2f} m'
            + (f', +/-{self.range_noise_sigma_m * 1000:.0f} mm multipath' if self.range_noise_sigma_m else ''))

    def _read_address(self, name):
        """A 16-bit module address.

        The wire format is two bytes and the driver assembles the address from two of
        them, so anything wider would produce a peer the fusion's id pairing can never
        mention: the range would be accepted and keyed, and simply never fused - the
        worst kind of failure, since it looks like a missing target rather than a typo.
        """
        value = read_int(self, name)
        if not 0 <= value <= 0xFFFF:
            raise ValueError(f'{name} must be a 16-bit module address, got {value}')
        return value

    def publish_range(self):
        # One clock read, used both to pick where the target is and to stamp the message.
        # Two reads would be a few microseconds apart, which is nothing on its own but
        # makes the message claim a time at which the target was not quite where the
        # message says it was - and `distance == |p(range_stamp)|` is the one exact
        # relation this rig owes anything downstream that checks it.
        #
        # This is also the clock the fusion ages the message against, and the same clock
        # the bearing side samples from, so the two agree on where the target was at a
        # given instant without ever talking to each other. With use_sim_time set both
        # follow /clock, so replay against a bag needs nothing here to know about it.
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9

        distance = self.trajectory.distance_at(t) + self.range_noise(t)
        distance = max(distance, 0.0)
        distance = round(distance / RANGE_QUANTISATION_M) * RANGE_QUANTISATION_M

        msg = UwbRangeStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        # This module rang out, so it is the initiator and the target answered. The
        # fusion takes whichever endpoint is not own_address as the peer, so which side is
        # which does not matter to it - but this is what a module that rang out reports,
        # and the driver passes the addresses through exactly as the module framed them.
        msg.range = UwbRange(initiator_address=self.this_address,
                             responder_address=self.peer_address,
                             own_address=self.this_address,
                             distance=distance)
        self.publisher.publish(msg)


def main(args=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = None
    try:
        node = SimUwbModule()
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
