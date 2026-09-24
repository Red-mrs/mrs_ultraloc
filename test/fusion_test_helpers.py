"""Helpers shared by the fusion integration tests.

Design notes that matter when changing these:

* Every test uses its own UWB:UVDAR pair from test/config/test_params.yaml and
  filters published targets by id. That makes the tests independent of each
  other and of ordering, and immune to a real fusion node running on the same
  machine (which would publish on different topics anyway).
* Inputs are published continuously rather than once. The node keeps the newest
  bearing and range and gates on their age, so a single message would let the
  test's assertions depend on how long the test took to get around to checking.
* Geometry is asserted against the expected values recomputed here from the same
  inputs, not against hard-coded numbers, so the test says what the node should
  do rather than restating what it currently does.
"""

import math
import threading
import time

from rclpy.qos import QoSProfile, ReliabilityPolicy

from uvdar_core.msg import BearingObservation, BearingObservationArrayStamped
from uwb_driver.msg import UwbRange, UwbRangeStamped
from mrs_ultraloc.msg import FusionTargetArrayStamped

BEARING_TOPIC = "/test_fusion/bearing"
UWB_TOPIC = "/test_fusion/uwb"
OUTPUT_TOPIC = "/test_fusion/targets"

# Must match test/config/test_params.yaml.
BEARING_TIMEOUT_SEC = 0.5
RANGE_TIMEOUT_SEC = 0.8
PUBLISH_RATE_HZ = 20.0
BEARING_SIGMA_RAD = 0.02
RANGE_SIGMA_M = 0.30
MIN_RANGE_M = 0.3
MAX_RANGE_M = 50.0

# Test frames are named so a wrong or missing frame_id is obvious in a failure.
FRAME = "test_camera_optical_frame"

# position = bearing * range is a scaling of a unit vector, so any disagreement
# with the expectation is a bug rather than accumulated float error. Kept far
# above double rounding and far below anything a wrong formula could produce.
EXPECTED_TOLERANCE = 1e-9

# Reliable, matching the fusion node's input QoS. A best-effort publisher would
# silently drop the occasional message without affecting these assertions, but
# matching the real producer is the point of an integration test.
INPUT_QOS_DEPTH = 20
OUTPUT_QOS_DEPTH = 50


def unit_bearing(azimuth_rad, elevation_rad):
    """A unit bearing for an azimuth from +x and an elevation out of the xy plane.

    Used instead of hand-written vectors so the tests can name the geometry they
    care about. elevation_rad drives the z component, which is what the ROS 1
    fusion's sqrt(d^2 - x^2 - y^2) could not represent: it forced z non-negative.
    """
    x = math.cos(elevation_rad) * math.cos(azimuth_rad)
    y = math.cos(elevation_rad) * math.sin(azimuth_rad)
    z = math.sin(elevation_rad)
    return (x, y, z)


def tangent_covariance(bearing, sigma_rad):
    """Rank-2 covariance of a unit bearing: sigma^2 over the plane normal to it."""
    s = sigma_rad ** 2
    return [[s * ((1.0 if i == j else 0.0) - bearing[i] * bearing[j]) for j in range(3)] for i in range(3)]


def expected_position_covariance(bearing, range_m, bearing_covariance, range_sigma_m):
    """P = r^2 * P_bearing + sigma_r^2 * b b^T, the propagation the node documents."""
    radial = range_sigma_m ** 2
    return [[(range_m ** 2) * bearing_covariance[i][j] + radial * bearing[i] * bearing[j]
             for j in range(3)] for i in range(3)]


def flatten(matrix):
    return [matrix[i][j] for i in range(3) for j in range(3)]


def max_abs_diff(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


class FusionDriver:
    """Publishes synthetic inputs at known geometry and records fused output.

    `spin` is threaded because these tests block waiting for output; a single
    threaded spin in the test body would never deliver the subscription callback.
    """

    def __init__(self, node):
        self.node = node
        self._lock = threading.Lock()
        self._batches = []

        reliable = QoSProfile(depth=INPUT_QOS_DEPTH, reliability=ReliabilityPolicy.RELIABLE)

        self.bearing_pub = node.create_publisher(BearingObservationArrayStamped, BEARING_TOPIC, reliable)
        self.range_pub = node.create_publisher(UwbRangeStamped, UWB_TOPIC, reliable)
        node.create_subscription(FusionTargetArrayStamped, OUTPUT_TOPIC, self._on_output,
                                 QoSProfile(depth=OUTPUT_QOS_DEPTH, reliability=ReliabilityPolicy.RELIABLE))

    def _on_output(self, msg):
        with self._lock:
            self._batches.append(msg)

    # ---- input side ----------------------------------------------------------

    def publish_bearings(self, entries, own_frame=FRAME):
        """entries: iterable of (signal_id, bearing_vector, covariance_or_None, predicted)."""
        batch = BearingObservationArrayStamped()
        batch.header.stamp = self.node.get_clock().now().to_msg()
        batch.header.frame_id = own_frame

        for signal_id, bearing, covariance, predicted in entries:
            observation = BearingObservation()
            observation.id = signal_id
            observation.track_id = abs(signal_id)
            observation.bearing.x, observation.bearing.y, observation.bearing.z = bearing
            if covariance is not None:
                observation.covariance = list(covariance)
            observation.predicted = predicted
            batch.observations.append(observation)

        self.bearing_pub.publish(batch)

    def publish_range(self, own_address, peer_address, distance, direction="initiator"):
        """Publish a range report with `own_address` as this vehicle's module.

        direction picks which endpoint this vehicle is, which exercises both
        branches of the node's peer-address resolution. "foreign" makes own_address
        neither endpoint, as when a range from another vehicle's module is overheard.
        """
        msg = UwbRangeStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "uwb"

        if direction == "initiator":
            initiator, responder = own_address, peer_address
        elif direction == "responder":
            initiator, responder = peer_address, own_address
        else:
            initiator, responder = 0x11, 0x22

        msg.range = UwbRange(initiator_address=initiator, responder_address=responder,
                             own_address=own_address, distance=distance)
        self.range_pub.publish(msg)

    # ---- output side ---------------------------------------------------------

    def messages(self):
        with self._lock:
            return list(self._batches)

    def marks(self):
        """A cursor into the recorded messages, for counting what arrives later."""
        return len(self.messages())

    def targets_since(self, mark, target_id=None):
        """Every target seen since `mark`, optionally narrowed to one signal id."""
        found = []
        for msg in self.messages()[mark:]:
            for target in msg.targets:
                if target_id is None or target.id == target_id:
                    found.append((msg, target))
        return found

    def newest_target(self, mark, target_id):
        matches = self.targets_since(mark, target_id)
        return matches[-1] if matches else None

    def count_since(self, mark, target_id=None):
        return len(self.targets_since(mark, target_id))

    def frames_since(self, mark):
        return {msg.header.frame_id for msg in self.messages()[mark:]}


class SteadyFeed:
    """Keeps one or more targets fresh for as long as a test needs them.

    The node gates on per-sensor age, so a test that publishes once and then
    asserts would be racing the timeouts. This runs the publishing in a thread and
    is used as a context manager so the feed is definitely stopped when a test
    needs to check that output stopped.
    """

    def __init__(self, driver, period_sec=0.02):
        self.driver = driver
        self.period = period_sec
        self._stop = threading.Event()
        self._thread = None
        self._supplier = None

    def start(self, supplier):
        """supplier() is called every period to produce the current inputs."""
        self._supplier = supplier
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def _run(self):
        while not self._stop.is_set():
            self._supplier()
            time.sleep(self.period)

    def stop(self):
        if self._thread is None:
            return self
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._thread = None
        return self

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.stop()


def wait_for(predicate, timeout_sec, period_sec=0.02):
    """Poll `predicate` until true or the timeout expires. Returns whether it held."""
    end = time.time() + timeout_sec
    while time.time() < end:
        if predicate():
            return True
        time.sleep(period_sec)
    return predicate()
