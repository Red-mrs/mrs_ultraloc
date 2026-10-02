"""Integration tests for the UWB + UVDAR fusion node.

Run them either way:

    colcon test --packages-select mrs_ultraloc --event-handlers console_direct+
    colcon test-result --verbose                       # results after a colcon test
    pytest test/test_fusion.py -v                      # direct, needs the overlay sourced

The node runs as its own process on real topics with real QoS and real parameter
loading, so what these exercise is the thing that actually ships - including that
the parameter file is read at all, which was broken once and silently.

Each test uses its own UWB:UVDAR pair from test/config/test_params.yaml and
filters output by signal id, so tests neither interfere nor depend on order.
"""

import math
import os
import signal
import subprocess
import sys
import threading
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fusion_test_helpers import (  # noqa: E402
    BEARING_SIGMA_RAD, BEARING_TIMEOUT_SEC, BEARING_TOPIC, EXPECTED_TOLERANCE, FRAME, MAX_RANGE_M,
    OUTPUT_TOPIC, PUBLISH_RATE_HZ, RANGE_SIGMA_M, RANGE_TIMEOUT_SEC, UWB_TOPIC, FusionDriver,
    SteadyFeed, expected_position_covariance, flatten, max_abs_diff, mean_direction,
    tangent_covariance, unit_bearing, wait_for,
)

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
TEST_PARAMS = os.path.join(TEST_DIR, 'config', 'test_params.yaml')

# (uwb_address, uvdar_id) per test, all declared in test/config/test_params.yaml.
#
# Distinct per test on purpose. The node keeps the newest range per address for
# range_timeout_sec, so two tests sharing an address could see the earlier test's
# distance fused before their own arrived - a failure that would look like a bug in
# the node and would only appear under a particular test order.
PAIR = {
    'geometry': (0xE1, 101),
    'frame': (0xE2, 102),
    'below': (0xE3, 103),
    'above': (0xE4, 104),
    'two_a': (0xE5, 105),
    'two_b': (0xE6, 106),
    'covariance': (0xE7, 107),
    'fallback': (0xE8, 108),
    'symmetric': (0xE9, 109),
    'peer_init': (0xEA, 110),
    'peer_resp': (0xEB, 111),
    'overheard': (0xEC, 112),
    'window': (0xED, 113),
    'unidentified': (0xEE, 114),
    'predicted': (0xEF, 115),
    'stamps': (0xF0, 116),
    'rate': (0xF1, 117),
    'staleness': (0xF2, 118),
    'noempty': (0xF3, 119),
    'merged': (0xF4, 120),
    'crossbatch': (0xF5, 121),
}

OWN_ADDRESS = 0xAA


@pytest.fixture(scope='session', autouse=True)
def middleware_matches_the_pin(rclpy_context):
    """Fail fast if the middleware loaded is not the one conftest.py pinned.

    conftest.py does the pinning, and it has to happen at import time because ROS
    resolves the RMW when a typesupport library is first loaded - which the message
    imports at the top of this module trigger. A fixture is far too late to change it,
    so this one only checks.

    Worth checking rather than trusting: on a mismatch the two sides simply never
    discover each other. The node still logs its configuration line, the topic still
    appears in get_topic_names_and_types(), and the only symptom is every test timing
    out - which reads like a broken node and sends you looking at the wrong code.
    """
    import rclpy.utilities

    expected = os.environ['RMW_IMPLEMENTATION']
    loaded = rclpy.utilities.get_rmw_implementation_identifier()
    if loaded != expected:
        raise RuntimeError(
            f'rclpy loaded {loaded!r} but the tests are pinned to {expected!r}.\n'
            f'  RMW_IMPLEMENTATION is resolved when the first typesupport is dlopen()ed, '
            f'so it must be set in conftest.py, before any message import.\n'
            f'  available: {sorted(rclpy.utilities.get_available_rmw_implementations())}')
    yield


def find_fusion_executable():
    """Locate the built node, so the test never hard-codes a workspace path."""
    from ament_index_python.packages import get_package_prefix

    prefix = get_package_prefix('mrs_ultraloc')
    path = os.path.join(prefix, 'lib', 'mrs_ultraloc', 'uwb_uvdar_fusion_node')
    if not os.path.exists(path):
        raise RuntimeError(f'built fusion node not found at {path}; run colcon build first')
    return path


@pytest.fixture(scope='session')
def fusion_node():
    """One node process for the whole module.

    Session-scoped because the thing most likely to make these tests flaky is
    discovery: each new process pays a second or so of DDS matching, and restarting
    the node between tests would pay it a dozen times for no benefit. The tests are
    independent by construction (distinct id pairs), so sharing one process costs
    nothing.
    """
    executable = find_fusion_executable()
    process = subprocess.Popen(
        [executable, '--ros-args', '--params-file', TEST_PARAMS],
        # Inheriting the environment is the point: conftest.py pinned the RMW and domain
        # there so that this child and the rclpy node in this process agree.
        env=dict(os.environ),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    # The node logs one INFO line per sensor timeout, so a test that waits on
    # staleness can fill the 64 KiB pipe and wedge the child in write(). Draining into
    # a list keeps the node's own writes non-blocking and keeps the line for the report.
    lines = []

    def drain():
        for line in process.stdout:
            lines.append(line.rstrip())

    reader = threading.Thread(target=drain, daemon=True)
    reader.start()

    # Wait for the startup line specifically, not merely for a live process. It names
    # the pair count, both input topics, the output topic and the rate, which is the
    # only cheap proof that the parameter file was applied at all - a file keyed to the
    # wrong node name is ignored silently - and that DDS came up on the RMW we pinned.
    # Reaching a test before that would just trade this message for a topic timeout.
    assert wait_for(lambda: any('Fusing' in line for line in lines), 30.0), \
        ('fusion node never reported its configuration within 30 s.\n'
         f'  {os.environ.get("RMW_IMPLEMENTATION")} on domain {os.environ.get("ROS_DOMAIN_ID")}\n'
         '  output so far:\n' + ('\n'.join('    ' + line for line in lines) or '    (nothing)'))

    yield process, lines

    if process.poll() is None:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait(timeout=5)
    process.stdout.close()


@pytest.fixture(scope='session')
def rclpy_context():
    """Init rclpy once for the module.

    rclpy.init() is per-process, not per-node, so doing it in a function-scoped
    fixture either raises or - with an `if not rclpy.ok()` guard - leaves a context
    alive that nothing ever shuts down. rclpy.shutdown() then has to run while no
    callback is in flight, which is why the spin thread below is stopped and joined
    before this fixture tears down. Leaving a daemon thread inside rclpy at interpreter
    exit takes the whole pytest process down with a core dump instead of a report.
    """
    import rclpy

    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def driver(rclpy_context, fusion_node):
    """Publishes synthetic inputs and records fused output for one test.

    Depending on fusion_node here rather than making it autouse keeps the meaning of
    the two fixtures separate - "the node is running" is the node's business, "this
    test talks to it" is the driver's - and every test that needs the node goes through
    a driver, so nothing runs without it in practice.
    """
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    node = rclpy.create_node('fusion_test_driver')
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # The tests block waiting for output, so the callback needs another thread; a
    # spin in the test body would starve the subscription it is waiting on.
    #
    # Bounded spin_once calls rather than spin(), because spin() only returns when the
    # context dies and would then have to be abandoned as a daemon thread.
    stop_spinning = threading.Event()

    def pump():
        while not stop_spinning.is_set():
            executor.spin_once(timeout_sec=0.05)

    spinning = threading.Thread(target=pump, daemon=True)
    spinning.start()

    harness = FusionDriver(node)

    # Wait for the node to be reachable before handing control to a test, so a
    # failure means the node is wrong rather than not yet discovered.
    process, node_log = fusion_node
    try:
        assert wait_for(lambda: node.count_publishers(OUTPUT_TOPIC) > 0, 20.0), (
            f'no publisher on {OUTPUT_TOPIC}.\n'
            f'  this process loaded {rclpy.utilities.get_rmw_implementation_identifier()}; '
            f'env says {os.environ["RMW_IMPLEMENTATION"]}, '
            f'domain {os.environ["ROS_DOMAIN_ID"]}, node alive: {process.poll() is None}\n'
            f'  publishers on output: {node.count_publishers(OUTPUT_TOPIC)}, '
            f'subscribers on inputs: bearing={node.count_subscribers(BEARING_TOPIC)} '
            f'uwb={node.count_subscribers(UWB_TOPIC)}\n'
            f'  topics visible here: {sorted(name for name, _ in node.get_topic_names_and_types())}\n'
            '  node log:\n' + '\n'.join('    ' + line for line in node_log[-15:])
        )
        yield harness
    finally:
        stop_spinning.set()
        spinning.join(timeout=2.0)
        executor.shutdown()
        node.destroy_node()


def feed(driver, signal_id, uwb_address, bearing, distance, covariance=None, predicted=False,
         direction='initiator'):
    """A continuously refreshed single-target input set.

    Refreshed rather than sent once because the node gates on the age of each
    sensor: one message would make every later assertion a race with the timeouts.
    """
    def supplier():
        driver.publish_bearings([(signal_id, bearing, covariance, predicted)])
        driver.publish_range(OWN_ADDRESS, uwb_address, distance, direction=direction)

    return SteadyFeed(driver).start(supplier)


def expect_fused(driver, mark, signal_id, timeout=8.0):
    """The newest fused target for `signal_id`, failing with a readable message."""
    assert wait_for(lambda: driver.newest_target(mark, signal_id) is not None, timeout), \
        (f'nothing fused for UVDAR id {signal_id} within {timeout} s - check that the pair is in '
         f'test/config/test_params.yaml and that inputs reached the node')
    return driver.newest_target(mark, signal_id)[1]


class TestGeometry:
    """position = bearing * range, published in the bearing's own frame."""

    def test_position_is_bearing_scaled_by_range(self, driver):
        uwb_address, signal_id = PAIR['geometry']
        bearing = unit_bearing(math.radians(30.0), math.radians(15.0))
        distance = 6.5

        mark = driver.marks()
        with feed(driver, signal_id, uwb_address, bearing, distance):
            target = expect_fused(driver, mark, signal_id)
            expected = tuple(component * distance for component in bearing)
            got = (target.position.x, target.position.y, target.position.z)

            assert max_abs_diff(got, expected) < EXPECTED_TOLERANCE, f'{got} != expected {expected}'
            # The fusion is a pure scaling, so the norm has to come back as the range.
            assert abs(math.dist(got, (0.0, 0.0, 0.0)) - distance) < EXPECTED_TOLERANCE

            # The bearing is carried through rather than reconstructed from position,
            # so a consumer can treat direction and range as two measurements.
            assert max_abs_diff((target.bearing.x, target.bearing.y, target.bearing.z),
                                bearing) < EXPECTED_TOLERANCE
            assert abs(target.distance - distance) < EXPECTED_TOLERANCE

    def test_frame_id_is_the_bearing_frame_unrotated(self, driver):
        """The bearing is used in the frame it arrives in, unrotated - see the README:
        uvdar_core's bearing endpoint has already rotated every camera into its
        `output_frame`, so a rotation applied here would be applied twice."""
        uwb_address, signal_id = PAIR['frame']
        bearing = unit_bearing(math.radians(10.0), 0.0)

        mark = driver.marks()
        with feed(driver, signal_id, uwb_address, bearing, 4.0):
            expect_fused(driver, mark, signal_id)
            assert driver.frames_since(mark) == {FRAME}

    def test_z_keeps_its_sign_on_both_sides_of_the_axis(self, driver):
        """The ROS 1 fusion took z as sqrt(d^2 - x^2 - y^2), which forced z >= 0.

        With a unit bearing that term is |b.z| * d, so it destroyed the sign. Both
        signs are fused in one window, sharing x and y, so a node that flipped
        either is caught in a single run.
        """
        lower_addr, lower_id = PAIR['below']
        upper_addr, upper_id = PAIR['above']

        elevation, azimuth = math.radians(20.0), math.radians(35.0)
        below = unit_bearing(azimuth, elevation)
        above = (below[0], below[1], -below[2])
        assert below[2] > 0.0 and above[2] < 0.0

        distance_below, distance_above = 6.0, 3.0
        mark = driver.marks()

        def supplier():
            driver.publish_bearings([(lower_id, below, None, False), (upper_id, above, None, False)])
            driver.publish_range(OWN_ADDRESS, lower_addr, distance_below)
            driver.publish_range(OWN_ADDRESS, upper_addr, distance_above)

        with SteadyFeed(driver).start(supplier):
            lower = expect_fused(driver, mark, lower_id)
            upper = expect_fused(driver, mark, upper_id)

            for target, bearing, distance, label in ((lower, below, distance_below, 'below axis'),
                                                     (upper, above, distance_above, 'above axis')):
                expected = tuple(c * distance for c in bearing)
                got = (target.position.x, target.position.y, target.position.z)
                assert max_abs_diff(got, expected) < EXPECTED_TOLERANCE, f'{label}: {got} != {expected}'

            assert lower.position.z > 0.0, 'the target below the axis lost its z sign'
            assert upper.position.z < 0.0, 'the target above the axis was forced non-negative (the ROS 1 sqrt)'

    def test_two_targets_are_fused_independently_in_one_message(self, driver):
        lower_addr, lower_id = PAIR['two_a']
        upper_addr, upper_id = PAIR['two_b']

        below = unit_bearing(math.radians(5.0), math.radians(8.0))
        above = unit_bearing(math.radians(-20.0), math.radians(-30.0))
        mark = driver.marks()

        def supplier():
            driver.publish_bearings([(lower_id, below, None, False), (upper_id, above, None, False)])
            driver.publish_range(OWN_ADDRESS, lower_addr, 2.5)
            driver.publish_range(OWN_ADDRESS, upper_addr, 9.0)

        with SteadyFeed(driver).start(supplier):
            lower = expect_fused(driver, mark, lower_id)
            upper = expect_fused(driver, mark, upper_id)

            ids = {t.id for t in driver.messages()[-1].targets}
            assert {lower_id, upper_id} <= ids, f'targets were not published together: {sorted(ids)}'

            # Each scaled by its own range, not by whichever arrived last.
            for target, distance, label in ((lower, 2.5, 'near'), (upper, 9.0, 'far')):
                norm = math.dist((target.position.x, target.position.y, target.position.z), (0, 0, 0))
                assert abs(norm - distance) < EXPECTED_TOLERANCE, f'{label} target used the wrong range'

    def test_two_observations_of_one_id_in_one_batch_are_averaged(self, driver):
        """Two cameras' sightings handed over *together* are averaged into one direction.

        This is the node's in-batch rule, and it is the one the multi-camera case below
        does NOT normally reach: uvdar_core's bearing node builds one output message per
        tracker callback and `onTrackerOutput` runs per camera input, so a real rig
        delivers each camera's ray in its own batch and the two never meet in one
        `observations` list. The averaging is kept and tested because the message does not
        name its camera, so nothing here can tell a merged producer from a per-camera one -
        but read `test_two_cameras_reporting_in_alternate_batches_overwrite` beside it for
        what a `three_bluefox_bearing.yaml` rig actually does.

        One of the two is `predicted` and the other measured, which is what a pair from a
        rig where only one camera has a fresh detector association would look like.
        """
        uwb_address, signal_id = PAIR['merged']
        distance = 6.0

        # One camera either side of the vehicle's x-z plane, 15 deg apart in azimuth at a
        # shared 5 deg elevation. Wide enough that "the last one won" and "the two were
        # averaged" differ by about 0.8 m at this range rather than by float noise.
        near = unit_bearing(math.radians(-7.5), math.radians(5.0))
        far = unit_bearing(math.radians(7.5), math.radians(5.0))

        cov_near = tangent_covariance(near, math.radians(1.0))
        cov_far = tangent_covariance(far, math.radians(3.0))

        average = mean_direction([near, far])

        # The expectation is a direction, not a scaled one: the test's own arithmetic is
        # checked here so a mean that was never renormalised could not become the
        # baseline the rest of the test is measured against.
        assert abs(math.sqrt(sum(c ** 2 for c in average)) - 1.0) < 1e-12, 'mean is not a unit vector'
        assert abs(average[1]) < 1e-12, 'the test geometry is not symmetric about x-z'
        # Two points at equal elevation average to one slightly *above* that elevation -
        # the mean of the elevations is not the elevation of the mean direction - because
        # the sum is shorter than 2 and the renormalisation lifts every component.
        assert average[2] > near[2], 'averaging two unit vectors must shorten z, not lengthen it'

        mark = driver.marks()

        def supplier():
            driver.publish_bearings([
                (signal_id, near, flatten(cov_near), False),
                (signal_id, far, flatten(cov_far), True),
            ])
            driver.publish_range(OWN_ADDRESS, uwb_address, distance)

        with SteadyFeed(driver).start(supplier):
            target = expect_fused(driver, mark, signal_id)

            expected = tuple(component * distance for component in average)
            got = (target.position.x, target.position.y, target.position.z)
            error = max_abs_diff(got, expected)
            assert error < EXPECTED_TOLERANCE, f'position is not mean(b1, b2) * r ({error:.3e})'

            # The two cameras' directions were each a few degrees off this one, so a
            # result equal to either of them means the second was overwritten, not merged.
            for label, bearing in (('first camera', near), ('second camera', far)):
                assert max_abs_diff(got, tuple(c * distance for c in bearing)) > 1e-3, \
                    f'the target sits exactly on the {label} - the pair was not averaged'

            # Published once: two observations of one id are one target, not two entries.
            batch = driver.messages()[-1]
            assert [t.id for t in batch.targets].count(signal_id) == 1, \
                'one id produced two targets'

            # The covariance is averaged across the observations, not summed: two cameras
            # agreeing about one blinker do not double its angular uncertainty. A sum
            # would be exactly twice the bearing term below, and keeping only one
            # observation would leave a 1 deg or a 3 deg term where the mean has 2 deg, so
            # checking the exact value settles all three readings at once.
            mean_cov = [[(cov_near[i][j] + cov_far[i][j]) / 2.0 for j in range(3)] for i in range(3)]
            expected_cov = expected_position_covariance(average, distance, mean_cov, RANGE_SIGMA_M)
            error = max_abs_diff(target.covariance, flatten(expected_cov))
            assert error < 1e-9, f'P is not r^2*mean(Pb) + sigma_r^2*bb^T (max element error {error:.3e})'

            # `bearing_predicted` is the OR of "not measured": a blinker one camera still
            # has a fresh fix on is a measured target, even though the other camera is
            # only coasting. Reporting it as predicted would make a downstream filter drop
            # a live target, so this is the one field where the two observations are
            # combined by anything other than an average.
            assert not target.bearing_predicted, 'a target one camera still measures was reported as predicted'

    def test_two_cameras_reporting_in_alternate_batches_overwrite(self, driver):
        """What a `three_bluefox_bearing.yaml` rig actually delivers, and what it costs.

        uvdar_core's bearing node publishes once per tracker callback and that callback
        runs per camera input, so each camera's ray reaches this node in its own batch
        (bearing_node.cpp:218-221, whose comment spells out the same reading). So the
        averaging above is NOT the multi-camera path - each batch replaces `bearings_[id]`
        and the camera that reported last wins outright.

        Asserted rather than merely noted because the result is a target whose direction
        alternates between the two cameras at the tracker rate, at full amplitude: a
        ~1.5 m jump here, not a subtle bias. A downstream filter seeing that would report
        a fusion that disagrees with itself, which is what this test's name is for. It is
        a characterisation of current behaviour, deliberately - changing the behaviour is
        a decision about which measurement to prefer, and the message carries no camera
        name to make that decision with.

        Driven by hand rather than through `SteadyFeed` so there is no race to lose: the
        node republishes at 20 Hz while these two batches are written back to back, so a
        feed thread could have a fusion tick land between them and the answer would then
        depend on scheduling. Each step instead publishes, waits for the fused value to
        settle, and asserts on it - and keeps the range fresh across both, so "newest
        bearing wins" is being tested rather than "the other one expired".
        """
        uwb_address, signal_id = PAIR['crossbatch']
        distance = 6.0

        left = unit_bearing(math.radians(-7.5), math.radians(5.0))
        right = unit_bearing(math.radians(7.5), math.radians(5.0))

        def fused_matches(bearing):
            """True once a fusion tick has published this camera's direction exactly."""
            expected = tuple(c * distance for c in bearing)
            return any(max_abs_diff((t.position.x, t.position.y, t.position.z), expected) < 1e-9
                       for _, t in driver.targets_since(mark, signal_id))

        mark = driver.marks()
        driver.publish_range(OWN_ADDRESS, uwb_address, distance)
        driver.publish_bearings([(signal_id, left, None, False)])
        assert wait_for(lambda: fused_matches(left), 8.0), \
            'the first batch was never fused at all'

        # Both sightings are well inside bearing_timeout_sec (0.5 s) at this point, so a
        # node that averaged the pair, or kept the first, or kept both, is distinguishable
        # here from one that replaced the sample.
        driver.publish_range(OWN_ADDRESS, uwb_address, distance)
        driver.publish_bearings([(signal_id, right, None, False)])
        assert wait_for(lambda: fused_matches(right), 8.0), \
            ('the second camera\'s batch never took effect - the node is not keeping the '
             'newest observation for an id it has already seen')

        target = expect_fused(driver, mark, signal_id)
        got = (target.position.x, target.position.y, target.position.z)
        mean = tuple(c * distance for c in mean_direction([left, right]))
        assert max_abs_diff(got, mean) > 1e-3, \
            'the two batches were averaged; each batch replaces the sample on its own'

        # The same target, once: replacement happens per id, so one blinker stays one
        # target as each camera reports.
        assert [t.id for t in driver.messages()[-1].targets].count(signal_id) == 1

        # How far the direction swings per batch, which is the whole disagreement between
        # the two cameras - nothing here damps it.
        span = max_abs_diff(tuple(c * distance for c in left),
                            tuple(c * distance for c in right))
        assert span > 1.0, f'test geometry too tight to show the alternation ({span:.3f} m)'


class TestCovariance:
    """The 3x3 position covariance a downstream filter is meant to consume."""

    def test_covariance_propagates_the_bearing_covariance(self, driver):
        uwb_address, signal_id = PAIR['covariance']
        bearing = unit_bearing(math.radians(50.0), math.radians(-25.0))
        distance = 7.0
        bearing_covariance = tangent_covariance(bearing, math.radians(1.0))

        mark = driver.marks()
        with feed(driver, signal_id, uwb_address, bearing, distance, covariance=flatten(bearing_covariance)):
            target = expect_fused(driver, mark, signal_id)

            expected = expected_position_covariance(bearing, distance, bearing_covariance, RANGE_SIGMA_M)
            error = max_abs_diff(target.covariance, flatten(expected))
            assert error < 1e-9, f'P != r^2*Pb + sigma_r^2*bb^T (max element error {error:.3e})'

            # The rank-2 bearing term and the radial term together span all three axes.
            assert min(target.covariance[0], target.covariance[4], target.covariance[8]) > 0.0

    def test_missing_covariance_falls_back_to_the_configured_sigma(self, driver):
        """An all-zero covariance means "no estimate", not "exactly known"."""
        uwb_address, signal_id = PAIR['fallback']
        bearing = unit_bearing(math.radians(20.0), math.radians(40.0))
        distance = 4.0

        mark = driver.marks()
        with feed(driver, signal_id, uwb_address, bearing, distance, covariance=None):
            target = expect_fused(driver, mark, signal_id)

            fallback = tangent_covariance(bearing, BEARING_SIGMA_RAD)
            expected = expected_position_covariance(bearing, distance, fallback, RANGE_SIGMA_M)
            error = max_abs_diff(target.covariance, flatten(expected))
            assert error < 1e-9, f'fallback is not sigma_b^2*(I-bb^T)*r^2 + sigma_r^2*bb^T ({error:.3e})'

            # A bearing with elevation has tangent-plane extent in z, so a zero here
            # would mean the fallback was skipped and the covariance reported as zero.
            assert target.covariance[8] > 1e-9

    def test_covariance_is_symmetric_and_positive_semidefinite(self, driver):
        uwb_address, signal_id = PAIR['symmetric']
        bearing = unit_bearing(math.radians(-15.0), math.radians(30.0))
        bearing_covariance = tangent_covariance(bearing, math.radians(2.0))

        mark = driver.marks()
        with feed(driver, signal_id, uwb_address, bearing, 5.0, covariance=flatten(bearing_covariance)):
            target = expect_fused(driver, mark, signal_id)

            matrix = [[target.covariance[3 * i + j] for j in range(3)] for i in range(3)]
            for i in range(3):
                assert matrix[i][i] >= 0.0, 'covariance has a negative diagonal'
                for j in range(3):
                    assert math.isfinite(matrix[i][j]), f'covariance[{3 * i + j}] is not finite'
                    assert abs(matrix[i][j] - matrix[j][i]) < 1e-12, 'covariance is not symmetric'

            # Eigenvalues via the characteristic polynomial's roots would be overkill;
            # the 2x2 minors and the determinant being non-negative is the practical
            # positive-semidefinite check for a symmetric 3x3.
            det = sum(matrix[0][j] * (matrix[1][(j + 1) % 3] * matrix[2][(j + 2) % 3]
                                      - matrix[1][(j + 2) % 3] * matrix[2][(j + 1) % 3]) for j in range(3))
            minor = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]
            assert minor >= -1e-15 and det >= -1e-18, f'covariance is not positive semidefinite (det {det:.3e})'


class TestAttribution:
    """Which range belongs to which vehicle, and which ranges may be used."""

    def test_peer_resolved_from_either_endpoint(self, driver):
        """A range report names initiator, responder and own; this vehicle can be either."""
        initiator_addr, initiator_id = PAIR['peer_init']
        responder_addr, responder_id = PAIR['peer_resp']

        bearing = unit_bearing(math.radians(12.0), math.radians(-6.0))
        mark = driver.marks()

        def supplier():
            driver.publish_bearings([(initiator_id, bearing, None, False),
                                     (responder_id, bearing, None, False)])
            driver.publish_range(OWN_ADDRESS, initiator_addr, 3.0, direction='initiator')
            driver.publish_range(OWN_ADDRESS, responder_addr, 8.0, direction='responder')

        with SteadyFeed(driver).start(supplier):
            first = expect_fused(driver, mark, initiator_id)
            second = expect_fused(driver, mark, responder_id)

            assert abs(first.distance - 3.0) < 1e-9
            assert abs(second.distance - 8.0) < 1e-9, 'the responder-side range was not attributed'

    def test_overheard_range_from_another_module_is_ignored(self, driver):
        """own_address matching neither endpoint means the report is not ours."""
        uwb_address, signal_id = PAIR['overheard']
        bearing = unit_bearing(math.radians(12.0), 0.0)
        bogus_distance = 42.0

        mark = driver.marks()

        def supplier():
            driver.publish_bearings([(signal_id, bearing, None, False)])
            # A good range keeps the pair alive while the foreign one tests the gate;
            # if the foreign one were ever stored, the target would jump to 42 m.
            driver.publish_range(OWN_ADDRESS, uwb_address, 3.0, direction='initiator')
            driver.publish_range(0x33, 0x11, bogus_distance, direction='foreign')

        with SteadyFeed(driver).start(supplier):
            expect_fused(driver, mark, signal_id)
            time.sleep(0.5)

            distances = {round(t.distance, 6) for _, t in driver.targets_since(mark, signal_id)}
            assert bogus_distance not in distances, f'fused a foreign module range: {sorted(distances)}'
            assert distances == {3.0}, f'expected only the own range, got {sorted(distances)}'

    def test_range_outside_the_window_never_replaces_a_good_one(self, driver):
        """Bad fixes are dropped before storing, not after overwriting."""
        uwb_address, signal_id = PAIR['window']
        bearing = unit_bearing(math.radians(25.0), math.radians(10.0))
        good_distance = 5.0

        mark = driver.marks()

        def supplier():
            driver.publish_bearings([(signal_id, bearing, None, False)])
            driver.publish_range(OWN_ADDRESS, uwb_address, good_distance)
            driver.publish_range(OWN_ADDRESS, uwb_address, MAX_RANGE_M * 3.0)   # above max_range_m
            driver.publish_range(OWN_ADDRESS, uwb_address, 0.01)                # below min_range_m

        with SteadyFeed(driver).start(supplier):
            expect_fused(driver, mark, signal_id)
            time.sleep(0.5)

            distances = {round(t.distance, 6) for _, t in driver.targets_since(mark, signal_id)}
            assert distances == {good_distance}, f'an out-of-window range was stored: {sorted(distances)}'

    def test_unidentified_tracks_are_not_fused(self, driver):
        """id < 0 marks an undecoded track, which no UWB address can describe."""
        uwb_address, signal_id = PAIR['unidentified']
        bearing = unit_bearing(math.radians(30.0), 0.0)

        mark = driver.marks()

        def supplier():
            driver.publish_bearings([(-1, (1.0, 0.0, 0.0), None, False),
                                     (-7, bearing, None, False),
                                     (signal_id, bearing, None, False)])
            driver.publish_range(OWN_ADDRESS, uwb_address, 6.0)

        with SteadyFeed(driver).start(supplier):
            expect_fused(driver, mark, signal_id)
            time.sleep(0.3)

            fused = {t.id for _, t in driver.targets_since(mark)}
            assert not (fused & {-1, -7}), f'unidentified tracks were fused: {sorted(fused)}'


class TestTiming:

    def test_predicted_flag_follows_the_newest_observation(self, driver):
        uwb_address, signal_id = PAIR['predicted']
        bearing = unit_bearing(math.radians(18.0), math.radians(-12.0))

        mark = driver.marks()
        with feed(driver, signal_id, uwb_address, bearing, 5.0, predicted=True):
            assert expect_fused(driver, mark, signal_id).bearing_predicted, \
                'a tracker prediction was reported as a measurement'

        mark = driver.marks()
        with feed(driver, signal_id, uwb_address, bearing, 5.0, predicted=False):
            assert not expect_fused(driver, mark, signal_id).bearing_predicted, \
                'the flag stayed set after a measured observation'

    def test_stamps_of_both_inputs_are_carried(self, driver):
        """A downstream filter cannot reject a stale side it cannot time."""
        uwb_address, signal_id = PAIR['stamps']
        bearing = unit_bearing(math.radians(1.0), math.radians(2.0))

        mark = driver.marks()
        with feed(driver, signal_id, uwb_address, bearing, 3.5):
            target = expect_fused(driver, mark, signal_id)

            for name, stamp in (('bearing_stamp', target.bearing_stamp), ('range_stamp', target.range_stamp)):
                assert stamp.sec > 0, f'{name} was not carried through'
                assert 0 <= stamp.nanosec < 10**9, f'{name}.nanosec is out of range'

    def test_publishes_at_the_configured_rate(self, driver):
        uwb_address, signal_id = PAIR['rate']
        bearing = unit_bearing(math.radians(40.0), math.radians(5.0))

        mark = driver.marks()
        with feed(driver, signal_id, uwb_address, bearing, 4.0):
            expect_fused(driver, mark, signal_id)

            window = 2.0
            mark = driver.marks()
            time.sleep(window)
            count = driver.count_since(mark, signal_id)

        expected = PUBLISH_RATE_HZ * window
        # A wall timer plus DDS delivery is not exact; 20% absorbs jitter while still
        # rejecting a node running at half or double the configured rate.
        assert 0.8 * expected <= count <= 1.2 * expected, \
            f'{count} messages in {window} s, expected ~{expected:.0f} at {PUBLISH_RATE_HZ} Hz'

    def test_empty_arrays_are_not_published(self, driver):
        """Silence must mean "nothing fused", never an empty message.

        Fed on purpose: checking for empty arrays while nothing is being fused at all
        would pass without the node publishing a single message, which is not the
        property under test.
        """
        uwb_address, signal_id = PAIR['noempty']
        bearing = unit_bearing(math.radians(8.0), math.radians(-4.0))

        mark = driver.marks()
        with feed(driver, signal_id, uwb_address, bearing, 5.0):
            expect_fused(driver, mark, signal_id)
            mark = driver.marks()
            time.sleep(1.0)

        batches = driver.messages()[mark:]
        assert batches, 'received no messages at all, so emptiness could not have been observed'
        empty = [msg for msg in batches if not msg.targets]
        assert not empty, f'published {len(empty)} empty arrays out of {len(batches)}'


class TestStaleness:
    """The per-sensor freshness gate that keeps stale pairs out of the output."""

    def test_output_continues_briefly_then_stops_when_inputs_stop(self, driver):
        """A stale bearing or range is not used, so a departed teammate disappears."""
        uwb_address, signal_id = PAIR['staleness']
        bearing = unit_bearing(math.radians(30.0), math.radians(15.0))

        mark = driver.marks()
        steady = feed(driver, signal_id, uwb_address, bearing, 6.0)
        expect_fused(driver, mark, signal_id)

        # Stopping the feed also stops refreshing the stamps. Output must then stop
        # rather than republish the last fusion indefinitely.
        mark = driver.marks()
        steady.stop()

        time.sleep(BEARING_TIMEOUT_SEC)
        tail = driver.count_since(mark, signal_id)

        time.sleep(2.0 * RANGE_TIMEOUT_SEC + 1.0)
        after = driver.count_since(mark, signal_id)

        assert tail > 0, 'output stopped instantly, so the timeout window is not being honoured'
        # The gate holds for the shorter timeout (the bearing side here).
        assert tail <= (BEARING_TIMEOUT_SEC + 0.4) * PUBLISH_RATE_HZ + 5, \
            f'{tail} messages published after the inputs stopped'
        assert after == tail, f'{after - tail} further messages after both inputs expired'
