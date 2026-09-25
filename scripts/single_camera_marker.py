#!/usr/bin/env python3
"""Shows what the fusion node thinks it sees, in RViz.

Reads `mrs_ultraloc/msg/FusionTargetArrayStamped` - the fusion node's output - and
publishes one `visualization_msgs/msg/MarkerArray` containing, per target:

  * an ellipsoid for the position covariance, rotated into the covariance's own frame;
  * a sphere at the fused position;
  * a faint line from the camera origin to it, which is the bearing scaled by the range;
  * its recorded history as a fading trail, so the flight path is visible;
  * a label with the target's id and distance.

Plus, at the origin, the camera: three axis arrows in the frame the bearings are
published in, and optionally a view frustum.

    ros2 run mrs_ultraloc single_camera_marker.py --ros-args \
        -p input_topic:=/uav/uwb_uvdar_fusion/targets

Usually started through `single_camera_marker.launch.py`, which also starts RViz with
`config/single_camera_marker.rviz` - a config with the fixed frame and the one Marker
display already set, so there is nothing to click. Against the simulator:

    ros2 launch mrs_ultraloc sim_fusion.launch.py              # in another shell
    ros2 launch mrs_ultraloc single_camera_marker.launch.py

What it shows is the geometry the fusion *claims*, not what the sensors measured. That
is the useful thing to look at while the numbers are new: a covariance far too small for
the scatter, a target teleporting between two places, or a sign error in z are all
immediate on screen and all but invisible in `ros2 topic echo`.

One topic, one display
----------------------
Everything goes into a single MarkerArray on one topic, so RViz needs one Marker display.
Its `Covariance` display would draw the ellipsoid from a `PoseWithCovarianceStamped`
instead, but that message holds one pose, so N targets would need N topics and N
displays - and the marker route carries the trail and the label too.

The ellipsoid is a SPHERE marker posed at the covariance's eigenvector frame with
per-axis scale `2 * n_sigma * sqrt(eigenvalue)`: a sphere scaled along its own local axes
*is* a rotated ellipsoid, and it is the only way to get one out of RViz markers, which
have no ellipsoid type. `n_sigma` is a parameter because 1 sigma of a UWB+UVDAR fusion is
a small shape at 5 m and 2 sigma is the one that reads as "that error bar is absurdly
large" - both are worth seeing.

Parameters
----------
The appearance ones - `n_sigma`, `target_diameter_m`, `trail_width_m`, `show_labels`,
`show_range_line`, `*_alpha` - are read on every publish, so `ros2 param set` changes
what is on screen while it runs. The structural ones - topics, `publish_rate_hz`,
`history_max_points`, `target_timeout_sec`, the frustum - are read once at startup,
because they decide what gets recorded, allocated or scheduled, and a parameter whose
change does not take effect until restart is only misleading if it is not labelled.
"""

import glob
import math
import os
import sys

# An installed node is run by path, so sys.path[0] is <prefix>/lib/mrs_ultraloc and this
# package's own Python directory is not on it - the sourced setup files normally put it
# there through a PYTHONPATH hook, and when that hook is present none of this is needed.
# It is here for the other case, running the script straight out of a checkout, where
# there is no prefix at all and `ultraloc_tools` is the sibling of `scripts/`.
#
# Both layouts put the module one directory up from this file: `<pkg>/scripts/` above is
# the package, and `<prefix>/lib/mrs_ultraloc/` above is `<prefix>/lib`, which is where
# ament's Python install directory lands. The glob covers whatever versioned `python3.x`
# suffix sysconfig picked, so no Python version is hard-coded. Duplicated in the two
# simulator scripts because it cannot be shared - it has to run before the module it
# makes importable can be imported.
_here = os.path.dirname(os.path.abspath(__file__))
_parent = os.path.dirname(_here)
for _candidate in [_parent] + sorted(glob.glob(os.path.join(_parent, 'python*'))):
    if os.path.isdir(os.path.join(_candidate, 'ultraloc_tools')) and _candidate not in sys.path:
        sys.path.insert(0, _candidate)
        break

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402

from geometry_msgs.msg import Point, Quaternion  # noqa: E402
from std_msgs.msg import ColorRGBA  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

from mrs_ultraloc.msg import FusionTargetArrayStamped  # noqa: E402

from ultraloc_tools.parameters import (  # noqa: E402
    declare_float, declare_int, declare_string, read_float, read_int, read_rate,
    read_string,
)

# RViz keys a marker by (ns, id), so ids have to be unique per marker and stable per
# target. Which means a target needs a fixed slot that gets reused: handing out a fresh
# id per target would march through the id space on a rig that loses and regains
# targets, and stale ids are the kind of thing that shows up as a marker that cannot be
# got rid of.
TARGET_SLOT_STRIDE = 10
MAX_TARGET_SLOTS = 20

# Kinds within one target's slot.
ELLIPSOID = 0
POINT = 1
RANGE_LINE = 2
TRAIL = 3
LABEL = 4

# Camera markers, which occupy ids below the first target's slot.
AXIS_X = 0
AXIS_Y = 1
AXIS_Z = 2
FRUSTUM = 3
CAMERA_MARKER_BASE = 0

# Cycled through as targets appear, so two targets on screen are never the same colour.
# Assigned by slot rather than by target id so a returning target keeps the colour it had.
PALETTE = (
    (0.95, 0.35, 0.30),
    (0.30, 0.70, 0.95),
    (0.45, 0.85, 0.40),
    (0.98, 0.78, 0.25),
    (0.75, 0.52, 0.92),
    (0.95, 0.55, 0.75),
    (0.55, 0.90, 0.85),
    (0.85, 0.85, 0.50),
)

# Used until the first input message names one. The frame is taken from the input whenever
# the `frame_id` parameter is empty - which is the default and the setting that needs no
# knowledge of the rig - so this only has to be a name RViz can resolve in the seconds
# before the first message, and it renders nothing until then either way.
#
# It is uvdar_core's own default-bearing-config frame, which is what a pipeline started
# from that file unmodified stamps its bearings with. The multi-camera rig
# (one_cam/two_cams/three_cams.launch.py) stamps `<uav>/<slot>` instead, so on that rig
# this value is simply never correct and never used. Kept rather than emptied because a
# node with no frame at all cannot build a marker header.
FALLBACK_FRAME = 'camera_0_optical_frame'

# RViz ignores the alpha of per-point colours - stated in Marker.msg, "NOTE: alpha is not
# yet used" - so the trail's fade is done in brightness instead. Against RViz's default
# dark background, dimming towards black reads as fading out, which is what is wanted.
TRAIL_TAIL_BRIGHTNESS = 0.15


def quaternion_from_rotation_matrix(matrix):
    """(x, y, z, w) for an orthonormal, proper-rotation 3x3.

    Shepperd's method: pick the branch with the largest diagonal so no division by a
    near-zero trace happens. The naive formula divides by sqrt(1 + trace), which for a
    covariance whose largest eigenvalue is along -x is a division by something near zero,
    and the result is a quaternion of garbage magnitude - on screen, an ellipsoid that
    tumbles every frame, which reads as a data problem and is an arithmetic one.

    The caller guarantees a proper rotation; `principal_axes` flips a column if numpy
    hands back a reflection, which `eigh` may do and which has no quaternion at all.
    """
    m = matrix
    trace = m[0, 0] + m[1, 1] + m[2, 2]

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x, y, z = (0.25 * s,
                      (m[2, 1] - m[1, 2]) / s,
                      (m[0, 2] - m[2, 0]) / s,
                      (m[1, 0] - m[0, 1]) / s)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w, x, y, z = ((m[2, 1] - m[1, 2]) / s,
                      0.25 * s,
                      (m[0, 1] + m[1, 0]) / s,
                      (m[0, 2] + m[2, 0]) / s)
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w, x, y, z = ((m[0, 2] - m[2, 0]) / s,
                      (m[0, 1] + m[1, 0]) / s,
                      0.25 * s,
                      (m[1, 2] + m[2, 1]) / s)
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w, x, y, z = ((m[1, 0] - m[0, 1]) / s,
                      (m[0, 2] + m[2, 0]) / s,
                      (m[1, 2] + m[2, 1]) / s,
                      0.25 * s)

    return x, y, z, w


def principal_axes(covariance):
    """(eigenvalues descending, matching rotation matrix) of a symmetric 3x3.

    `numpy.linalg.eigh` is the symmetric/Hermitian solver: it returns real eigenvalues
    and an orthonormal basis. The general `eig` does not, and a covariance carrying a
    hair of asymmetry from round-off would otherwise come back complex, producing NaNs in
    the marker scale and an ellipsoid that silently does not render.

    Eigenvalues are clamped at zero rather than rejected. A tangent-plane bearing
    covariance is rank-2 by construction, so a zero eigenvalue is correct and expected -
    the ellipsoid is a flat ellipse, which is exactly the right picture - while a small
    negative from round-off would make `sqrt` NaN.
    """
    symmetric = 0.5 * (covariance + covariance.T)
    values, vectors = np.linalg.eigh(symmetric)

    order = np.argsort(values)[::-1]
    values = np.clip(values[order], 0.0, None)
    vectors = vectors[:, order]

    if np.linalg.det(vectors) < 0.0:
        # A reflection, not a rotation. Flip one axis to recover a proper rotation;
        # the ellipsoid is symmetric so the choice is free, and the smallest eigenvalue's
        # axis is the flattest one, so it is the least visible change.
        vectors[:, 2] = -vectors[:, 2]

    return values, vectors


class SingleCameraMarker(Node):
    """Fused targets, their history and their uncertainty, as RViz markers."""

    def __init__(self):
        super().__init__('single_camera_marker')

        # Relative, so running this node in the fusion node's namespace finds it with no
        # arguments - the fusion's output topic is relative to itself too.
        declare_string(self, 'input_topic', 'uwb_uvdar_fusion/targets')
        # Absolute, deliberately, while the input is relative. RViz reads the topic it
        # listens to out of config/single_camera_marker.rviz, which is a plain file that
        # cannot name a namespace, so a namespaced output topic would need the config
        # edited or a launch remap for every uav_name. Publishing on one fixed topic lets
        # the shipped config always be right. The cost: visualising two rigs on one
        # machine shares this topic - separate machines, or a remap on one of them.
        declare_string(self, 'output_topic', '/markers')
        # Empty means "follow the input message's frame_id", which is the setting that
        # works for the real pipeline and for the simulator without editing anything.
        # Set it explicitly to re-parent the whole picture into another frame, which then
        # needs a TF for the camera frame to exist there.
        declare_string(self, 'frame_id', '')
        declare_float(self, 'publish_rate_hz', 10.0)

        # ---- appearance, all read live ----
        declare_float(self, 'n_sigma', 1.0)
        # Named for what RViz's SPHERE scale means, which is the diameter, not the
        # radius: a `target_radius_m` that sets scale.x directly would draw targets half
        # the size they ask for.
        declare_float(self, 'target_diameter_m', 0.15)
        declare_float(self, 'trail_width_m', 0.03)
        declare_int(self, 'show_labels', 1)
        declare_int(self, 'show_range_line', 1)
        declare_float(self, 'range_line_alpha', 0.35)
        declare_float(self, 'covariance_alpha', 0.30)
        declare_float(self, 'trail_alpha', 0.9)

        # ---- structure, read once ----
        # 2000 points at the fusion's 20 Hz output is 100 s of flight: several laps of
        # the simulated path, which is what it takes to see whether a trajectory closes
        # on itself or drifts. Bump it to draw longer, not to draw more targets.
        declare_int(self, 'history_max_points', 2000)
        # A target that stops being reported stops being drawn, trail included. Beyond a
        # few seconds a still-drawn target is a stale screen rather than information.
        declare_float(self, 'target_timeout_sec', 2.0)

        declare_float(self, 'camera_axis_length_m', 0.4)
        declare_float(self, 'camera_axis_radius_m', 0.02)
        # 0 turns the frustum off, which is the default. A pyramid with guessed angles is
        # worse than none, because it looks like a claim about the hardware; take the
        # angles from the camera calibration before trusting one.
        declare_float(self, 'frustum_range_m', 0.0)
        declare_float(self, 'frustum_hfov_deg', 60.0)
        declare_float(self, 'frustum_vfov_deg', 45.0)

        self.input_topic = read_string(self, 'input_topic')
        self.output_topic = read_string(self, 'output_topic')
        self.history_max_points = max(read_int(self, 'history_max_points'), 2)
        self.target_timeout_sec = read_float(self, 'target_timeout_sec')
        self.camera_frame = read_string(self, 'frame_id') or FALLBACK_FRAME

        self._targets = {}      # target id -> {slot, points, distance, covariance, predicted, seen}
        self._slots = {}        # slot -> target id
        self._published_ids = set()
        self._overflow_warned = False

        # Reliable, matching the fusion node's publisher. A best-effort subscription
        # would still match, then drop messages under load and show a target skipping -
        # which is precisely the artefact a visualiser is supposed to be able to rule out.
        self.create_subscription(FusionTargetArrayStamped, self.input_topic, self.on_targets,
                                 QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))
        self.publisher = self.create_publisher(
            MarkerArray, self.output_topic,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))

        # Drawn on a timer rather than from the callback, for three reasons: the camera
        # has to be on screen before the first target arrives, RViz needs the complete
        # array every time (markers are added or deleted, never edited in place), and
        # decoupling the draw rate from a 20-60 Hz input keeps the render cheap.
        self.create_timer(1.0 / read_rate(self, 'publish_rate_hz'), self.publish_markers)

        self._publish_wipe()
        self.get_logger().info(
            f"visualising '{self.input_topic}' as markers on '{self.output_topic}'")

    # ---- input -------------------------------------------------------------

    def on_targets(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9

        if msg.header.frame_id and msg.header.frame_id != self.camera_frame:
            # Every position in the message, and every trail recorded so far, is in the
            # old frame - so the old points are not merely stale, they are in the wrong
            # place. Dropping them is the only honest option short of a TF transform,
            # which this node deliberately does not do.
            if self._targets:
                # Worth a warning rather than silence, because the symptom is the trails
                # vanishing, which looks like a visualiser bug.
                self.get_logger().warn(
                    f"target frame changed '{self.camera_frame}' -> '{msg.header.frame_id}'; "
                    'dropping recorded trails')
                self._reset_targets()
            self.camera_frame = msg.header.frame_id

        for target in msg.targets:
            entry = self._targets.get(target.id)
            if entry is None:
                slot = self._claim_slot(target.id)
                if slot is None:
                    continue
                entry = {'slot': slot, 'points': []}
                self._targets[target.id] = entry

            entry['points'].append((target.position.x, target.position.y, target.position.z))
            # deque-style trimming without importing deque: this list is also iterated in
            # order every draw, and appending to a list is cheaper than to a deque.
            del entry['points'][:-self.history_max_points]
            entry['distance'] = target.distance
            entry['covariance'] = np.array(target.covariance, dtype=float).reshape(3, 3)
            entry['predicted'] = bool(target.bearing_predicted)
            entry['seen'] = now

    def _claim_slot(self, target_id):
        if len(self._slots) >= MAX_TARGET_SLOTS:
            # rclpy has no throttled logger method, and this is reached on every input
            # message - so a plain warn() here would print at the input rate. The flag
            # makes it once per overflow instead, which is the only part worth reading.
            if not self._overflow_warned:
                self._overflow_warned = True
                self.get_logger().warn(
                    f'more than {MAX_TARGET_SLOTS} targets at once; the extras are not drawn. '
                    'The marker-id budget is fixed at startup; the targets drawn are the '
                    'ones that appeared first.')
            return None
        for slot in range(MAX_TARGET_SLOTS):
            if slot not in self._slots:
                self._slots[slot] = target_id
                return slot
        return None

    def _reset_targets(self):
        self._slots.clear()
        self._targets.clear()

    # ---- output ------------------------------------------------------------

    def _publish_wipe(self):
        """Wipe anything a previous run of this node drew.

        RViz keeps markers whose lifetime has not expired, and `lifetime = 0` means
        never, so after a restart - same namespace, same ids, same topic - a fresh run
        would show the previous run's trails underneath its own. That is invisible enough
        to be dangerous: a two-minute-old trail of the same target looks identical.
        """
        wipe = Marker()
        wipe.header.frame_id = self.camera_frame
        wipe.ns = MARKER_NAMESPACE
        wipe.action = Marker.DELETEALL
        # Sent on its own, ahead of any ADD: RViz applies an array in order, so a
        # DELETEALL in the same array as this cycle's markers would erase them too.
        self.publisher.publish(MarkerArray(markers=[wipe]))

    def publish_markers(self):
        # One clock read, not one per use, so the timeout comparison and the header stamp
        # cannot disagree about what now is.
        now_time = self.get_clock().now()
        now = now_time.nanoseconds * 1e-9
        stamp = now_time.to_msg()

        for target_id in [tid for tid, entry in self._targets.items()
                          if now - entry['seen'] > self.target_timeout_sec]:
            self._slots.pop(self._targets[target_id]['slot'], None)
            del self._targets[target_id]

        markers = self._camera_markers(stamp)
        for target_id, entry in self._targets.items():
            markers.extend(self._target_markers(target_id, entry, stamp))

        # Anything drawn last time and not wanted now is deleted explicitly. Without
        # this a target that vanishes leaves its ellipsoid and trail frozen on screen,
        # because lifetime 0 never expires and ids are reused rather than rotated.
        wanted = {marker.id for marker in markers}
        markers.extend(self._delete_marker(marker_id, stamp)
                       for marker_id in self._published_ids - wanted)
        self._published_ids = wanted

        self.publisher.publish(MarkerArray(markers=markers))

    # ---- camera ------------------------------------------------------------

    def _camera_markers(self, stamp):
        """Three axis arrows at the origin, plus a frustum if one was asked for.

        The frame is the *optical* frame - +x along the optical axis, +y left, +z up -
        not a body frame, so the arrows are worth having for that reason alone: a target
        at negative z is below the camera, and with the blue arrow pointing up there is no
        way to read it the other way round.
        """
        length = read_float(self, 'camera_axis_length_m')
        radius = read_float(self, 'camera_axis_radius_m')

        markers = []
        for kind, direction, colour in (
                (AXIS_X, (1.0, 0.0, 0.0), (1.0, 0.2, 0.2)),
                (AXIS_Y, (0.0, 1.0, 0.0), (0.2, 1.0, 0.2)),
                (AXIS_Z, (0.0, 0.0, 1.0), (0.3, 0.5, 1.0))):
            marker = self._marker(CAMERA_MARKER_BASE + kind, Marker.ARROW, stamp)
            # An ARROW's pose is its TAIL and scale.x its shaft length, so the tail goes
            # at the origin and the length is the full extent. Offsetting the tail by half
            # the length - which is what a pose-centred shape would need - makes an arrow
            # that starts in front of the camera and stops twice as far away.
            marker.pose.position = _point((0.0, 0.0, 0.0))
            marker.pose.orientation = _axis_quaternion(direction)
            marker.scale.x = max(length, 1e-3)
            marker.scale.y = max(radius, 1e-4)
            # The head is deliberately fatter than the shaft; equal values give an arrow
            # that reads as a plain rod at a metre away.
            marker.scale.z = max(radius * 2.0, 1e-4)
            marker.color.r, marker.color.g, marker.color.b = colour
            marker.color.a = 1.0
            markers.append(marker)

        frustum_range = read_float(self, 'frustum_range_m')
        if frustum_range > 0.0:
            markers.append(self._frustum_marker(stamp, frustum_range))
        return markers

    def _frustum_marker(self, stamp, frustum_range):
        """Four lines from the origin to the corners of the image plane at `frustum_range`.

        A rectangular FOV is an approximation - a real lens has distortion, and the
        calibration knows it - so this is for asking "was that target even in view", not
        for measuring anything.
        """
        half_y = frustum_range * math.tan(math.radians(read_float(self, 'frustum_hfov_deg')) / 2.0)
        half_z = frustum_range * math.tan(math.radians(read_float(self, 'frustum_vfov_deg')) / 2.0)
        corners = [(frustum_range, sy * half_y, sz * half_z)
                   for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)]

        marker = self._marker(CAMERA_MARKER_BASE + FRUSTUM, Marker.LINE_LIST, stamp)
        # LINE_LIST pairs consecutive points, so it has to be origin, corner, origin,
        # corner ... rather than a fan.
        for corner in corners:
            marker.points.append(_point((0.0, 0.0, 0.0)))
            marker.points.append(_point(corner))
        marker.scale.x = 0.008
        marker.color.r = marker.color.g = marker.color.b = 0.8
        marker.color.a = 0.5
        return marker

    # ---- targets ------------------------------------------------------------

    def _target_markers(self, target_id, entry, stamp):
        slot = entry['slot']
        colour = PALETTE[slot % len(PALETTE)]
        position = entry['points'][-1]
        base = CAMERA_MARKER_BASE + MAX_TARGET_SLOTS * TARGET_SLOT_STRIDE \
            + slot * TARGET_SLOT_STRIDE

        n_sigma = max(read_float(self, 'n_sigma'), 0.0)
        diameter = read_float(self, 'target_diameter_m')
        trail_width = read_float(self, 'trail_width_m')
        covariance_alpha = read_float(self, 'covariance_alpha')
        trail_alpha = read_float(self, 'trail_alpha')

        markers = []

        # Covariance ellipsoid, drawn before the point so the point lands on top of it.
        values, vectors = principal_axes(entry['covariance'])
        if values[0] > 0.0:
            ellipsoid = self._marker(base + ELLIPSOID, Marker.SPHERE, stamp)
            ellipsoid.pose.position = _point(position)
            x, y, z, w = quaternion_from_rotation_matrix(vectors)
            ellipsoid.pose.orientation = Quaternion(x=x, y=y, z=z, w=w)
            # A SPHERE's scale is its diameter along each of its own (here rotated) axes,
            # so 2 * n_sigma * sqrt(eigenvalue) is the axis extent that corresponds to
            # n_sigma standard deviations. Dropping the 2 shows half the error bar.
            ellipsoid.scale.x = max(2.0 * n_sigma * math.sqrt(values[0]), 1e-4)
            ellipsoid.scale.y = max(2.0 * n_sigma * math.sqrt(values[1]), 1e-4)
            ellipsoid.scale.z = max(2.0 * n_sigma * math.sqrt(values[2]), 1e-4)
            ellipsoid.color.r, ellipsoid.color.g, ellipsoid.color.b = colour
            # Semi-transparent by default: the ellipsoid's whole job is to show how big
            # the uncertainty is, and an opaque one hides the target it describes.
            ellipsoid.color.a = covariance_alpha
            markers.append(ellipsoid)

        point = self._marker(base + POINT, Marker.SPHERE, stamp)
        point.pose.position = _point(position)
        point.scale.x = point.scale.y = point.scale.z = max(diameter, 1e-4)
        point.color.r, point.color.g, point.color.b = colour
        # A coasting (predicted) bearing is shown by transparency, not by a different
        # hue: the colour identifies *which* target this is, and using it for two things
        # means you can no longer tell a tracked target from a predicted one at a glance.
        point.color.a = 0.4 if entry['predicted'] else 1.0
        markers.append(point)

        if read_int(self, 'show_range_line'):
            line = self._marker(base + RANGE_LINE, Marker.LINE_LIST, stamp)
            line.points.append(_point((0.0, 0.0, 0.0)))
            line.points.append(_point(position))
            line.scale.x = max(trail_width * 0.5, 1e-4)
            line.color.r, line.color.g, line.color.b = colour
            line.color.a = read_float(self, 'range_line_alpha')
            markers.append(line)

        if len(entry['points']) > 1:
            markers.append(self._trail_marker(base + TRAIL, entry['points'], colour,
                                              trail_width, trail_alpha, stamp))

        if read_int(self, 'show_labels'):
            label = self._marker(base + LABEL, Marker.TEXT_VIEW_FACING, stamp)
            label.pose.position = _point((position[0], position[1],
                                          position[2] + diameter * 1.5))
            label.scale.z = max(diameter * 1.2, 0.08)
            label.text = f'id {target_id}  {entry["distance"]:.2f} m'
            if entry['predicted']:
                label.text += '  (predicted)'
            label.color.r, label.color.g, label.color.b = colour
            label.color.a = 1.0
            markers.append(label)

        return markers

    def _trail_marker(self, marker_id, points, colour, width, alpha, stamp):
        """The recorded path as one LINE_STRIP, oldest end dimmest.

        The fade carries the direction of travel and the age of any jump, which a uniform
        line cannot show. It is done in brightness rather than alpha because RViz ignores
        the alpha of per-point colours - see TRAIL_TAIL_BRIGHTNESS.
        """
        marker = self._marker(marker_id, Marker.LINE_STRIP, stamp)
        marker.points = [_point(p) for p in points]
        # Marker.msg requires either zero colours or exactly one per point.
        marker.colors = [_colour(colour, alpha, TRAIL_TAIL_BRIGHTNESS +
                                 (1.0 - TRAIL_TAIL_BRIGHTNESS) * index / (len(points) - 1))
                         for index in range(len(points))]
        marker.scale.x = max(width, 1e-4)
        # Still needed: with no single colour set, a display that falls back to it would
        # render the strip invisible.
        marker.color.a = alpha
        return marker

    # ---- marker plumbing ----------------------------------------------------

    def _marker(self, marker_id, marker_type, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.camera_frame
        # One namespace for everything, so DELETEALL clears exactly what this node drew
        # and nothing another publisher happens to share an id with.
        marker.ns = MARKER_NAMESPACE
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        # 0 means "never expires", which is what is wanted: this node owns deletion, via
        # the explicit DELETE pass in publish_markers. Relying on a short lifetime
        # instead makes the display blink whenever the draw rate slips below the expiry.
        #
        # Left at the message's own default rather than assigned. `lifetime` is a
        # builtin_interfaces Duration, and rclpy.duration.Duration - which is what
        # get_clock().now() hands back - is a *different* class. Assigning one into the
        # other does not raise in Python; it reaches the C converter and aborts the
        # process with an assertion in librcutils, which is a very hard crash to trace
        # back to a line that looks like it merely writes a zero.
        marker.frame_locked = False
        return marker

    def _delete_marker(self, marker_id, stamp):
        marker = self._marker(marker_id, Marker.SPHERE, stamp)
        marker.action = Marker.DELETE
        return marker


MARKER_NAMESPACE = 'mrs_ultraloc'


def _point(position):
    point = Point()
    point.x, point.y, point.z = (float(position[0]), float(position[1]), float(position[2]))
    return point


def _colour(rgb, alpha, brightness=1.0):
    out = ColorRGBA()
    out.r = float(rgb[0]) * brightness
    out.g = float(rgb[1]) * brightness
    out.b = float(rgb[2]) * brightness
    out.a = min(1.0, max(0.0, float(alpha)))
    return out


def _axis_quaternion(direction):
    """Quaternion taking +x onto one of the three axes.

    Only the three axis directions are supported, which is all the camera arrows need;
    a general version is not worth having while the callers are a fixed set of three.
    """
    # Half-angle form: sin(45 deg) = sqrt(2)/2 = sqrt(0.5) in the two non-identity cases.
    half = math.sqrt(0.5)
    if direction == (0.0, 1.0, 0.0):
        # +x -> +y, a quarter turn about +z.
        return Quaternion(z=half, w=half)
    if direction == (0.0, 0.0, 1.0):
        # +x -> +z, a quarter turn about -y.
        return Quaternion(y=-half, w=half)
    return Quaternion(w=1.0)


def main(args=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = None
    try:
        node = SingleCameraMarker()
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
