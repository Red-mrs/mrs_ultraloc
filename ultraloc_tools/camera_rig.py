"""The camera rig: one description of the cameras, resolved once, used by everything.

This module answers three questions and nothing else, because each answer has to be the
same one in several places that cannot see each other:

  * *Which cameras does this vehicle have?* ``config/cameras.yaml``, one entry per slot.
  * *Where do their serial, exposure, calibration and mask come from?* Environment
    variables, named in that file rather than valued there, and file-name patterns in
    which only the serial changes.
  * *What does ``uvdar_core`` have to be told to see N cameras as one endpoint?* The
    generated detector/tracker/bearing config.

The reason it is a module and not launch-file code is that the third answer is a nested
dictionary built by one loop over the slots, and the failure mode of getting one string
in it wrong is not an error. ``uvdar_core`` never compares its stages' topic names:
``detector.inputs[S].output_topic`` and ``tracking.inputs[S].input_topic`` are each
required and each used, and a mismatch makes the tracker subscribe to a topic nobody
publishes, so the whole chain starts, announces itself, and stays silent forever. That is
only checkable by having all three stages' topics in one place, which is what
:func:`build_uvdar_config` is. The ``launch/*.launch.py`` files perform arguments and
start processes; the naming lives here so it can be read, and asserted on, without
starting ROS.

Slot names are the topic-and-TF names. For slot ``S`` and namespace ``<uav>``:

===============================================  ==========================================
``<uav>/<S>/bluefox_<S>/image_raw``              the camera, published by node ``bluefox_<S>``
``<uav>/uvdar/detector/<S>/points_seen``         detector output, one of N in one process
``<uav>/uvdar/tracker/<S>/blinkers``             tracker output
``<uav>/uvdar/bearing/<S>/observations``         bearing output, one of N in one process
``<uav>/<S>``                                    the camera's TF frame
``<uav>/<S>/uwb_uvdar_fusion/targets``           this slot's fusion output
===============================================  ==========================================

The first three are *relative* in the generated config and resolve against ``<uav>``,
which is what ``uvdar_core/config/default_bearing.yaml:11`` documents as the intended
contract and what lets the config name no UAV at all.
"""

import copy
import os

from dataclasses import dataclass, field

# Every stage's topics carry this prefix inside the UAV namespace.
#
# The two shipped uvdar configs disagree about it, which is the concrete argument for
# generating rather than editing: in default_bluefox.yaml, `detector.inputs[name: left]`
# publishes `detector/left/points_seen` (line 22) while `tracking.inputs[name: left]`
# subscribes to `uvdar/detector/left/points_seen` (line 90) - a topic nothing publishes,
# so the shipped two-camera left chain would never deliver a point. The `right` entries
# on the same two lines both use the prefix and do connect. One loop over the slots
# cannot produce that divergence.
TOPIC_PREFIX = "uvdar"

# The frame a slot's bearings are expressed in, relative to the UAV namespace. Named
# after the slot and nothing else, because the mount's roll already carries the
# body->optical rotation, so no `_optical_frame` suffix is needed to tell the frames
# apart - there is exactly one frame per camera. See DEFAULT_MOUNT for that roll.
FRAME_TEMPLATE = "{uav}/{slot}"

#: Where a slot's fusion node publishes, relative to that node.
#:
#: Relative on purpose: the node's namespace is the slot's, so the output moves with the
#: slot and one argument changes both. config/uwb_uvdar_fusion.yaml carries the same
#: string; it is repeated here because a launch file needs the absolute name to log and
#: to hand to a viewer, and deriving it from two sources is how a log line starts
#: pointing at a topic that does not exist.
TARGETS_TOPIC = "uwb_uvdar_fusion/targets"

#: The rigs, as comma-separated slot lists, in the order the cameras should be started.
#:
#: Here rather than in the launch files so that `one_cam.launch.py` and the bringup it
#: includes read one definition. A wrapper that disagreed with the thing it wraps about
#: which cameras exist would launch a rig whose name means something else, which is the
#: kind of thing nobody notices until two vehicles are flying different configurations
#: under the same name.
RIG_ONE = "camera"
RIG_TWO = "camera_front,camera_back"
RIG_THREE = "camera_left,camera_right,camera_back"

# The six numbers a static camera mount needs, with the convention they are in.
MOUNT_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")

#: A camera looking horizontally forward, in the body->optical convention: +x along the
#: optical axis, +z down, +y left.
#:
#: The -90 degree roll is not a measured angle and must not be "corrected" to match a
#: bracket: it is what makes a frame named ``camera_left`` an optical frame rather than a
#: body frame, and it is what uvdar_core's own mount in `two_bluefox.launch.py:52` uses.
#: Changing it silently rotates every bearing.
DEFAULT_MOUNT = {"x": 0.03, "y": 0.0, "z": 0.06, "roll": -1.57079632679,
                 "pitch": 0.0, "yaw": 0.0}


@dataclass
class Camera:
    """One slot, fully resolved to concrete values.

    Every field is the value a consumer needs, already namespaced and already absolute,
    so that no launch file or generated config has to re-derive any of them.
    """

    #: The slot name, which is the uvdar input name and the middle of every topic.
    slot: str
    #: USB serial of the bluefox on this mount, from ``serial_env``.
    serial: str
    #: Exposure in microseconds, from ``exposure_env``.
    exposure_us: str
    #: True when the exposure came from the file rather than the environment.
    exposure_defaulted: bool
    #: Environment variable names, kept so a warning can name what to set.
    serial_env: str
    exposure_env: str
    #: The vehicle namespace, kept because the absolute form of every topic below is
    #: this plus the relative name.
    uav_name: str
    #: TF frame for this camera - the driver's, the mount's child, and the bearing's.
    frame: str
    #: Absolute path, because uvdar resolves a relative one against the *generated*
    #: config in /tmp, not against this package.
    calib_file: str
    #: Absolute path, or "" for no mask. See `resolve_rig` for why empty is meaningful.
    mask_file: str
    #: Namespace of the camera node and of this slot's fusion node.
    namespace: str
    #: ``bluefox_<slot>``, the node name uvdar_core's single_bluefox.launch.py builds.
    node_name: str
    mount: dict = field(default_factory=dict)

    # ---- topics ---------------------------------------------------------------
    #
    # Two forms of each, and the difference is load-bearing. The three uvdar stages run
    # in the *vehicle* namespace with one input per camera, so the names written into the
    # generated config are relative to that and must not repeat the vehicle name - a
    # leading `<uav>/` here would resolve to `/uav13/uav13/...` and the chain would start
    # silent. The fusion node, in contrast, lives one level down in the slot's own
    # namespace and cannot reach a relative name into the vehicle namespace, so it is
    # given the absolute form. Both come from the same two strings below.

    @property
    def image_topic(self):
        """The camera's image topic, relative to the uvdar stages' namespace.

        ``<slot>/bluefox_<slot>/image_raw``, which is what
        ``default_bluefox.yaml:21`` does for its ``left`` camera and what the camera
        driver actually produces: the node lands in ``<uav>/<slot>`` (its
        ``camera_name``-derived namespace) and bluefox2 advertises ``~/image_raw``
        (``include/bluefox2/camera_ros_base.h:46``) - in the *node's* namespace, so the
        node name is part of the topic. That is the one topic string that is not a free
        choice, and the reason the shape looks redundant.
        """
        return f'{self.slot}/{self.node_name}/image_raw'

    @property
    def detector_output_topic(self):
        return f'{TOPIC_PREFIX}/detector/{self.slot}/points_seen'

    @property
    def tracker_input_topic(self):
        return self.detector_output_topic

    @property
    def tracker_output_topic(self):
        return f'{TOPIC_PREFIX}/tracker/{self.slot}/blinkers'

    @property
    def bearing_input_topic(self):
        return self.tracker_output_topic

    @property
    def bearing_output_topic(self):
        return f'{TOPIC_PREFIX}/bearing/{self.slot}/observations'

    @property
    def absolute_image_topic(self):
        return f'/{self.uav_name}/{self.image_topic}'

    @property
    def absolute_bearing_topic(self):
        return f'/{self.uav_name}/{self.bearing_output_topic}'

    @property
    def targets_topic(self):
        """This slot's fusion output, absolute - the name a viewer subscribes to."""
        return f'/{self.namespace}/{TARGETS_TOPIC}'

    def describe(self, uwb_topic):
        """A block of human-readable lines per camera, for the dry run and startup log.

        Absolute topic names throughout, because these lines get pasted into
        ``ros2 topic echo`` - the relative names the config file uses would not run.
        """
        mask = self.mask_file or '(none, masking off)'
        exposure = self.exposure_us + ('' if not self.exposure_defaulted
                                       else f'  [default; {self.exposure_env} unset]')
        prefix = f'/{self.uav_name}/'
        lines = [
            f'{self.slot}: serial {self.serial} (from ${self.serial_env}), '
            f'exposure {exposure} us',
            f'    frame     {self.frame}   mount '
            + ' '.join(f'{key}={self.mount[key]:g}' for key in MOUNT_KEYS),
            f'    image     {self.absolute_image_topic}',
            f'    detector  {prefix}{self.detector_output_topic}',
            f'    tracker   {prefix}{self.tracker_output_topic}',
            f'    bearing   {self.absolute_bearing_topic}',
            f'    targets   {self.targets_topic}   <- bearing + {uwb_topic}',
            f'    calib     {self.calib_file}',
            f'    mask      {mask}',
        ]
        return '\n'.join(lines)


@dataclass
class Rig:
    """A resolved rig: its cameras, and everything questionable found on the way."""

    uav_name: str
    cameras: list
    #: UWB module serial, or "" to leave that driver's own config in charge.
    uwb_serial: str
    #: ``(level, text)`` pairs; "error" entries only exist when resolution was strict.
    issues: list = field(default_factory=list)

    @property
    def slots(self):
        return [camera.slot for camera in self.cameras]

    def errors(self):
        return [text for level, text in self.issues if level == 'error']

    def warnings(self):
        return [text for level, text in self.issues if level == 'warning']


class RigConfigError(ValueError):
    """The rig file itself is wrong - a typo, an unknown slot, a missing pattern key.

    Raised rather than collected, because these are not conditions a rig can run under:
    a slot name that does not exist in cameras.yaml means the launch file asked for a
    camera nobody described, and continuing would start a chain with a name in it that
    nothing else knows.
    """


# ---------------------------------------------------------------------------
# reading the file
# ---------------------------------------------------------------------------

def load_rig_config(cameras_yaml):
    """Read ``config/cameras.yaml`` into plain dicts, with the defaults filled in.

    Deliberately ignorant of which slots are wanted: the file describes every camera the
    vehicle can wear, and the launch file picks a subset. Keeping the two apart means
    adding a rig costs no edit to the file.
    """
    import yaml

    if not os.path.isfile(cameras_yaml):
        raise RigConfigError(
            f'camera rig file \'{cameras_yaml}\' does not exist. It is installed under '
            f'share/mrs_ultraloc/config/cameras.yaml; a launch file defaults to that path.')

    with open(cameras_yaml) as handle:
        document = yaml.safe_load(handle)

    if not isinstance(document, dict):
        raise RigConfigError(f'camera rig file \'{cameras_yaml}\' is empty or not a mapping')

    cameras = document.get('cameras')
    if not isinstance(cameras, dict) or not cameras:
        raise RigConfigError(
            f'camera rig file \'{cameras_yaml}\' needs a non-empty \'cameras\' mapping of '
            f'slot name to camera settings')

    paths = document.get('paths') or {}
    if not isinstance(paths, dict):
        raise RigConfigError(f'\'paths\' in \'{cameras_yaml}\' must be a mapping')

    for key in ('calib_file', 'mask_file'):
        if key not in paths:
            raise RigConfigError(
                f'\'paths.{key}\' is missing from \'{cameras_yaml}\'. Calibration and mask '
                f'names are derived from the serial, so the pattern has to be stated; '
                f'write an explicit constant (no {{serial}}) to give every camera the same '
                f'file.')

    return {
        'base_dir': os.path.dirname(os.path.abspath(cameras_yaml)),
        'paths': paths,
        'uwb': document.get('uwb') or {},
        'cameras': cameras,
    }


def _camera_settings(slot, described, cameras_yaml):
    """Validate one slot's entry and return it with defaults applied."""
    if isinstance(described, dict):
        settings = dict(described)
    elif described is None:
        # `camera_back:` with nothing under it, which is how a person writes "the
        # defaults are fine". Anything else non-mapping is a typo worth naming.
        settings = {}
    else:
        raise RigConfigError(
            f'cameras.{slot} in \'{cameras_yaml}\' must be a mapping or empty, got '
            f'{described!r}')

    for key in ('serial_env', 'exposure_env'):
        value = settings.get(key)
        if not isinstance(value, str) or not value.strip():
            raise RigConfigError(
                f'cameras.{slot}.{key} must name an environment variable '
                f'(e.g. CAMERA_LEFT), got {value!r}')
        settings[key] = value.strip()

    mount = dict(DEFAULT_MOUNT)
    override = settings.get('mount') or {}
    if not isinstance(override, dict):
        raise RigConfigError(
            f'cameras.{slot}.mount must be a mapping of {list(MOUNT_KEYS)}, got '
            f'{override!r}')
    for key, raw in override.items():
        if key not in MOUNT_KEYS:
            raise RigConfigError(
                f'cameras.{slot}.mount has unknown key \'{key}\'; the accepted keys are '
                f'{list(MOUNT_KEYS)}')
        try:
            mount[key] = float(raw)
        except (TypeError, ValueError) as exc:
            raise RigConfigError(
                f'cameras.{slot}.mount.{key} must be a number, got {raw!r}') from exc

    settings['mount'] = mount
    # '' is a real value here, meaning "no mask for this camera", and it survives the
    # pattern below only if it is distinguished from "unset".
    settings['mask_override'] = settings.get('mask_file')
    return settings


def mrs_id_from_environment(environ=None):
    """The vehicle number used in mask file names.

    ``MRS_ID`` when set; otherwise ``UAV_NAME`` with a leading ``uav`` removed, which is
    the MRS convention (``uav13`` is vehicle 13) and saves exporting the same number
    twice under two names. The stripping is deliberately conservative: only the literal
    ``uav`` prefix, so a name like ``rover3`` is passed through untouched rather than
    mangled into ``over3``.
    """
    environ = os.environ if environ is None else environ
    explicit = environ.get('MRS_ID', '').strip()
    if explicit:
        return explicit
    uav_name = environ.get('UAV_NAME', '').strip()
    if uav_name.startswith('uav') and len(uav_name) > 3:
        return uav_name[3:]
    return uav_name


# ---------------------------------------------------------------------------
# resolving a rig
# ---------------------------------------------------------------------------

def resolve_rig(cameras_yaml, slots, uav_name, environ=None, require_calib=True):
    """Resolve the named slots into concrete values.

    :param slots: slot names in the order the cameras should be started. Each must appear
        under ``cameras`` in the file.
    :param require_calib: when False, a missing calibration is a warning instead of an
        error. Only sane for a dry run: see the note in the loop below.
    :raises RigConfigError: for a problem in the file or in ``slots``, which no run mode
        can proceed past.
    """
    environ = os.environ if environ is None else environ
    config = load_rig_config(cameras_yaml)
    base_dir = config['base_dir']

    if not uav_name.strip():
        raise RigConfigError('the vehicle namespace is empty; pass uav_name:=<name>')

    unknown = [slot for slot in slots if slot not in config['cameras']]
    if unknown:
        raise RigConfigError(
            f'{unknown} is/are not camera slots of \'{cameras_yaml}\'; the described '
            f'slots are {sorted(config["cameras"])}')
    duplicates = sorted({slot for slot in slots if slots.count(slot) > 1})
    if duplicates:
        # Two slots on one serial would be two processes opening one device, and the
        # driver's failure to do so looks like a broken camera rather than a duplicated
        # argument.
        raise RigConfigError(f'camera slot(s) {duplicates} requested more than once')

    mrs_id = mrs_id_from_environment(environ)
    issues = []
    cameras = []
    seen_serials = {}

    for slot in slots:
        settings = _camera_settings(slot, config['cameras'][slot], cameras_yaml)
        serial_env = settings['serial_env']
        exposure_env = settings['exposure_env']

        serial = environ.get(serial_env, '').strip()
        if not serial:
            issues.append((
                'error',
                f'cameras.{slot}: environment variable {serial_env} is unset, so this '
                f'camera has no serial to open and no file name to load. Set it in '
                f'~/.bashrc, e.g. export {serial_env}=<serial as bluefox2_list_cameras '
                f'prints it>'))
            # Keep going rather than raising: one report listing every unset variable is
            # more use on a bench than the first one alone, and the caller aborts on the
            # error list anyway. The empty serial then skips the file checks below, which
            # would otherwise complain about a path containing '<unset>'.
        elif serial in seen_serials:
            issues.append((
                'error',
                f'cameras.{slot} and cameras.{seen_serials[serial]} both use serial '
                f'\'{serial}\'; one device cannot be opened by two camera nodes'))
        else:
            seen_serials[serial] = slot

        exposure = environ.get(exposure_env, '').strip()
        exposure_defaulted = not exposure
        if exposure_defaulted:
            issues.append(('warning',
                           f'cameras.{slot}: {exposure_env} is unset; the camera driver\'s '
                           f'own default exposure will be used'))
            exposure = '2000'

        try:
            calib_file = _expand(config['paths']['calib_file'], serial, mrs_id, slot)
        except KeyError as exc:
            raise RigConfigError(
                f'paths.calib_file uses an unknown placeholder {exc}; the available ones '
                f'are {{serial}}, {{mrs_id}} and {{slot}}') from exc
        calib_file = _make_absolute(calib_file, base_dir)

        override = settings.get('mask_override')
        if isinstance(override, str):
            # An explicit `mask_file` on the camera beats the pattern, including the
            # explicit empty string that means "never mask this one".
            mask_file = _make_absolute(override, base_dir) if override else ''
        else:
            try:
                mask_file = _expand(config['paths']['mask_file'], serial, mrs_id, slot)
            except KeyError as exc:
                raise RigConfigError(
                    f'paths.mask_file uses an unknown placeholder {exc}; the available '
                    f'ones are {{serial}}, {{mrs_id}} and {{slot}}') from exc
            mask_file = _make_absolute(mask_file, base_dir)

        if serial:
            if not os.path.isfile(calib_file):
                detail = (f'cameras.{slot}: calibration file \'{calib_file}\' (from '
                          f'\'{config["paths"]["calib_file"]}\' with serial {serial}) does '
                          f'not exist')
                # A missing calibration is fatal on a real start because uvdar_core
                # reaches it through a `requireScalar` deep inside the lens-model loader,
                # so the bearing node aborts with a message about the lens model that
                # names neither this slot nor this path. In a dry run it must not be
                # fatal: the derived name is a guess about files that may not be on this
                # machine yet, and the dry run's whole purpose is to show what the guess
                # resolved to so it can be corrected.
                issues.append(('error' if require_calib else 'warning', detail + (
                    '' if require_calib else
                    ' (not checked strictly because dry_run; a real start refuses)')))

            if mask_file and not os.path.isfile(mask_file):
                # Deliberately not fatal, unlike the calibration. An empty mask_file is
                # the supported way to disable masking (default.yaml:40-41), while
                # detector_node.cpp:47-49 throws on a non-empty one that does not exist -
                # so passing the derived name through would turn "nobody has made a mask
                # for this camera yet" into a detector that will not start.
                issues.append(('warning',
                               f'cameras.{slot}: mask \'{mask_file}\' does not exist; '
                               f'masking disabled for this camera'))
                mask_file = ''

        cameras.append(Camera(
            slot=slot,
            serial=serial,
            exposure_us=exposure,
            exposure_defaulted=exposure_defaulted,
            serial_env=serial_env,
            exposure_env=exposure_env,
            uav_name=uav_name,
            frame=FRAME_TEMPLATE.format(uav=uav_name, slot=slot),
            calib_file=calib_file,
            mask_file=mask_file,
            namespace=f'{uav_name}/{slot}',
            node_name=f'bluefox_{slot}',
            mount=settings['mount']))

    uwb_serial_env = (config['uwb'].get('serial_env') or 'UWB_SERIAL').strip()
    uwb_serial = environ.get(uwb_serial_env, '').strip()
    if not uwb_serial:
        # Not an error: uwb_driver's own config file carries a serial, and leaving this
        # one empty is what lets that file be the authority. uwb_driver.launch.py:17-22
        # has the same behaviour for the same reason.
        issues.append(('warning',
                       f'{uwb_serial_env} is unset; the UWB driver\'s own config file '
                       f'stays in charge of which module to open'))

    return Rig(uav_name=uav_name, cameras=cameras, uwb_serial=uwb_serial, issues=issues)


def _uvdar_config_dir():
    """``share/uvdar_core/config``, for patterns that want uvdar_core's own files.

    Exists because uvdar_core ships real Bluefox OCam calibrations
    (``camera/bluefox_ocam_calib/bf_uv_<serial>.yaml``) and a rig should be able to reach
    them without hard-coding an install prefix, which differs between a bench checkout and
    the vehicle. A rig that has calibrated its own cameras replaces the pattern with a path
    relative to cameras.yaml.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('uvdar_core'), 'config')
    except Exception as exc:
        raise RigConfigError(
            'a paths.* pattern uses {uvdar_config}, which needs uvdar_core to be '
            f'installed and the workspace sourced; ament_index_python said: {exc}. Replace '
            'the pattern with a path relative to cameras.yaml to drop the dependency.') from exc


class _PatternFields(dict):
    """The values a ``paths.*`` pattern may name, with ``{uvdar_config}`` resolved lazily.

    A ``dict`` subclass with ``__missing__`` rather than six keyword arguments, so that a
    pattern mentioning ``{uvdar_config}`` is the only thing that triggers the package
    lookup - a rig pointing its calibs at its own directory then reads fine with
    uvdar_core not installed. An unknown name raises ``KeyError``, which the callers turn
    into a message about the pattern; a silent pass-through would put a literal
    ``{serila}`` into a path and the failure would surface in the lens-model loader.
    """

    def __init__(self, serial, mrs_id, slot):
        super().__init__(serial=serial, mrs_id=mrs_id, slot=slot)

    def __missing__(self, key):
        if key == 'uvdar_config':
            value = _uvdar_config_dir()
            self[key] = value
            return value
        raise KeyError(key)


def _expand(pattern, serial, mrs_id, slot):
    """Substitute the file-name patterns.

    ``format_map`` with :class:`_PatternFields` rather than ``str.format`` so an unknown
    placeholder - ``{serila}`` - raises here instead of reaching the lens-model loader as
    a literal directory name.
    """
    return pattern.format_map(_PatternFields(serial, mrs_id, slot))


def _make_absolute(path, base_dir):
    """Anchor a derived path at the rig file's directory.

    uvdar_core resolves a relative ``calib_file``/``mask_file`` against the directory of
    the config file that mentions it (``helpers/yaml.hpp:205-216``), and that file is the
    generated one in /tmp. Writing absolute paths here is what stops every derived path
    from silently pointing into /tmp.
    """
    if not path:
        return ''
    return path if os.path.isabs(path) else os.path.join(base_dir, path)


# ---------------------------------------------------------------------------
# the generated uvdar_core config
# ---------------------------------------------------------------------------

def build_uvdar_config(base_config_yaml, rig):
    """Rewrite ``base_config_yaml``'s three ``inputs`` lists to describe ``rig``.

    The base file supplies everything that is not per-camera - thresholds, tracker
    tuning, blink sequences - and one throwaway ``camera_0`` entry supplies the per-input
    defaults. Only the three ``inputs`` lists and two thread-pool sizes are rewritten, so
    a change to uvdar_core's tuning picks itself up on the next build rather than living
    in a second copy here.

    The one loop is the point: see the module docstring for what a mismatch between the
    stages costs, and note that it is *not* an error - the tracker just subscribes to
    nothing.
    """
    import yaml

    with open(base_config_yaml) as handle:
        config = yaml.safe_load(handle)

    if not isinstance(config, dict):
        raise RigConfigError(f'uvdar base config \'{base_config_yaml}\' is empty')

    for section in ('detector', 'tracking', 'bearing'):
        if not isinstance(config.get(section), dict):
            raise RigConfigError(
                f'uvdar base config \'{base_config_yaml}\' has no \'{section}\' section, '
                f'so there are no per-input defaults to copy')

    base_dir = os.path.dirname(os.path.abspath(base_config_yaml))
    count = len(rig.cameras)

    # One worker per input. Correctness only needs a non-zero pool
    # (helpers/thread_pool.cpp:13); sizing it to the input count is so that three
    # cameras' frames are not serialised through two threads. It stays inside each
    # stage's own limit, which is a throughput question rather than a limit.
    config['detector']['thread_pool_size'] = count
    config['tracking']['thread_pool_size'] = count

    # A sequence_file in the base file resolves against the base file, not the generated
    # one, so it has to be anchored before the generated file moves it into /tmp. An
    # empty one means the inline `sequences` below, which travel untouched.
    sequence_file = config['tracking'].get('sequence_file') or ''
    if sequence_file:
        config['tracking']['sequence_file'] = _make_absolute(sequence_file, base_dir)

    detector_template = _input_template(config['detector'], base_config_yaml)
    tracking_template = _input_template(config['tracking'], base_config_yaml)
    bearing_template = _input_template(config['bearing'], base_config_yaml)

    config['detector']['inputs'] = []
    config['tracking']['inputs'] = []
    config['bearing']['inputs'] = []

    for camera in rig.cameras:
        detector_input = dict(detector_template)
        detector_input['name'] = camera.slot
        detector_input['input_topic'] = camera.image_topic
        detector_input['output_topic'] = camera.detector_output_topic
        detector_input['sun_output_topic'] = camera.detector_output_topic + '/sun'
        detector_input['visualization_topic'] = (
            f'{TOPIC_PREFIX}/detector/{camera.slot}/visualization')
        # '' is a value, not an omission: it is what disables masking.
        detector_input['mask_file'] = camera.mask_file

        tracking_input = dict(tracking_template)
        tracking_input['name'] = camera.slot
        tracking_input['input_topic'] = camera.tracker_input_topic
        tracking_input['input_image_topic'] = camera.image_topic
        tracking_input['output_topic'] = camera.tracker_output_topic
        tracking_input['visualization_topic'] = (
            f'{TOPIC_PREFIX}/tracker/{camera.slot}/visualization')

        bearing_input = dict(bearing_template)
        bearing_input['name'] = camera.slot
        bearing_input['input_topic'] = camera.bearing_input_topic
        bearing_input['output_topic'] = camera.bearing_output_topic
        # Stamped verbatim into every observation's header.frame_id by
        # bearing_node.cpp:135, so this string and the mount's child frame and the
        # driver's frame_id have to be one string. They are, because all three read
        # `Camera.frame`.
        bearing_input['camera_frame'] = camera.frame
        bearing_input['calib_file'] = camera.calib_file

        config['detector']['inputs'].append(detector_input)
        config['tracking']['inputs'].append(tracking_input)
        config['bearing']['inputs'].append(bearing_input)

    return config


def _input_template(section, base_config_yaml):
    """The base config's single example input, to copy per-camera values from."""
    inputs = section.get('inputs')
    if not isinstance(inputs, list) or not inputs:
        raise RigConfigError(
            f'uvdar base config \'{base_config_yaml}\' needs a non-empty \'inputs\' list '
            f'to take per-camera defaults from')
    if not isinstance(inputs[0], dict):
        raise RigConfigError(
            f'uvdar base config \'{base_config_yaml}\' has a non-mapping inputs[0]')
    return copy.deepcopy(inputs[0])


def dump_uvdar_config(config, destination):
    """Write the generated config and return its path.

    ``safe_dump`` with ``sort_keys=False``, because these files are read by people and
    the base file's comments-on-sections order is part of what makes them legible.

    One hazard worth naming: launch's own parameter files and uvdar_core's config are both
    written by PyYAML and read by yaml-cpp, and the two disagree about a scalar like
    ``2075E33814`` - PyYAML reads it as a string, yaml-cpp as a float, and the UWB driver
    aborts on the type mismatch (which is why config/uwb_uvdar_fusion.yaml's id pairs and
    this file's serials are always quoted or non-numeric). Nothing written here can take
    that shape: the floats are written by PyYAML from Python floats, which always emits a
    ``.`` or an exponent, and the strings go through ``safe_dump``'s own quoting. A
    round-trip of the shipped base config was checked for type drift before this was
    relied on.
    """
    import yaml

    with open(destination, 'w') as handle:
        yaml.safe_dump(config, handle, sort_keys=False, default_flow_style=False)
    return destination


def check_topic_chain(config):
    """Return a description of every break between the three stages, empty if sound.

    Not a formality, and not a check on our own arithmetic either: this is the only place
    the three stages' topics are compared, and uvdar_core will not do it. A base config
    with, say, a stage-specific prefix in one section's input would produce a silently
    dead chain here rather than a startup error, and would otherwise be found by watching
    for messages that never come.
    """
    breaks = []
    stages = (config['detector']['inputs'], config['tracking']['inputs'],
              config['bearing']['inputs'])

    for name in ('detector', 'tracking', 'bearing'):
        if not stages[{'detector': 0, 'tracking': 1, 'bearing': 2}[name]]:
            breaks.append(f'{name} has no inputs')

    for detector, tracking in zip(stages[0], stages[1]):
        if detector['output_topic'] != tracking['input_topic']:
            breaks.append(
                f'detector \'{detector["name"]}\' publishes \'{detector["output_topic"]}\' '
                f'but tracker subscribes \'{tracking["input_topic"]}\'')
        if detector['input_topic'] != tracking['input_image_topic']:
            breaks.append(
                f'detector \'{detector["name"]}\' reads \'{detector["input_topic"]}\' but '
                f'tracker reads image \'{tracking["input_image_topic"]}\'')

    for tracking, bearing in zip(stages[1], stages[2]):
        if tracking['output_topic'] != bearing['input_topic']:
            breaks.append(
                f'tracker \'{tracking["name"]}\' publishes \'{tracking["output_topic"]}\' '
                f'but bearing subscribes \'{bearing["input_topic"]}\'')

    names = ([entry['name'] for entry in stages[0]], [entry['name'] for entry in stages[1]],
             [entry['name'] for entry in stages[2]])
    if not names[0] == names[1] == names[2]:
        breaks.append(f'the three stages disagree on the input names: {names}')

    return breaks
