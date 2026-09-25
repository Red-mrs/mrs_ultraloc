#!/usr/bin/env python3

"""Bring up a vehicle's cameras, the UVDAR bearing endpoint, one UWB driver, and one
fusion node per camera.

This is the file the three rig launch files include; it is normally not called directly,
because the cameras it starts are named by the `cameras` argument and the thin wrappers
exist to name them per vehicle. Called directly it is still the most useful thing in the
package for seeing what a rig would do::

    ros2 launch mrs_ultraloc camera_rig.launch.py cameras:=camera dry_run:=true
    ros2 launch mrs_ultraloc camera_rig.launch.py \\
        cameras:=camera_left,camera_right,camera_back dry_run:=true

The three front ends are started from their own packages rather than re-declared here, so
this file cannot drift away from how each stage is meant to be started - the same reason
`pipeline.launch.py` included them, and the reason it is gone: it could only ever describe
one camera.

What is launched, for N cameras
-------------------------------

======================  ===========  ==================================================
stage                   processes    namespace
======================  ===========  ==================================================
bluefox2 camera         **N**        ``<uav>/<slot>``, node ``bluefox_<slot>``
static camera mount     **N**        ``<uav>/fcu`` -> ``<uav>/<slot>``
detector                1            ``<uav>`` - N inputs in one process
tracker                 1            ``<uav>``
bearing                 1            ``<uav>`` - N publishers in one process
uwb_driver              **1**        ``<uav>`` - one module, however many cameras
fusion node             **N**        ``<uav>/<slot>``
======================  ===========  ==================================================

The chain is not per-camera and that is not a compromise: ``uvdar_core``'s detector,
tracker and bearing each iterate over an ``inputs`` list (``detector_node.cpp:67-102``
builds one pipeline per entry) and its shipped ``default_bluefox.yaml:19-53`` gives the
detector two entries in one file. Starting N detectors would open one image topic N times
and put N publishers on one output topic. Fusion *is* per-camera: the node fuses one
bearing topic with one range topic - ``bearing_topic`` is a single string parameter, and
the frame it publishes in is the frame that one bearing was stamped with
(``uwb_uvdar_fusion_node.h:35-37``).

So ``camera_left`` and ``camera_right`` each get their own
``<uav>/<slot>/uwb_uvdar_fusion/targets``, and nothing merges them. That is not only the
node's shape but a real gap: a UWB range says nothing about which camera ought to be
seeing the target, so attributing a range to the camera that sees its owner is a decision,
not a transform. This file makes the streams exist and be nameable, which is all a
combination stage will need; it does not make the decision on that stage's behalf.

Everything comes from one description of the rig
------------------------------------------------

Which serial, which exposure, which calibration, which mask, which mount, which topics,
which frame: all of it is resolved once by
``ultraloc_tools/camera_rig.py`` from ``config/cameras.yaml``, and the launch files read
the result. Not for tidiness - a camera is named in five places that cannot see each
other (the driver's node name, the detector's input, the bearing's ``camera_frame``, the
mount's child frame, the fusion node's namespace) and a disagreement between them is not
an error. A ``camera_frame`` that does not match the mount's child frame leaves the
bearings' TF frame parentless, which RViz shows as targets stuck to the origin and
``ros2 topic echo`` shows as nothing at all.

The generated uvdar config is the same argument in one file's worth of strings: all three
stages' topic names are written by one loop over the slots, because uvdar_core never
compares them and a mismatch starts a chain that stays silent forever. Its
``tracking`` stage in the shipped two-camera config subscribes to
``uvdar/detector/left/points_seen`` while the ``detector`` stage publishes
``detector/left/points_seen`` (default_bluefox.yaml:22 vs :90) - the left camera in that
file has never delivered a point, and nothing said so.

Dry run
-------

``dry_run:=true`` resolves the rig, generates the config, prints both and starts nothing.
That is not the same as ``--print-description``, which prints the launch description
*without executing it*: everything here lives in one ``OpaqueFunction`` because the
argument set is cross-dependent, so ``--print-description`` would print an opaque
placeholder and resolve nothing. Use ``dry_run`` to check a rig, and start the real thing
once it reads right. It prints the generated file's path; pass
``keep_uvdar_config:=/path/to.yaml`` to have it written somewhere you can open afterwards.

In a dry run a missing calibration file warns instead of refusing, so a rig can be
inspected before its calibration files have been copied onto the machine. A real start
refuses - see ``resolve_rig``'s note on why a derived calibration path is checked here
rather than discovered inside the lens-model loader.

Without hardware
----------------

Nothing here needs a camera or a UWB module attached to come up, and what it looks like
when neither is attached is the thing to check first, because each of these is a stage
behaving correctly and together they look like a broken bringup:

* **Silent, not stuck.** detector, tracker and bearing start and stay quiet - a camera
  that cannot open its device publishes nothing, and a tracker seeing no LED pattern
  decodes no id. All of them staying up is the pass condition.
* **Every paired id reported once** as "Never received a UVDAR bearing for id ..." by
  each fusion node, at ``diagnostics_rate_hz``.
* **Bearings from a wrong calibration are numerically plausible.** The shipped default
  calibration pattern picks a file per serial out of uvdar_core's Bluefox set; two of
  those files differ in every coefficient and in nothing a version check would catch, so
  a wrong pick cannot crash the node. The only check is a known distance - see the
  static-hold test in README.md.
"""

import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Where the UWB driver publishes, relative to its namespace. It is relative in that
# package's own config, so it follows the namespace and only needs the prefix.
UWB_OUTPUT_TOPIC = 'uwb/distance'

# Seconds between one camera process and the next.
#
# Not a nicety: uvdar_core's single_bluefox.launch.py declares `node_start_delay` as "Node
# delay for multiple cameras (driver can crash if run multiple times in the same moment)"
# and then never uses the value, and its own two_bluefox.launch.py passes no delay at all.
# Two Bluefox drivers opening simultaneously has been the failure it was written against,
# and with three cameras the third one would otherwise join the pile-up. The stagger is
# implemented here rather than by forwarding that argument, because the argument does
# nothing and because the included launch file's own action tree is not ours to schedule.
CAMERA_START_STAGGER_SEC = 2.0


def generate_launch_description():

    uav_name = LaunchConfiguration('uav_name')

    return LaunchDescription([

        DeclareLaunchArgument(
            'uav_name',
            default_value=EnvironmentVariable('UAV_NAME', default_value='uav'),
            description='Namespace for every node here, and the prefix of every TF frame.',
        ),

        # No rig named as the default here. The three wrappers pass one in from
        # ultraloc_tools/camera_rig.py; a default chosen by this file would be a fourth
        # rig nobody asked for, and this file's honest default is "you tell me", since
        # calling it directly means you are choosing cameras deliberately.
        DeclareLaunchArgument(
            'cameras',
            default_value='',
            description='Comma-separated camera slots to start, in the order they should '
                        'be started. Each must be described in cameras_config. one_cam, '
                        'two_cams and three_cams.launch.py pass their own subset; empty is '
                        'an error rather than a guess.',
        ),

        DeclareLaunchArgument(
            'cameras_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('mrs_ultraloc'), 'config', 'cameras.yaml',
            ]),
            description='The rig description: per-camera environment variable names, file '
                        'name patterns and mounts.',
        ),

        DeclareLaunchArgument(
            'uvdar_base_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('uvdar_core'), 'config', 'default_bearing.yaml',
            ]),
            description='Detector/tracker/bearing configuration whose non-camera settings '
                        '(thresholds, tracker tuning, blink sequences) are kept, and whose '
                        'inputs are replaced by one entry per camera. Named uvdar_base_'
                        'config rather than config so that it cannot collide with the '
                        'argument uwb_driver.launch.py declares its own config under - see '
                        'the note on fusion_config below, which is the same trap.',
        ),

        DeclareLaunchArgument(
            'fusion_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('mrs_ultraloc'), 'config', 'uwb_uvdar_fusion.yaml',
            ]),
            description='Parameter file for each fusion node. Its bearing_topic and '
                        'uwb_topic are overridden below - they are per-camera, so a single '
                        'file cannot state them - and every other value in it applies to '
                        'every camera, including its own bearing_timeout_sec and sigmas.',
        ),

        DeclareLaunchArgument(
            'uwb_serial',
            # Empty by default so that an unset UWB_SERIAL leaves uwb_driver's own config
            # file in charge, which is that package's documented behaviour
            # (uwb_driver.launch.py:17-22) rather than a fallback invented here.
            default_value=EnvironmentVariable('UWB_SERIAL', default_value=''),
            description='USB serial of the UWB module, overriding that package\'s config '
                        'file. One module per vehicle however many cameras are started.',
        ),

        DeclareLaunchArgument(
            'standalone',
            default_value='true',
            description='Run uwb_driver as its own node instead of a component. This '
                        'default is load-bearing: as a component the driver is created '
                        'from the root namespace and ends up on /uwb_driver publishing '
                        '/uwb/distance, while every fusion node here listens under '
                        '/<uav> - a driver that is running, visible in `ros2 node list` '
                        'and fused by nothing.',
        ),

        DeclareLaunchArgument(
            'uwb_uvdar_id_pairs',
            default_value='',
            description='Comma-separated "uwb_address:uvdar_id" pairs overriding the config '
                        'file, e.g. "0x13:3,0x15:0". Applied to *every* fusion node here: '
                        'the pairing is a property of which vehicles are around, not of '
                        'which camera saw them. Empty leaves the file in charge.',
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=EnvironmentVariable('USE_SIM_TIME', default_value='false'),
            description='Follow /clock, so this works against a bag.',
        ),

        DeclareLaunchArgument(
            'camera_stagger_sec',
            default_value=str(CAMERA_START_STAGGER_SEC),
            description='Delay added between consecutive camera processes. 0 starts them '
                        'together, which is what the driver has been observed not to like.',
        ),

        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            description='Resolve the rig, generate and print the uvdar configuration, and '
                        'start nothing. See the module docstring for why this is not '
                        'redundant with --print-description.',
        ),

        DeclareLaunchArgument(
            'keep_uvdar_config',
            default_value='',
            description='Write the generated uvdar configuration here instead of a '
                        'temporary file, so it can be opened after the fact. A generated '
                        'file is not the file read while debugging, which is why the path '
                        'is always logged.',
        ),

        # uwb_driver.launch.py and uvdar_core's single_bluefox.launch.py both read UAV_NAME
        # from the *environment* - the driver for its namespace (uwb_driver.launch.py:59),
        # the camera file for its namespace default and for the TF frame_id it builds
        # (single_bluefox.launch.py:63,130) - and neither looks at the uav_name argument
        # they are handed. bearing.launch.py has a different fallback again (uav1). So one
        # argument on this launch file only reaches all four stages through here; without
        # it, a shell with UAV_NAME unset would namespace the driver by the shell and the
        # bearing stage by uav1 while the fusion nodes listened under uav_name, and the
        # symptom would be a pipeline that starts everything and fuses nothing.
        SetEnvironmentVariable(name='UAV_NAME', value=uav_name),

        OpaqueFunction(function=launch_rig),
    ])


def launch_rig(context, *args, **kwargs):
    """Resolve the rig, then start every process it implies - or none of them.

    One function, performing every argument once, for the same reason the sim launch file
    is written this way: the camera frame, the mount's child frame, the bearing's
    ``camera_frame`` and the fusion node's input topic all have to be the *same strings*,
    and performing them once here is what makes that a fact rather than a coincidence
    between four call sites.
    """

    from ultraloc_tools.camera_rig import RigConfigError
    from ultraloc_tools.camera_rig import build_uvdar_config
    from ultraloc_tools.camera_rig import check_topic_chain
    from ultraloc_tools.camera_rig import dump_uvdar_config
    from ultraloc_tools.camera_rig import resolve_rig

    def value(name, cast=str):
        return cast(perform_substitutions(context, [LaunchConfiguration(name)]).strip())

    uav_name = value('uav_name')
    slots = [entry.strip() for entry in value('cameras').split(',') if entry.strip()]
    cameras_config = value('cameras_config')
    base_config = value('uvdar_base_config')
    dry_run = value('dry_run').lower() in ('true', '1', 'yes', 'on')
    keep_at = value('keep_uvdar_config')

    if not slots:
        raise RigConfigError(
            'no cameras requested; `cameras` is empty. It takes a comma-separated list of '
            f'slot names described in \'{cameras_config}\'')

    try:
        rig = resolve_rig(cameras_config, slots, uav_name,
                          require_calib=not dry_run)
    except RigConfigError as exc:
        # An exception from here is printed by launch with a traceback and no context
        # about the rig, which is a poor way to learn that a name was misspelled. Say what
        # was asked for first.
        raise RigConfigError(
            f'cannot start the rig cameras={slots} from \'{cameras_config}\': {exc}') from exc

    # Depends on the namespace alone, so it is already final here even though the driver
    # that publishes it is started further down.
    uwb_topic = f'/{uav_name}/{UWB_OUTPUT_TOPIC}'

    # Serial rather than parallel deliberately: on a bench the first question is always
    # "what did it decide?", before anything else has had a chance to scroll the answer
    # away.
    report = [LogInfo(msg=f'[mrs_ultraloc] rig \'{uav_name}\', {len(rig.cameras)} camera(s):')]
    for camera in rig.cameras:
        report.append(LogInfo(msg=camera.describe(uwb_topic)))

    # Warnings first, errors last: the errors are the reason a start stops, so they should
    # be the last thing on the screen when it does.
    for text in rig.warnings():
        report.append(LogInfo(msg=f'[mrs_ultraloc] WARNING: {text}'))
    errors = rig.errors()

    config = build_uvdar_config(base_config, rig)

    # The one place the three stages' topics are compared, and the only defence against
    # the failure mode this whole file is arranged to avoid: uvdar_core will accept a
    # config whose stages do not agree and start every process in it, silent. A base
    # config with a stray per-stage prefix would otherwise be found by watching for
    # messages that never arrive.
    breaks = check_topic_chain(config)
    if breaks:
        errors.extend(f'generated uvdar config is internally broken: {entry}'
                      for entry in breaks)

    uvdar_config_path = dump_uvdar_config(config, keep_at or _temporary_config_path())
    report.append(LogInfo(
        msg=f'[mrs_ultraloc] generated uvdar config '
            f'({len(rig.cameras)} inputs in each of detector, tracking, bearing): '
            f'{uvdar_config_path}'))

    if errors:
        for text in errors:
            report.append(LogInfo(msg=f'[mrs_ultraloc] ERROR: {text}'))
        report.append(LogInfo(
            msg=f'[mrs_ultraloc] not starting: {len(errors)} problem(s) above. '
                f'dry_run:=true starts nothing and reports the same resolution, which is '
                f'the way to look over a rig before fixing it.'))
        if not dry_run:
            # Raising rather than returning the log actions: returning them starts
            # nothing anyway, but leaves the exit status up to launch's reading of an
            # empty action list, and a bringup that half-starts is worse than one that
            # refuses. In a dry run the errors are the *output*, so they only stop a real
            # start.
            raise RigConfigError('; '.join(errors))

    if dry_run:
        with open(uvdar_config_path) as handle:
            generated = handle.read()
        report.append(LogInfo(msg='[mrs_ultraloc] dry run, generated uvdar config:\n'
                              + generated))
        report.append(LogInfo(msg='[mrs_ultraloc] dry run complete, nothing started.'))
        return report

    actions = report

    # ---- cameras -------------------------------------------------------------
    #
    # Included from uvdar_core rather than declared here, so the ~20 driver parameters
    # (pixel format, binning, trigger mode, compression, the libusb LD_LIBRARY_PATH) stay
    # that package's to own. `two_bluefox.launch.py:20-41` is the same pattern one camera
    # deeper. The frame_id override is the one thing this file has to reach into it for -
    # see _write_frame_id_override.
    stagger = value('camera_stagger_sec', float)
    for index, camera in enumerate(rig.cameras):
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('uvdar_core'), 'launch', 'single_bluefox.launch.py',
            ])),
            launch_arguments={
                'uav_name': uav_name,
                'camera_name': camera.slot,
                'device': camera.serial,
                'expose_us': camera.exposure_us,
                # Manual exposure, so EXPOSE_US_* means what it says and no automatic
                # loop is adjusting the image under the detector. Same choice as
                # two_bluefox.launch.py:27.
                'aec': 'false',
                'standalone': 'true',
                'custom_config': _write_frame_id_override(camera),
            }.items(),
        )
        if stagger > 0.0 and index:
            actions.append(TimerAction(
                period=stagger * index, actions=[camera_launch]))
        else:
            actions.append(camera_launch)

    # ---- mounts --------------------------------------------------------------
    #
    # <uav>/fcu has to be published by something else - the state estimator on the
    # vehicle. Nothing here invents it, and the camera driver does not publish TF either,
    # so until the estimator runs <uav>/fcu is simply the *root* of the tree these
    # publishers form. That is not a broken tree and view_frames will not show two:
    # `lookup_transform` from a camera frame to <uav>/fcu works immediately, and what it
    # returns is the static mount. What it does not include is any motion, so a moving
    # vehicle reports every target at its position relative to where the estimator last
    # published, and RViz with Fixed Frame <uav>/fcu looks perfectly stable while showing
    # the wrong world. The sign that the estimator is missing is a static <uav>/fcu in
    # `ros2 topic echo /tf`, not an error from anything here.
    for camera in rig.cameras:
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{camera.slot}_tf',
            namespace=[uav_name],
            output='screen',
            arguments=[
                '--x', f'{camera.mount["x"]:.9f}',
                '--y', f'{camera.mount["y"]:.9f}',
                '--z', f'{camera.mount["z"]:.9f}',
                '--roll', f'{camera.mount["roll"]:.9f}',
                '--pitch', f'{camera.mount["pitch"]:.9f}',
                '--yaw', f'{camera.mount["yaw"]:.9f}',
                '--frame-id', f'{uav_name}/fcu',
                # The same string as the generated config's camera_frame and as the
                # driver's frame_id, because all three read Camera.frame. See the module
                # docstring for what a disagreement costs.
                '--child-frame-id', camera.frame,
            ],
        ))

    # ---- one bearing endpoint for all of them --------------------------------
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('uvdar_core'), 'launch', 'bearing.launch.py',
        ])),
        launch_arguments={
            'namespace': uav_name,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_file': uvdar_config_path,
        }.items(),
    ))

    # ---- one UWB driver, however many cameras --------------------------------
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('uwb_driver'), 'launch', 'uwb_driver.launch.py',
        ])),
        launch_arguments={
            'serial_number': LaunchConfiguration('uwb_serial'),
            'standalone': LaunchConfiguration('standalone'),
        }.items(),
    ))

    # ---- one fusion node per camera ------------------------------------------
    pairs = [entry.strip() for entry in value('uwb_uvdar_id_pairs').split(',')
             if entry.strip()]
    for camera in rig.cameras:
        parameters = [
            LaunchConfiguration('fusion_config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # Absolute and per-camera: this node sits one namespace below the bearing
                # endpoint, so a relative name resolves against the slot rather than the
                # vehicle. The config file pins both to one namespace and cannot be made
                # to describe N cameras, which is why they are overridden here rather
                # than corrected there.
                'bearing_topic': camera.absolute_bearing_topic,
                'uwb_topic': uwb_topic,
            },
        ]
        if pairs:
            # Only when non-empty, and as a plain list of str: the node rejects an empty
            # pairing outright, and an empty `[]` arriving from a YAML file aborts rclcpp
            # on Jazzy because the sequence carries no element type.
            parameters.append({'uwb_uvdar_id_pairs': pairs})

        actions.append(Node(
            package='mrs_ultraloc',
            executable='uwb_uvdar_fusion_node',
            name='uwb_uvdar_fusion',
            namespace=[camera.namespace],
            output='screen',
            parameters=parameters,
        ))

    actions.append(LogInfo(
        msg=f'[mrs_ultraloc] {len(rig.cameras)} fusion node(s) fusing '
            f'{uwb_topic} with one bearing stream each'
            + (f' on pairs {pairs}' if pairs else '')
            + '. Targets: '
            + ', '.join(camera.targets_topic for camera in rig.cameras)))

    return actions


def _temporary_config_path():
    """A path in the temp directory for the generated config.

    Not a `NamedTemporaryFile`: the file has to outlive this function, be openable by a
    second process reading it as a parameter file, and have a name short enough to read in
    a log line. ``delete=False`` is the part that matters - the process that would remove
    it is this one, and launch does not run clean-up handlers for a plain temp file.

    ``$TMPDIR`` is honoured by `tempfile` itself, so a machine that keeps /tmp on a tmpfs
    with a small quota is not a surprise here; a config file this size is not worth a
    special case.
    """
    handle, path = tempfile.mkstemp(prefix='mrs_ultraloc_uvdar_', suffix='.yaml')
    os.close(handle)
    return path


def _write_frame_id_override(camera):
    """A one-parameter file renaming this camera's TF frame.

    Exists because uvdar_core's single_bluefox.launch.py builds the driver's ``frame_id``
    itself - ``$UAV_NAME + '/bluefox' + '_' + camera_name``, line 130 - with no launch
    argument for it, and the bearing stage's ``camera_frame`` and the mount's child frame
    have to be that same string. ``custom_config`` is the supported door: the file is
    appended to the parameter list *after* the built-in dictionary (lines 166-168), and a
    later parameter source overrides an earlier one for an already-declared parameter,
    which was checked against rclcpp on this Jazzy install before it was relied on.

    The alternative, one launch argument in that file, is the cleaner change and would also
    drop the ``bluefox_`` prefix from the node and topic names; this way the rig needs
    nothing from uvdar_core.

    ``frame_id`` is a parameter the node declares itself (``src/single/single_main.cpp:41``)
    and reads once at construction (``include/bluefox2/camera_ros_base.h:54``), so overriding
    it has to happen before the node starts - which is what a parameter file is for. The
    file keys to ``/**`` rather than to a node name for the reason spelled out at the top of
    config/uwb_uvdar_fusion.yaml: a parameter file is matched against the node's fully
    qualified name, the node here lives at ``<uav>/<slot>/bluefox_<slot>``, and a key that
    matches nothing is silently ignored - no warning, and the frame stays ``bluefox_<slot>``.
    This file is only ever passed to this one camera, so matching any node is the right
    scope.

    The path is deterministic per vehicle and slot rather than random, so re-launching does
    not leave a new file behind each time; the contents are derived and rewritten every
    start.
    """
    path = os.path.join(tempfile.gettempdir(),
                        f'mrs_ultraloc_frame_{camera.uav_name}_{camera.slot}.yaml')
    with open(path, 'w') as handle:
        handle.write('# Written by mrs_ultraloc camera_rig.launch.py. Renames the camera\n'
                     '# driver\'s TF frame to the rig\'s frame for this slot, which is the\n'
                     '# frame the bearings are stamped with and the mount is published to.\n'
                     '/**:\n'
                     '  ros__parameters:\n'
                     f'    frame_id: {camera.frame}\n')
    return path
