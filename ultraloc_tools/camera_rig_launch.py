"""The shared body of the three rig entry-point launch files.

``one_cam``, ``two_cams`` and ``three_cams.launch.py`` are three names for one launch
description that differs in one string. Keeping that description in three copies is how
they drift - a pass-through argument added to two of them and forgotten in the third is a
launch file that silently drops an option, which is exactly the kind of thing that gets
noticed on the vehicle rather than at the desk.

So each launch file is three lines of this, and the difference between the rigs is the
slot list it passes in.

``camera_rig.launch.py`` remains the file that decides what runs and why; this module only
forwards arguments to it. It lives in ``ultraloc_tools`` rather than beside the launch
files because that is the one directory that is importable from a launch file - the launch
directory itself is not on ``sys.path`` - which ``launch/`` files discover through the
Python environment hook the package installs.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def rig_launch_description(slots, what_this_rig_is):
    """The launch description for the rig made of ``slots``.

    :param slots: comma-separated slot names, normally one of this package's ``RIG_*``
        constants so that the name a vehicle flies and the list the bringup receives are
        the same object.
    :param what_this_rig_is: one clause for ``cameras``' help text, e.g. "the front and
        the aft camera". It appears in ``ros2 launch ... --show-args``, which is what a
        person reads when they have this file open and the other two closed.
    """

    return LaunchDescription([

        # Every argument below is declared so it can be *named* on this launch file's
        # command line, and passed through unchanged rather than defaulted a second time:
        # camera_rig.launch.py owns each default, including the UAV_NAME fallback. A
        # default copied here would be a second answer to the same question, and the one
        # that wins is whichever file launch happens to consult first.
        DeclareLaunchArgument(
            'uav_name',
            default_value=EnvironmentVariable('UAV_NAME', default_value='uav'),
            description='Namespace for every node, and the prefix of every TF frame.',
        ),
        DeclareLaunchArgument(
            'cameras',
            default_value=slots,
            description=f'The camera slots to start ({what_this_rig_is}), comma-separated, '
                        f'in the order they are started. Each must be described in '
                        f'cameras_config; overriding this is how a camera is left out for '
                        f'a bench test without editing the rig file.',
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
            'dry_run',
            default_value='false',
            description='Resolve and print the rig and the generated uvdar configuration, '
                        'start nothing. The way to check a rig before its hardware is on.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=EnvironmentVariable('USE_SIM_TIME', default_value='false'),
            description='Follow /clock, so this works against a bag.',
        ),
        DeclareLaunchArgument(
            'uwb_uvdar_id_pairs',
            default_value='',
            description='Override the UWB:UVDAR pairing on every fusion node, e.g. '
                        '"0x13:3,0x15:0". Empty leaves config/uwb_uvdar_fusion.yaml in '
                        'charge.',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('mrs_ultraloc'), 'launch', 'camera_rig.launch.py',
            ])),
            launch_arguments={
                'uav_name': LaunchConfiguration('uav_name'),
                'cameras': LaunchConfiguration('cameras'),
                'cameras_config': LaunchConfiguration('cameras_config'),
                'dry_run': LaunchConfiguration('dry_run'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'uwb_uvdar_id_pairs': LaunchConfiguration('uwb_uvdar_id_pairs'),
            }.items(),
        ),
    ])
