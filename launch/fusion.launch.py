#!/usr/bin/env python3

"""Start the one fusion node for a vehicle, and nothing else.

    uvdar_core   bearing.launch.py            <ns>/uvdar/bearing/observations
    uwb_driver   uwb_driver.launch.py         <ns>/uwb/distance
    mrs_ultraloc uwb_uvdar_fusion_node        <ns>/uwb_uvdar_fusion/targets

This file assumes the two rows above it are already running, in their own tmux sessions
or not. It starts no camera, no detector, no tracker, no bearing stage and no UWB
driver - it only says which bearing and range topics to listen to. That is the whole of
its relationship to the rest of the system, which is why it can be restarted after
changing the pairing without touching a camera that is already open.

    # session 1
    ros2 launch uwb_driver uwb_driver.launch.py
    # session 2
    ros2 launch uvdar_core three_bluefox.launch.py
    # session 3
    ros2 launch mrs_ultraloc fusion.launch.py

One node, whatever the size of the rig
--------------------------------------

There is one fusion node per vehicle and one bearing topic per vehicle, for a rig of one
camera or three. uvdar_core's bearing node has a single publisher, on its config's
`bearing.output_topic`, and transforms every camera it is configured with into
`bearing.output_frame` before publishing - so cameras are added to that one topic, not
as extra topics. Which cameras exist is therefore a question about which uvdar_core
config session 2 started:

============================================  =========================  ==========================
uvdar_core launch                             config                     cameras
============================================  =========================  ==========================
``two_bluefox.launch.py``                     ``default_bearing.yaml``   left, right
``three_bluefox.launch.py``                   ``three_bluefox_bearing.yaml``  left, right, back
``bearing.launch.py`` with ``config_file:=``  whatever you pass          whatever it lists
============================================  =========================  ==========================

None of that reaches this file. It takes `bearing_topic`, which defaults to
``<uav_name>/uvdar/bearing/observations`` - the `output_topic` of both of those configs -
and a node whose bearing topic nothing publishes on reports that once per paired id
rather than refusing to start. So the line worth reading at startup is the fusion node's
own, which names the topic it settled on:

    ros2 launch mrs_ultraloc fusion.launch.py uav_name:=uav13
    ros2 topic info /uav13/uvdar/bearing/observations   # 1 publisher, 1 subscriber

Arguments
---------

`uav_name:=uav13` namespaces this node and prefixes both input topics; it defaults to
`$UAV_NAME`, the same variable the camera and bearing stages read, so one export covers
the vehicle. `bearing_topic:=/uav13/uvdar/bearing/other` and `uwb_topic:=...` point the
node somewhere else than the default - a bag, a second bearing endpoint, or a config
whose `output_topic` was renamed. `uwb_uvdar_id_pairs:=0x13:3,0x15:0` overrides the
pairing in config/uwb_uvdar_fusion.yaml; comma-separated because a ROS 2 command line
cannot express a list argument, and an empty `[]` reaching rclcpp from a YAML file
aborts the process. `fusion_config:=/path/other.yaml` for a different parameter file.
`use_sim_time:=true` for a bag.

    ros2 launch mrs_ultraloc fusion.launch.py --show-args
    ros2 topic echo /uav13/uwb_uvdar_fusion/targets

Targets and their frame
-----------------------

Positions are published in the frame the bearing stream carries, which for the shipped
configs is `bearing.output_frame` - `<uav>/fcu`, the vehicle's body frame. This node
applies no rotation: the composing of cameras happened in the bearing node, using the
static mounts that launch file publishes. A frame in the output that is not the one you
expected means the bearing endpoint's `output_frame`, not anything here.

One topic is not one merged reading, though. uvdar_core publishes one message per camera
callback, so a blinker two cameras can see arrives as two separate messages and the newest
replaces the previous - the fused direction alternates between the cameras at the tracker
rate rather than settling between them. See README.md's "Two cameras on one blinker" for
why this node does not average across them, and the static-hold check before debugging a
position that looks wrong: a bad mount and this look the same in the topics.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Where the bearing endpoint publishes, relative to its namespace. The `output_topic` of
# both default_bearing.yaml and three_bluefox_bearing.yaml, and the name a single
# bearing node publishes on for a rig of any size - it has one publisher.
BEARING_OUTPUT_TOPIC = 'uvdar/bearing/observations'

# Where the UWB driver publishes, relative to its namespace. Relative in that package's
# own config, so it follows the namespace and only needs the prefix.
UWB_OUTPUT_TOPIC = 'uwb/distance'


def generate_launch_description():

    uav_name = LaunchConfiguration('uav_name')

    return LaunchDescription([

        DeclareLaunchArgument(
            'uav_name',
            default_value=EnvironmentVariable('UAV_NAME', default_value='uav'),
            description='Vehicle namespace. Prefixes both input topics as well as this '
                        'node, so it is the name the cameras, the bearing endpoint and '
                        'the UWB driver are running under - the same $UAV_NAME they read.',
        ),

        DeclareLaunchArgument(
            'bearing_topic',
            # Relative, so it resolves against uav_name below. An absolute name also
            # works and is what to pass for a bag or a second endpoint.
            default_value=BEARING_OUTPUT_TOPIC,
            description='The bearing endpoint\'s output topic. One topic for a rig of any '
                        'size: uvdar_core\'s bearing node publishes all of its camera '
                        'inputs on bearing.output_topic, transformed into one frame. Which '
                        'cameras those are is decided by the uvdar_core config that '
                        'endpoint was started with, not by this argument.',
        ),

        DeclareLaunchArgument(
            'uwb_topic',
            default_value=UWB_OUTPUT_TOPIC,
            description='The UWB driver\'s range topic, relative to uav_name. One module '
                        'per vehicle however many cameras it wears, so this does not '
                        'change with the rig.',
        ),

        DeclareLaunchArgument(
            'fusion_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('mrs_ultraloc'), 'config', 'uwb_uvdar_fusion.yaml',
            ]),
            description='Parameter file. Its bearing_topic, uwb_topic and uwb_uvdar_id_pairs '
                        'are overridden below because they depend on the arguments above and '
                        'on $UAV_NAME, which one file cannot state; everything else in it - '
                        'timeouts, the id pairing when not overridden, the sigmas, the '
                        'diagnostics rate - applies as written.',
        ),

        DeclareLaunchArgument(
            'uwb_uvdar_id_pairs',
            default_value='',
            description='Comma-separated "uwb_address:uvdar_id" pairs overriding the config '
                        'file, e.g. "0x13:3,0x15:0". Empty leaves the file in charge.',
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=EnvironmentVariable('USE_SIM_TIME', default_value='false'),
            description='Follow /clock, so this works against a bag.',
        ),

        # The bearing node builds its `output_frame` from $UAV_NAME in the config it
        # reads (helpers/yaml.hpp expands $VARIABLE in every value and refuses to load
        # the file if the variable is unset), and the camera driver builds its frame_id
        # from the same variable. This node does not read UAV_NAME itself - the topics
        # above are performed from `uav_name` - but setting it here keeps the shell this
        # launch file runs in consistent with the namespace it chose, so a bearing or
        # camera node started afterwards from the same shell agrees with the one already
        # running. Without it, `uav_name:=uav13` on this command line and UAV_NAME=uav7
        # in the shell would put this node's topics under uav13 while anything started
        # later from that shell built its frames under uav7 - two TF trees, and no error
        # from anything.
        SetEnvironmentVariable(name='UAV_NAME', value=uav_name),

        OpaqueFunction(function=launch_fusion),
    ])


def launch_fusion(context, *args, **kwargs):
    """Perform the arguments once, then start the node.

    A function rather than substitutions on the Node because the two input topics have to
    be turned absolute here, from one performed `uav_name`: written as relative strings
    they would resolve against the node's own namespace, which happens to be the same
    answer today and a different one the moment the node is moved somewhere else - and a
    fusion node listening in the wrong namespace is a target that never appears.
    """

    def value(name, cast=str):
        return cast(perform_substitutions(context, [LaunchConfiguration(name)]).strip())

    uav_name = value('uav_name')
    if not uav_name:
        raise ValueError('uav_name is empty; pass uav_name:=<name> or export UAV_NAME')

    # Absolute unless the caller said so. A leading '/' in either argument is taken as
    # "I know the topic I want" and passed through; anything else is prefixed with the
    # vehicle, which is the usual case and the one that is easy to get wrong by hand.
    def absolute(topic):
        if not topic:
            raise ValueError('an input topic is empty; both default to real topic names '
                             'relative to uav_name')
        return topic if topic.startswith('/') else f'/{uav_name}/{topic}'

    bearing_topic = absolute(value('bearing_topic'))
    uwb_topic = absolute(value('uwb_topic'))

    pairs = [entry.strip() for entry in value('uwb_uvdar_id_pairs').split(',') if entry.strip()]

    parameters = [
        LaunchConfiguration('fusion_config'),
        {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'bearing_topic': bearing_topic,
            'uwb_topic': uwb_topic,
        },
    ]

    if pairs:
        # Only when non-empty, and as a plain list of str: the node rejects an empty
        # pairing outright, and an empty `[]` handed to rclcpp as a parameter with no
        # element type aborts the process.
        parameters.append({'uwb_uvdar_id_pairs': pairs})

    return [
        LogInfo(msg='[mrs_ultraloc] fusing %s + %s -> /%s/uwb_uvdar_fusion/targets'
                      % (bearing_topic, uwb_topic, uav_name)
                      + (f' on pairs {pairs}' if pairs else '')),
        # Worth printing because it names the thing this file cannot check: a bearing
        # topic nothing publishes on starts a node that fuses nothing and says so only in
        # a once-per-id warning. `ros2 topic info` on the line above is the check.
        LogInfo(msg='[mrs_ultraloc] expects the cameras, the bearing endpoint and the UWB '
                    'driver to already be running - this file starts none of them. The '
                    'number of cameras is whatever the running bearing endpoint\'s config '
                    'lists; targets are published in the frame that endpoint stamps.'),

        # In the vehicle namespace, so the output lands on <uav>/uwb_uvdar_fusion/targets
        # and moves with `uav_name`. One node per vehicle, not one per camera: there is
        # one bearing stream per vehicle.
        Node(
            package='mrs_ultraloc',
            executable='uwb_uvdar_fusion_node',
            name='uwb_uvdar_fusion',
            namespace=[uav_name],
            output='screen',
            parameters=parameters,
        ),
    ]
