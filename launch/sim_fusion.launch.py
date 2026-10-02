#!/usr/bin/env python3

"""Bring up the fusion node with both of its inputs simulated.

  mrs_ultraloc   sim_uvdar_target.py        <ns>/uvdar/bearing/observations
  mrs_ultraloc   sim_uwb_module.py          <ns>/uwb/distance
  mrs_ultraloc   uwb_uvdar_fusion_node      <ns>/uwb_uvdar_fusion/targets

One target, flying a rounded square roughly 1 to 5 m in front of the camera. No
camera, no UWB module, and neither `uvdar_core` nor `uwb_driver` is started - the two
simulators publish on the topics those would publish on, spelled exactly as
uvdar_core's bearing config and the UWB driver's config spell them, so what the fusion
node sees here is what it sees on the vehicle and only the producer differs. See
`sim_uvdar_target.py`'s docstring for why the detector is not in the loop.

Try it::

  ros2 launch mrs_ultraloc sim_fusion.launch.py
  ros2 topic hz /uav/uvdar/bearing/observations   # ~60 Hz
  ros2 topic hz /uav/uwb/distance                 # ~42 Hz
  ros2 topic echo  /uav/uwb_uvdar_fusion/targets  # one target, id 0

Different target, different flight::

  ros2 launch mrs_ultraloc sim_fusion.launch.py uvdar_id:=3 peer_address:=0x13
  ros2 launch mrs_ultraloc sim_fusion.launch.py centre_x:=8 half_side:=3 period_sec:=40
  ros2 launch mrs_ultraloc sim_fusion.launch.py range_noise_sigma_m:=0.3 trajectory_noise_sigma_m:=0.1

In place of a running bearing endpoint
-------------------------------------

The bearing endpoint publishes on one topic for a rig of any size, stamped in its own
`output_frame`, so that is what this file imitates - one topic, one frame, no camera in
either name. Note that one topic is not one merged reading: the endpoint publishes one
message per camera, so a real multi-camera rig interleaves messages rather than
combining them into one. This file publishes one target per message, which is what one
camera does and is the simpler case a single stream is still valid for. To stand in for
a rig that is actually running, stop that rig's bearing stage and start this file's
simulator alone::

  ros2 run mrs_ultraloc sim_uvdar_target.py --ros-args \\
      -p topic:=/uav13/uvdar/bearing/observations \\
      -p camera_frame:=uav13/fcu -p signal_id:=0

which leaves the real cameras and the real fusion node in place and replaces only the
bearing source. `camera_frame` matters: the fusion publishes in whatever frame the
bearings carry, so giving it the frame the rig's bearings use is what keeps RViz
resolving the target against the rig's own TF tree. `output_frame:=` here does the same
thing through this file.

Everything is built in one place, on purpose
--------------------------------------------

All three nodes are constructed by the `OpaqueFunction` below, from one performed set
of arguments. Not for tidiness: the fusion's `uwb_uvdar_id_pairs` has to name the same
two numbers the two simulators were started with, and `0x2345` on the command line is
text that each consumer would otherwise have to agree on a way to read. Performed and
parsed once here, an address is one integer with one spelling everywhere - and a typo
fails at startup instead of producing a bearing with no range and a range with no
bearing, which presents as a target that simply never appears.

The same holds for the trajectory: the geometry arguments are handed to *both*
simulators from one dictionary. If they ever differed, each node would produce a
self-consistent output about a different point, which again reads as a fusion bug and
is not one. Change the path through the arguments below, never by editing a node.

config/uwb_uvdar_fusion.yaml's `uwb_uvdar_id_pairs` is overridden rather than used: its
shipped pairs (0xAA:28 and friends) mention neither of the addresses started here.

Rates
-----

Bearings at 60 Hz, ranges at 42 Hz. Both far inside the fusion's timeouts
(`bearing_timeout_sec` 1.0, `range_timeout_sec` 2.0), so a run that loses no frames
should never go stale. The node republishes the newest pair at `publish_rate_hz` from
the config file - 20 Hz - so the *output* is slower than either input; raising it is a
`publish_rate_hz` override on the fusion, not a change here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Where uvdar_core's bearing endpoint publishes, relative to its namespace: the
# `bearing.output_topic` of both default_bearing.yaml and three_bluefox_bearing.yaml. One
# topic for a rig of any size, because that node has one publisher for all its inputs -
# a simulator whose topic has drifted from the producer's tests a wiring that no longer
# exists, which is why this is spelled from the producer's config rather than invented.
BEARING_OUTPUT_TOPIC = "uvdar/bearing/observations"

# Where the UWB driver publishes, relative to its namespace.
UWB_OUTPUT_TOPIC = "uwb/distance"

# The frame those bearings carry: the endpoint's `bearing.output_frame`, which the
# configs state as $UAV_NAME/fcu. The fusion node publishes in the frame it is given, so
# a simulation that stamped its bearings with anything else would be testing a frame the
# rig never produces.
FRAME_SUFFIX = "fcu"

# The namespace config/uwb_uvdar_fusion.yaml's own topic names were written for.
PINNED_UAV_NAME = "uav"

# A trajectory parameter SquareOrbit decides for itself is sent to the nodes as this
# sentinel, which they translate back into "you choose". Mirrors CORNER_RADIUS_AUTO in
# ultraloc_tools/parameters.py; a literal rather than an import so that evaluating this
# file does not depend on the install being complete.
CORNER_RADIUS_AUTO = "-1"

# The geometry arguments, in the order the nodes expect them. Both simulators are given
# this same set, which is the guarantee that the target the bearing points at and the
# target the range measures are one and the same.
TRAJECTORY_ARGUMENTS = {
    "trajectory_centre_x": ("centre_x", float),
    "trajectory_lateral_y": ("lateral_y", float),
    "trajectory_half_side": ("half_side", float),
    "trajectory_corner_radius": ("corner_radius", float),
    "trajectory_period_sec": ("period_sec", float),
    "trajectory_noise_sigma_m": ("trajectory_noise_sigma_m", float),
    "trajectory_noise_seed": ("noise_seed", int),
}


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            "uav_name",
            default_value=PINNED_UAV_NAME,
            description="Namespace for all three nodes, so this matches a real bringup.",
        ),

        # ---- which frame the bearings are in ---------------------------------
        #
        # The endpoint stamps every camera it publishes with its own `output_frame`, and
        # the fusion node publishes positions in the frame the bearings carry. So this is
        # the one thing to get right when standing in for a running rig: a frame the TF
        # tree does not know is not an error, it is a target RViz draws at the origin.
        # The default is what both uvdar_core configs state for this vehicle.
        DeclareLaunchArgument(
            "output_frame",
            default_value="",
            description="header.frame_id of the simulated bearings. Empty means "
                        "<uav_name>/fcu, which is what uvdar_core's bearing configs set "
                        "output_frame to. Pass an absolute frame to check that a fused "
                        "target resolves in it.",
        ),

        # ---- which target, on which pair ------------------------------------

        DeclareLaunchArgument(
            "this_address",
            default_value="0x1234",
            description="This vehicle's UWB module address, hex or decimal. The fusion "
                        "only requires that it be one of the two endpoints of the "
                        "exchange, which the sim guarantees by filling it in as the "
                        "initiator.",
        ),
        DeclareLaunchArgument(
            "peer_address",
            default_value="0x2345",
            description="The simulated target's module address, hex or decimal. Becomes "
                        "the UWB half of the id pairing, so it is the address a real "
                        "module would have to have for this rig to stand in for it.",
        ),
        DeclareLaunchArgument(
            "uvdar_id",
            default_value="0",
            description="The signal id the tracker would decode from the target's "
                        "blinker - its index into tracking.sequences in the uvdar "
                        "config. Becomes the UVDAR half of the id pairing.",
        ),

        # ---- how fast -------------------------------------------------------

        DeclareLaunchArgument(
            "bearing_rate_hz",
            default_value="60",
            description="Rate of the simulated bearing endpoint.",
        ),
        DeclareLaunchArgument(
            "range_rate_hz",
            default_value="42",
            description="Rate of the simulated UWB module.",
        ),

        # ---- how it flies ---------------------------------------------------

        DeclareLaunchArgument(
            "centre_x",
            default_value="3.0",
            description="Depth of the path's centre along the optical axis, in metres.",
        ),
        DeclareLaunchArgument(
            "lateral_y",
            default_value="0.5",
            description="Constant lateral offset, in metres.",
        ),
        DeclareLaunchArgument(
            "half_side",
            default_value="2.0",
            description="Half the side of the square, so the path spans +/- this much in "
                        "depth and in elevation. Closest approach is "
                        "hypot(centre_x - half_side, lateral_y).",
        ),
        DeclareLaunchArgument(
            "corner_radius",
            default_value=CORNER_RADIUS_AUTO,
            description="Corner rounding, a quarter of the side by default. 0 is a sharp "
                        "square, half_side is a circle.",
        ),
        DeclareLaunchArgument(
            "period_sec",
            default_value="20.0",
            description="Seconds for one lap; about 0.6 m/s with the other defaults.",
        ),
        DeclareLaunchArgument(
            "trajectory_noise_sigma_m",
            default_value="0.0",
            description="Position wiggle, 1 sigma per axis, applied identically in both "
                        "nodes so the target stays self-consistent. 0 keeps the path exact.",
        ),
        DeclareLaunchArgument(
            "noise_seed",
            default_value="0",
            description="Seed for the position wiggle. Same seed, same wiggle, so a run "
                        "can be repeated.",
        ),

        # ---- how noisy the sensors are --------------------------------------

        DeclareLaunchArgument(
            "bearing_sigma_rad",
            default_value="0.02",
            description="Angular 1-sigma written into the bearing covariance. Matches the "
                        "fusion's own bearing_sigma_rad default, so the output covariance "
                        "is what the config alone would produce unless this is changed.",
        ),
        DeclareLaunchArgument(
            "range_noise_sigma_m",
            default_value="0.0",
            description="Multipath-style 1-sigma on the reported range. The fusion adds "
                        "range_sigma_m to the covariance whatever the input says, so this "
                        "moves the position error without moving the reported uncertainty.",
        ),

        # ---- the node itself ------------------------------------------------

        DeclareLaunchArgument(
            "fusion_config",
            default_value=[FindPackageShare("mrs_ultraloc"), "/config/uwb_uvdar_fusion.yaml"],
            description="Parameter file for the fusion node. Its bearing_topic, uwb_topic "
                        "and uwb_uvdar_id_pairs are overridden below, so the file's values "
                        "for those three do not apply here.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false"),
            description="Follow /clock, so the sims can be replayed against a bag.",
        ),

        OpaqueFunction(function=launch_nodes),

        LogInfo(msg=[
            "[mrs_ultraloc] simulating one target, id '", LaunchConfiguration("uvdar_id"),
            "' at module '", LaunchConfiguration("peer_address"), "' on /",
            LaunchConfiguration("uav_name"), ". Watch /", LaunchConfiguration("uav_name"),
            "/uwb_uvdar_fusion/targets"],
        ),
    ])


def launch_nodes(context, *args, **kwargs):
    """Perform and parse every argument once, then start all three nodes.

    See the module docstring for why one function builds all of them.
    """

    def value(name, cast=str):
        return cast(perform_substitutions(context, [LaunchConfiguration(name)]).strip())

    # UWB module addresses are conventionally written in hex, and a launch argument is
    # only a string, so base-0 accepts 0x2345 and 9029 alike.
    this_address = value("this_address", lambda text: int(text, 0))
    peer_address = value("peer_address", lambda text: int(text, 0))
    uvdar_id = value("uvdar_id", lambda text: int(text, 0))
    namespace = perform_substitutions(context, [LaunchConfiguration("uav_name")])

    for name, address in (("this_address", this_address), ("peer_address", peer_address)):
        # Wider than 16 bits and the node would key the range to a peer the pairing can
        # never mention: accepted, stored, and never fused.
        if not 0 <= address <= 0xFFFF:
            raise ValueError(f'{name} must be a 16-bit module address, got {address}')
    if this_address == peer_address:
        raise ValueError('this_address and peer_address must differ, or the fusion has no '
                         f'peer to key the range to (both are 0x{this_address:04X})')

    trajectory = {
        parameter: value(argument, cast)
        for parameter, (argument, cast) in TRAJECTORY_ARGUMENTS.items()
    }

    bearing_topic = f'/{namespace}/{BEARING_OUTPUT_TOPIC}'
    uwb_topic = f'/{namespace}/{UWB_OUTPUT_TOPIC}'

    # The frame a real endpoint's bearings carry, so a target fused against these
    # bearings is published in the frame the rig would publish it in and resolves against
    # the rig's own TF tree. Left to the simulator's default this file would silently
    # test a frame name no rig produces.
    camera_frame = value("output_frame") or f'{namespace}/{FRAME_SUFFIX}'

    return [
        Node(
            package="mrs_ultraloc",
            executable="sim_uvdar_target.py",
            name="sim_uvdar_target",
            # In the vehicle namespace, as the fusion node is. Two instances of this file
            # on one machine would therefore collide on node names and double-publish the
            # same targets topic - use the bare `ros2 run` form in the docstring for a
            # second one, which is also the form that leaves the real nodes running.
            namespace=[namespace],
            output="screen",
            parameters=[{
                # Absolute: the sim publishes where the fusion listens rather than
                # namespacing itself, because matching the real endpoint's topic is the
                # whole point.
                "topic": bearing_topic,
                "camera_frame": camera_frame,
                "publish_rate_hz": value("bearing_rate_hz", float),
                "signal_id": uvdar_id,
                "bearing_sigma_rad": value("bearing_sigma_rad", float),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                **trajectory,
            }],
        ),

        Node(
            package="mrs_ultraloc",
            executable="sim_uwb_module.py",
            name="sim_uwb_module",
            namespace=[namespace],
            output="screen",
            parameters=[{
                "topic": uwb_topic,
                "publish_rate_hz": value("range_rate_hz", float),
                "this_address": this_address,
                "peer_address": peer_address,
                "range_noise_sigma_m": value("range_noise_sigma_m", float),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                **trajectory,
            }],
        ),

        Node(
            package="mrs_ultraloc",
            executable="uwb_uvdar_fusion_node",
            name="uwb_uvdar_fusion",
            namespace=[namespace],
            output="screen",
            parameters=[
                LaunchConfiguration("fusion_config"),
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    # Recomputed rather than read from the file, exactly as
                    # fusion.launch.py does and for the same reason: the file pins both to
                    # one namespace, which is right for the default and wrong for any
                    # other uav_name.
                    "bearing_topic": bearing_topic,
                    "uwb_topic": uwb_topic,
                    # The one pair this rig has, from the same two integers the
                    # simulators were started with. Hex on the UWB side because that is
                    # how the config file and the node's own parsing examples write it.
                    "uwb_uvdar_id_pairs": [f"0x{peer_address:04X}:{uvdar_id}"],
                },
            ],
        ),
    ]
