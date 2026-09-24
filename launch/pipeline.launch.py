#!/usr/bin/env python3

"""Bring up the UWB driver, the UVDAR bearing endpoint and the fusion node.

  uwb_driver        uwb_driver.launch.py     <ns>/uwb/distance
  uvdar_core        bearing.launch.py        <ns>/bearing/camera_0/observations
  mrs_ultraloc      this file                <ns>/uwb_uvdar_fusion/targets

The front ends are included from their own packages rather than re-declared here,
so this file cannot drift away from how each stage is meant to be started. It is
a bringup file, not a test fixture: the integration tests drive the fusion node
directly, against synthetic bearing and range messages.

Try it::

  ros2 launch mrs_ultraloc pipeline.launch.py uwb_serial:=<12-hex-digit-serial>
  ros2 launch mrs_ultraloc pipeline.launch.py \\
      uwb_uvdar_id_pairs:="0x13:3,0x15:0,0x16:2"

what came up::

  ros2 topic list -t | grep -E "uwb|bearing"
  ros2 node info /uav/uwb_uvdar_fusion

and what the fusion node makes of its inputs::

  ros2 topic echo /uav/uwb_uvdar_fusion/targets

The fusion node's two input topics are overridden here, not read from the config
--------------------------------------------------------------------------------

config/uwb_uvdar_fusion.yaml pins `bearing_topic` and `uwb_topic` to /uav as
absolute topics, which is only correct for one namespace, and its bearing topic
is spelled with an `uvdar/` segment that the bearing endpoint does not actually
use - it publishes `bearing.inputs[].output_topic` relative to its own namespace,
which lands on <ns>/bearing/camera_0/observations. Both are recomputed here from
`uav_name` and the loaded uvdar config, so the pipeline connects under any
namespace and the config's values for those two keys do not apply to it. Launch
the stages separately and the config file is back in charge.

What it looks like with no hardware attached
--------------------------------------------

None of these is a fault, and each is what a stage is supposed to do with a
missing input - collected here because together they look like a broken bringup.

* **Silent, not stuck.** With no camera and no module in range, detector, tracker
  and bearing start and stay quiet - the camera is not launched at all, see
  "Feeding images" below - and the fusion node repeats that it has never seen a
  bearing or a range for each paired id. All five nodes staying up is the pass
  condition here; `ros2 node list` showing five is the check.

* **Wrong calibration.** `bearing.inputs[].calib_file` in the uvdar config points
  at `camera/ocam.yaml`, which ships with uvdar_core, so the bearing node does
  come up - with a calibration that is not the Bluefox's. Bearings will be
  numerically plausible and spatially wrong. Replace it with one from
  `calibrator.launch.py` before trusting a bearing.

* **Nothing to composite.** `tracking.sequences` in the uvdar config holds the
  blink patterns the Bluefox blinkers were measured with; a tracker seeing no LED
  patterns never decodes an id, and `bearing.publish_unidentified: true` then
  emits observations with negative ids that match no pair. A fused target needs a
  real blinker, not just a bright spot in the image.

Feeding images
--------------

The bearing stage reads whatever `detector.inputs[].input_topic` and
`tracking.inputs[].input_image_topic` name in the config it loads, and the
uvdar_core default is `/my_camera/pylon_ros2_camera_node/image_raw` - a Basler
node name. `single_bluefox.launch.py` in the same package publishes relative to
its own namespace, so the two only meet if a config re-points those topics at the
camera in use. Pass that config through::

  ros2 launch mrs_ultraloc pipeline.launch.py uvdar_config:=/path/to/own.yaml

Stage-by-stage startup stays available while integrating; each stage's own launch
file takes the same arguments this one passes it.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# FIXME: placeholder, replace with the serial of the module on the bench.
#
# Twelve hexadecimal digits, which is what `udevadm info -q property -n
# /dev/ttyACM0 | grep ID_SERIAL_SHORT` prints for a connected module. Two ways a
# wrong value misleads:
#
#   - The driver logs only the serial it was given ("Device <serial> is not
#     connected, retrying") and never enumerates what it can see, so a typo reads
#     exactly like an absent module. Leaving `uwb_serial` empty is the exception:
#     the driver then logs the serial numbers it does see, which is the quickest
#     way to find the right one.
#   - A serial of the form digits-E-digits aborts the driver on startup, because
#     launch writes the parameter file with PyYAML, which reads such a token as a
#     string and leaves it unquoted, while rclcpp parses it with yaml-cpp, which
#     reads it as a float - "parameter 'usb_serial' has invalid type". 206133814E31
#     and 207533814E31 are both of that shape; the placeholder below is not. Quote
#     the value in uwb_driver's config file instead of passing it here if a real
#     serial turns out to have that shape.
DEFAULT_UWB_SERIAL = "00AA11BB22CC"

# BearingObservationArrayStamped output of the bearing endpoint, as
# `bearing.inputs[].output_topic` names it in uvdar_core's default_bearing.yaml,
# relative to that stage's namespace. Repeated here because the fusion node needs
# the same name as an absolute topic - see the note in the module docstring. If
# `uvdar_config` selects a file where that key differs, this has to follow it.
DEFAULT_BEARING_OUTPUT_TOPIC = "bearing/camera_0/observations"

# Where uwb_driver publishes, relative to its namespace. It is a relative topic in
# that package's config, so it follows the namespace and only needs the prefix.
UWB_OUTPUT_TOPIC = "uwb/distance"

# The namespace config/uwb_uvdar_fusion.yaml's own topic names were written for.
# Only of historical interest for this file, which recomputes both inputs, but it
# is what the config's comments refer to.
PINNED_UAV_NAME = "uav"


def generate_launch_description():

    uav_name = LaunchConfiguration("uav_name")

    return LaunchDescription([

        DeclareLaunchArgument(
            "uav_name",
            default_value=PINNED_UAV_NAME,
            description="Namespace shared by all three stages. Exported as UAV_NAME, "
                        "which is how uwb_driver picks its namespace, and used as the "
                        "namespace of the bearing stage and of this node.",
        ),

        # Named fusion_config rather than config: uwb_driver.launch.py declares its
        # own `config`, and an argument of that name set out here reaches into the
        # included launch description and replaces its default. The driver then
        # loads a fusion parameter file and aborts on the first type mismatch - a
        # crash whose message names neither file nor the collision.
        DeclareLaunchArgument(
            "fusion_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("mrs_ultraloc"), "config", "uwb_uvdar_fusion.yaml",
            ]),
            description="Parameter file for the fusion node: timeouts, sigmas, the id "
                        "pairing and the output topic. Its bearing_topic and uwb_topic "
                        "are overridden below - see the module docstring.",
        ),

        DeclareLaunchArgument(
            "uvdar_config",
            # Given explicitly rather than left to the included file's default,
            # because an argument here that could be empty would need to be passed
            # conditionally to avoid overriding that default with a blank path.
            default_value=PathJoinSubstitution([
                FindPackageShare("uvdar_core"), "config", "default_bearing.yaml",
            ]),
            description="Detector/tracker/bearing configuration for the bearing stage.",
        ),

        DeclareLaunchArgument(
            "bearing_topic",
            default_value=PathJoinSubstitution(
                ["/", uav_name, DEFAULT_BEARING_OUTPUT_TOPIC]),
            description="Absolute topic the fusion node reads bearings from. Must be "
                        "the bearing endpoint's bearing.inputs[].output_topic under its "
                        "namespace, which this default is for the shipped config.",
        ),

        DeclareLaunchArgument(
            "uwb_serial",
            default_value=DEFAULT_UWB_SERIAL,
            description="USB serial of the UWB module on this computer. Passed to "
                        "uwb_driver as serial_number, where it overrides usb_serial "
                        "from that package's config file.",
        ),

        DeclareLaunchArgument(
            "standalone",
            default_value="true",
            description="Run uwb_driver as its own node instead of as a component. "
                        "See the namespace note where it is used.",
        ),

        DeclareLaunchArgument(
            "uwb_uvdar_id_pairs",
            default_value="",
            description='Comma-separated "uwb_address:uvdar_id" pairs overriding '
                        "uwb_uvdar_id_pairs from the config file, e.g. "
                        '"0x13:3,0x15:0,0x16:2". Empty leaves the config file in '
                        "charge. UWB addresses are hexadecimal module addresses, the "
                        "UVDAR side is the id that vehicle's blinker decodes to.",
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false"),
            description="Use the /clock topic, so this works against a bag",
        ),

        # uwb_driver.launch.py takes no namespace argument, it reads UAV_NAME from
        # the environment. Without this the driver namespaces itself from whatever
        # the shell exports - and the two included files disagree on the fallback,
        # `uav` for the driver and `uav1` for the bearing stage - so a shell with
        # UAV_NAME unset would put the ranges somewhere the fusion node is not
        # listening, even though `uav_name` was passed to both of them.
        SetEnvironmentVariable(name="UAV_NAME", value=uav_name),

        # Standalone by default, which is what keeps the driver inside the
        # namespace. As a component - the default of the included file - the
        # container is namespaced but the component is created from the root
        # namespace, so the driver ends up on /uwb_driver publishing /uwb/distance
        # while everything else looks under /<ns>. The container path is there for
        # the deployment that composes the stages into one container, with that
        # container's own namespacing to get right.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("uwb_driver"), "launch", "uwb_driver.launch.py",
                ])),
            launch_arguments={
                "serial_number": LaunchConfiguration("uwb_serial"),
                "standalone": LaunchConfiguration("standalone"),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("uvdar_core"), "launch", "bearing.launch.py",
                ])),
            launch_arguments={
                "namespace": uav_name,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "config_file": LaunchConfiguration("uvdar_config"),
            }.items(),
        ),

        # The fusion node is built at launch time because its id pairing is a
        # string array, which a launch argument - one string - cannot express.
        OpaqueFunction(function=launch_fusion_node),

        LogInfo(
            msg=["[mrs_ultraloc] fusing '", LaunchConfiguration("bearing_topic"), "' with '",
                PathJoinSubstitution(["/", uav_name, UWB_OUTPUT_TOPIC]), "' into '/", uav_name,
                "/uwb_uvdar_fusion/targets'. No camera started, so the bearing stage "
                "stays idle until images reach the input topics of its config."],
        ),
    ])


def launch_fusion_node(context, *args, **kwargs):
    """Start the fusion node, with the id pairing overridden only if one was given.

    An empty argument has to be left out rather than forwarded as an empty array:
    the node rejects an empty pairing outright, so forwarding one would turn the
    common case - not wanting to override anything - into a startup failure.
    """

    parameters = [
        LaunchConfiguration("fusion_config"),
        {
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            # The config file's values for these two are absolute and pinned to one
            # namespace, and its bearing topic carries an uvdar/ segment the
            # endpoint does not publish under. Recomputed from what is actually
            # being started here, so the pipeline connects without editing that
            # file; see the module docstring.
            "bearing_topic": LaunchConfiguration("bearing_topic"),
            "uwb_topic": ParameterValue(
                PathJoinSubstitution(
                    ["/", LaunchConfiguration("uav_name"), UWB_OUTPUT_TOPIC]),
                value_type=str),
        },
    ]

    raw = perform_substitutions(context, [LaunchConfiguration("uwb_uvdar_id_pairs")])
    pairs = [entry.strip() for entry in raw.split(",") if entry.strip()]

    # A plain Python list, not a ParameterValue: by this point the substitutions
    # have been performed and a list of str is the value the parameter wants, while
    # wrapping it makes launch try to resolve it as a scalar substitution and raise.
    if pairs:
        parameters.append({"uwb_uvdar_id_pairs": pairs})

    return [
        Node(
            package="mrs_ultraloc",
            executable="uwb_uvdar_fusion_node",
            name="uwb_uvdar_fusion",
            # Relative, so the node lands on /<uav_name>/uwb_uvdar_fusion like the
            # stages in front of it. The output topic is relative too and moves with
            # it; the two inputs above are absolute.
            namespace=[LaunchConfiguration("uav_name")],
            output="screen",
            parameters=parameters,
        ),
    ]
