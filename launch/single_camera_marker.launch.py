#!/usr/bin/env python3

"""Show the fused targets in RViz: camera at the origin, target, its trail, its covariance.

  ros2 launch mrs_ultraloc single_camera_marker.launch.py
  ros2 launch mrs_ultraloc single_camera_marker.launch.py uav_name:=uav9
  ros2 launch mrs_ultraloc single_camera_marker.launch.py rviz:=false

It only visualises, so the data has to be coming from somewhere. Alongside a real
pipeline, or - the usual bench case - next to the simulator, in a second shell:

  ros2 launch mrs_ultraloc sim_fusion.launch.py
  ros2 launch mrs_ultraloc single_camera_marker.launch.py

Both take the same `uav_name`, which is what puts the visualiser's *input* where the
fusion node publishes. The *output* is the fixed absolute `/markers`, because RViz reads
the topic to listen to from config/single_camera_marker.rviz and that file cannot name a
namespace - so the config is right under every `uav_name` and there is nothing to keep in
step. See sim_fusion.launch.py for the middleware caveat: the default rmw_zenoh_cpp needs
a router, and without one everything starts, every topic exists, and nothing arrives.

What you should see
-------------------

A grid in the camera's own optical frame (+x forward along the optical axis, +y left, +z
up), three short axis arrows at the origin, and one target orbiting a rounded square 1.1
to 5.3 m out, leaving a trail that fades towards its oldest end. The target is the
fusion's fused position, not the simulated ground truth - the two agreeing to the rounding
is the point of running them together.

`n_sigma:=2` doubles the covariance ellipsoid. The ellipsoid is the fusion's *reported*
uncertainty and the trail is what it *did*, so comparing the two is the whole reason to
draw a covariance: a trail that wanders outside the 1-sigma shell means the reported
uncertainty is too small.

rviz:=false starts only the node, for attaching to an RViz you already have open - in
which case set the fixed frame to `camera_0_optical_frame` and add one Marker display on
`/markers`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            "uav_name",
            default_value=EnvironmentVariable("UAV_NAME", default_value="uav"),
            description="Namespace of the fusion node being visualised. The input topic "
                        "is read relative to it, so this has to match whatever is "
                        "publishing - the simulator's uav_name, or the real vehicle's.",
        ),
        DeclareLaunchArgument(
            "output_topic",
            default_value="/markers",
            description="Topic the MarkerArray is published on. Absolute by default, and "
                        "config/single_camera_marker.rviz listens on exactly that, so "
                        "changing it means pointing the RViz display at the new name.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz with this package's config. false runs only the "
                        "marker node.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=[FindPackageShare("mrs_ultraloc"),
                           "/config/single_camera_marker.rviz"],
            description="RViz config: the fixed frame and the one Marker display. "
                        "Anything else about the view is yours to change and save.",
        ),
        # The two parameters worth reaching for from the command line. Most of the rest can
        # be changed live with `ros2 param set`, which is the better way to tune what is on
        # screen - except the rate itself, whose launch argument is the only way to change
        # it, since a rclpy timer's period is fixed when it is created.
        DeclareLaunchArgument(
            "n_sigma",
            default_value="1.0",
            description="How many standard deviations across the covariance ellipsoid's "
                        "axes. 1 is the fusion's own 1-sigma; 2 is easier to see at range.",
        ),
        DeclareLaunchArgument(
            "publish_rate_hz",
            default_value="10",
            description="How often the marker array is redrawn. Independent of the input "
                        "rate on purpose; the trail records every input message.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false"),
            description="Follow /clock, to match the node being visualised.",
        ),
        # Relative by default, so it resolves against uav_name above and lands on the one
        # topic a single-camera rig publishes. Named as an argument rather than fixed here
        # because a multi-camera rig publishes one target stream per camera
        # (one_cam/two_cams/three_cams.launch.py), and which one to look at is a question
        # this file cannot answer. An absolute name works too.
        #
        # One instance per camera means a second output_topic as well, since the shipped
        # rviz config listens on /markers only - see output_topic's note on the shared
        # topic. A second Marker display pointed at the second output topic is the whole
        # extra step.
        DeclareLaunchArgument(
            "input_topic",
            default_value="uwb_uvdar_fusion/targets",
            description="Which fusion output to visualise. For a camera slot pass e.g. "
                        "input_topic:=camera_left/uwb_uvdar_fusion/targets.",
        ),

        # Only needed for a second instance: two nodes of the same name in the same
        # namespace work but are indistinguishable in `ros2 node list` and in a TF or
        # topic tool's output, which is a poor way to find out which camera you are
        # looking at.
        DeclareLaunchArgument(
            "node_name",
            default_value="single_camera_marker",
            description="Name of the marker node. Change it when visualising two cameras "
                        "at once, so the two instances can be told apart.",
        ),

        Node(
            package="mrs_ultraloc",
            executable="single_camera_marker.py",
            name=LaunchConfiguration("node_name"),
            # The node's namespace scopes only what it reads; see output_topic.
            namespace=[LaunchConfiguration("uav_name")],
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                # Coerced, because a launch argument is text and the node's read helpers
                # reject a string where a number is wanted rather than guessing.
                "n_sigma": ParameterValue(LaunchConfiguration("n_sigma"), value_type=float),
                "publish_rate_hz": ParameterValue(LaunchConfiguration("publish_rate_hz"),
                                                  value_type=float),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ])
