from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    # The fusion node only subscribes, so nothing here starts the UVDAR pipeline or
    # the UWB driver - include those alongside this, or run this against a bag.
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=PathJoinSubstitution([
                FindPackageShare("mrs_ultraloc"),
                "config",
                "uwb_uvdar_fusion.yaml",
            ]),
            description="Node parameter file with the topic names and the UWB:UVDAR id pairing",
        ),
        # The node name and namespace only affect where the output topic lands and
        # what `ros2 node list` shows; the config file keys its parameters to /**
        # rather than to the node name, so it still applies under any of these.
        DeclareLaunchArgument(
            "node_name",
            default_value="uwb_uvdar_fusion",
        ),
        # The inputs are absolute topics from the config file, so this namespace does
        # not have to match either producer's - it only scopes the output.
        DeclareLaunchArgument(
            "namespace",
            default_value=EnvironmentVariable("UAV_NAME", default_value="uav"),
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false"),
            description="Use the /clock topic, so this works against a bag",
        ),
        Node(
            package="mrs_ultraloc",
            executable="uwb_uvdar_fusion_node",
            name=LaunchConfiguration("node_name"),
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            # The config file does not set use_sim_time, but listing it last keeps
            # the intent obvious if someone adds it there later.
            parameters=[
                LaunchConfiguration("config"),
                {"use_sim_time": use_sim_time},
            ],
        ),
    ])
