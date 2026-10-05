"""Launch the self-contained EmberTrace forest and one Husky UGV."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from ament_index_python.packages import get_package_share_directory
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("embertrace_sim")
    config_dir = PathJoinSubstitution([package_share, "config"])
    model_dir = PathJoinSubstitution([package_share, "models"])
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_rviz = LaunchConfiguration("use_rviz")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([package_share, "urdf", "husky.urdf.xacro"]),
            " prefix:=husky1_ gz_model_name:=husky1",
        ]),
        value_type=str,
    )

    def launch_world(context):
        scenario = LaunchConfiguration("scenario").perform(context)
        if scenario not in ("fire", "no_fire", "dust", "hot_surface", "visual_decoy", "dropout"):
            raise ValueError("Unknown scenario: " + scenario)
        world = Path(get_package_share_directory("embertrace_sim")) / "worlds" / "embertrace_forest.sdf"
        tree = ET.parse(world)
        if scenario in ("no_fire", "dust", "hot_surface"):
            parent = tree.getroot().find("world")
            for item in list(parent.findall("include")):
                if item.findtext("name") == "fire_source_visual":
                    parent.remove(item)
        with tempfile.NamedTemporaryFile(prefix="embertrace_", suffix=".sdf", delete=False) as handle:
            tree.write(handle, encoding="utf-8", xml_declaration=True)
            runtime_world = handle.name
        def cleanup(_context):
            Path(runtime_world).unlink(missing_ok=True)
            return []
        return [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("ros_ign_gazebo"), "launch", "ign_gazebo.launch.py"
            ])), launch_arguments={"ign_args": [runtime_world, " -r"]}.items()),
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        ]

    gazebo = OpaqueFunction(function=launch_world)

    drone_bridge = Node(
        package="ros_ign_bridge", executable="parameter_bridge", namespace="drone1",
        name="gazebo_bridge", output="screen", condition=IfCondition(LaunchConfiguration("use_drone")),
        parameters=[{"config_file": PathJoinSubstitution([config_dir, "gazebo_bridge_drone1.yaml"]),
                     "use_sim_time": use_sim_time}],
    )
    spawn_drone = TimerAction(period=4.0, actions=[Node(
        package="ros_ign_gazebo", executable="create", name="spawn_drone1", output="screen",
        condition=IfCondition(LaunchConfiguration("use_drone")),
        arguments=["-file", PathJoinSubstitution([model_dir, "scout_drone", "model.sdf"]),
                   "-name", "drone1", "-x", "-14", "-y", "-12", "-z", "8"],
    )])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="husky1",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    spawn_husky = TimerAction(
        period=3.0,
        actions=[Node(
            package="ros_ign_gazebo",
            executable="create",
            name="spawn_husky1",
            output="screen",
            arguments=[
                "-topic", "/husky1/robot_description",
                "-name", "husky1",
                "-x", "-14.0",
                "-y", "-14.0",
                "-z", "0.4",
                "-Y", "0.7854",
            ],
        )],
    )

    robot_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        namespace="husky1",
        name="gazebo_bridge",
        output="screen",
        parameters=[{
            "config_file": PathJoinSubstitution([config_dir, "gazebo_bridge_husky1.yaml"]),
            "use_sim_time": use_sim_time,
        }],
    )

    clock_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        parameters=[{
            "config_file": PathJoinSubstitution([config_dir, "gazebo_bridge_clock.yaml"]),
            "use_sim_time": use_sim_time,
        }],
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        namespace="husky1",
        name="robot_localization",
        output="screen",
        parameters=[
            PathJoinSubstitution([config_dir, "robot_localization_husky1.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("odometry/filtered", "odom"),
        ],
    )

    navigation = TimerAction(
        period=7.0,
        condition=IfCondition(use_nav2),
        actions=[GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([package_share, "launch", "navigation.launch.py"])
            ),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        )])],
    )

    rviz = TimerAction(
        period=8.0,
        condition=IfCondition(use_rviz),
        actions=[Node(
            package="rviz2",
            executable="rviz2",
            namespace="husky1",
            name="embertrace_rviz",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=["-d", PathJoinSubstitution([package_share, "rviz", "embertrace.rviz"])],
            remappings=[("/tf", "/husky1/tf"), ("/tf_static", "/husky1/tf_static")],
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_drone", default_value="true"),
        DeclareLaunchArgument("scenario", default_value="fire"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_nav2", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            [model_dir, ":", EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value="")],
        ),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            [model_dir, ":", EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
        ),
        SetEnvironmentVariable(
            "IGN_GAZEBO_SERVER_CONFIG_PATH",
            PathJoinSubstitution([config_dir, "ignition_server.config"]),
        ),
        SetEnvironmentVariable(
            "GZ_SIM_SERVER_CONFIG_PATH",
            PathJoinSubstitution([config_dir, "ignition_server.config"]),
        ),
        gazebo,
        clock_bridge,
        robot_state_publisher,
        robot_bridge,
        ekf,
        spawn_husky,
        drone_bridge,
        spawn_drone,
        navigation,
        rviz,
    ])


