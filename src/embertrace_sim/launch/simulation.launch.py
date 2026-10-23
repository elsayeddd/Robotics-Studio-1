"""Launch the self-contained EmberTrace forest and one Husky UGV."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
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
    world_file = LaunchConfiguration("world_file")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([package_share, "urdf", "husky.urdf.xacro"]),
            " prefix:=husky1_ gz_model_name:=husky1",
        ]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_ign_gazebo"),
                "launch",
                "ign_gazebo.launch.py",
            ])
        ),
        launch_arguments={
            "ign_args": [
                PathJoinSubstitution([package_share, "worlds", world_file]),
                " -r",
            ]
        }.items(),
    )

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
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_nav2", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "world_file",
            default_value="embertrace_forest.sdf",
            description=(
                "World SDF filename under embertrace_sim/worlds/ to load. "
                "Use the small/sparse default for fast iteration, or "
                "'embertrace_forest_dense.sdf' for the larger final-demo world."
            ),
        ),
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
        navigation,
        rviz,
    ])