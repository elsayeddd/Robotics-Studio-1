"""Top-level EmberTrace launch file used by the team and final demonstration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_rviz = LaunchConfiguration("use_rviz")
    use_dashboard = LaunchConfiguration("use_dashboard")
    world_file = LaunchConfiguration("world_file")
    parameters = PathJoinSubstitution([
        FindPackageShare("embertrace_bringup"), "config", "embertrace.yaml"
    ])
    patrol_waypoints_file = PathJoinSubstitution([
        FindPackageShare("embertrace_bringup"), "config", "patrol_waypoints.yaml"
    ])

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("embertrace_sim"), "launch", "simulation.launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_nav2": use_nav2,
            "use_rviz": use_rviz,
            "world_file": world_file,
        }.items(),
    )

    core_nodes = TimerAction(period=2.0, actions=[
        Node(
            package="embertrace_core",
            executable="environment_simulator_node",
            name="environment_simulator_node",
            output="screen",
            parameters=[parameters, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="embertrace_core",
            executable="camera_verifier_node",
            name="camera_verifier_node",
            output="screen",
            parameters=[parameters, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="embertrace_core",
            executable="perception_node",
            name="perception_node",
            output="screen",
            parameters=[parameters, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="embertrace_core",
            executable="mission_manager_node",
            name="mission_manager_node",
            output="screen",
            parameters=[parameters, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="embertrace_core",
            executable="search_planner_node",
            name="search_planner_node",
            output="screen",
            parameters=[parameters, {"use_sim_time": use_sim_time}],
            # Simulation and SLAM publish TF within the Husky namespace.
            remappings=[("/tf", "/husky1/tf"), ("/tf_static", "/husky1/tf_static")],
        ),
        Node(
            package="embertrace_core",
            executable="heatmap_node",
            name="heatmap_node",
            output="screen",
            parameters=[parameters, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="embertrace_core",
            executable="patrol_manager_node",
            name="patrol_manager_node",
            output="screen",
            parameters=[
                parameters,
                {"use_sim_time": use_sim_time, "waypoints_file": patrol_waypoints_file},
            ],
            remappings=[("/tf", "/husky1/tf"), ("/tf_static", "/husky1/tf_static")],
        ),
        Node(
            package="embertrace_core",
            executable="nav2_goal_adapter_node",
            name="nav2_goal_adapter_node",
            output="screen",
            condition=IfCondition(use_nav2),
            parameters=[parameters, {"use_sim_time": use_sim_time}],
            remappings=[("/tf", "/husky1/tf"), ("/tf_static", "/husky1/tf_static")],
        ),
    ])

    dashboard = TimerAction(
        period=5.0,
        condition=IfCondition(use_dashboard),
        actions=[Node(
            package="embertrace_ui",
            executable="operator_dashboard",
            name="operator_dashboard",
            output="screen",
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "use_nav2",
            default_value="true",
            description=(
                "Start SLAM, Nav2 and nav2_goal_adapter_node. Required for the rover "
                "to move; set false only to run the sim without navigation."
            ),
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_dashboard", default_value="true"),
        DeclareLaunchArgument(
            "world_file",
            default_value="embertrace_forest.sdf",
            description=(
                "World SDF filename under embertrace_sim/worlds/ to load. "
                "Use the small/sparse default for fast iteration, or "
                "'embertrace_forest_dense.sdf' for the larger final-demo world."
            ),
        ),
        simulation,
        core_nodes,
        dashboard,
    ])