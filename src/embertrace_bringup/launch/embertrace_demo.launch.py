"""Top-level EmberTrace launch file used by the team and final demonstration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
            "use_drone": LaunchConfiguration("use_drone"),
            "scenario": LaunchConfiguration("scenario"),
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
            **{key: LaunchConfiguration(key) for key in ("rover_x", "rover_y", "drone_x", "drone_y")},
        }.items(),
    )

    core_nodes = TimerAction(period=2.0, actions=[
        Node(package="embertrace_core", executable="thermal_verifier_node", name="thermal_verifier_node",
             output="screen", condition=IfCondition(LaunchConfiguration("use_thermal")),
             parameters=[parameters, {"use_sim_time": use_sim_time}]),
        Node(package="embertrace_core", executable="thermal_verifier_node", name="drone_thermal_verifier_node",
             output="screen", condition=IfCondition(PythonExpression(["'", LaunchConfiguration("use_drone"), "' == 'true' and '", LaunchConfiguration("use_thermal"), "' == 'true'"])),
             parameters=[parameters, {"use_sim_time": use_sim_time}]),
        Node(package="embertrace_core", executable="velocity_guard_node", name="velocity_guard_node",
             output="screen", parameters=[parameters, {"use_sim_time": use_sim_time}]),
        Node(package="embertrace_core", executable="sensor_health_node", name="sensor_health_node",
             output="screen", parameters=[parameters, {"use_sim_time": use_sim_time, "require_thermal": LaunchConfiguration("use_thermal")}]),
        Node(package="embertrace_core", executable="drone_scout_node", name="drone_scout_node",
             output="screen", condition=IfCondition(LaunchConfiguration("use_drone")),
             parameters=[parameters, {"use_sim_time": use_sim_time}],
             remappings=[("/tf", "/husky1/tf"), ("/tf_static", "/husky1/tf_static")]),
        Node(package="embertrace_core", executable="sensor_health_node", name="drone_sensor_health_node",
             output="screen", condition=IfCondition(LaunchConfiguration("use_drone")),
             parameters=[parameters, {"use_sim_time": use_sim_time, "require_thermal": LaunchConfiguration("use_thermal")}]),
        Node(package="embertrace_core", executable="environment_simulator_node", name="drone_environment_simulator_node",
             output="screen", condition=IfCondition(LaunchConfiguration("use_drone")),
             parameters=[parameters, {"use_sim_time": use_sim_time, "scenario": LaunchConfiguration("scenario"),
                         "random_seed": LaunchConfiguration("drone_random_seed"),
                         "dropout_start_s": LaunchConfiguration("dropout_start_s"),
                         "dropout_duration_s": LaunchConfiguration("dropout_duration_s")}]),
        Node(package="embertrace_core", executable="camera_verifier_node", name="drone_camera_verifier_node",
             output="screen", condition=IfCondition(LaunchConfiguration("use_drone")),
             parameters=[parameters, {"use_sim_time": use_sim_time}]),
        Node(package="embertrace_core", executable="perception_node", name="drone_perception_node",
             output="screen", condition=IfCondition(LaunchConfiguration("use_drone")),
             parameters=[parameters, {"use_sim_time": use_sim_time, "require_thermal": LaunchConfiguration("use_thermal")}]),
        Node(
            package="embertrace_core",
            executable="environment_simulator_node",
            name="environment_simulator_node",
            output="screen",
            parameters=[parameters, {"use_sim_time": use_sim_time, "scenario": LaunchConfiguration("scenario"),
                        "random_seed": LaunchConfiguration("random_seed"),
                        "dropout_start_s": LaunchConfiguration("dropout_start_s"),
                        "dropout_duration_s": LaunchConfiguration("dropout_duration_s")}],
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
            parameters=[parameters, {"use_sim_time": use_sim_time, "require_thermal": LaunchConfiguration("use_thermal")}],
        ),
        Node(
            package="embertrace_core",
            executable="mission_manager_node",
            name="mission_manager_node",
            output="screen",
            parameters=[parameters, {"use_sim_time": use_sim_time, "require_thermal_confirmation": LaunchConfiguration("use_thermal")}],
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
            parameters=[{"use_sim_time": use_sim_time}],
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_thermal", default_value="true", choices=["true", "false"],
                              description="Require native IR sensor; false is a labelled reduced debug mode"),
        DeclareLaunchArgument("use_drone", default_value="true"),
        DeclareLaunchArgument("gazebo_gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("random_seed", default_value="42"),
        DeclareLaunchArgument("drone_random_seed", default_value="43"),
        DeclareLaunchArgument("dropout_start_s", default_value="25.0"),
        DeclareLaunchArgument("dropout_duration_s", default_value="8.0"),
        DeclareLaunchArgument("rover_x", default_value="-14.0"),
        DeclareLaunchArgument("rover_y", default_value="-14.0"),
        DeclareLaunchArgument("drone_x", default_value="-14.0"),
        DeclareLaunchArgument("drone_y", default_value="-12.0"),
        DeclareLaunchArgument("scenario", default_value="fire"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_nav2", default_value="true"),
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
