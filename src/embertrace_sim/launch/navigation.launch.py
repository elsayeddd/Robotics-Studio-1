"""Start namespaced SLAM Toolbox and Nav2 for the EmberTrace Husky."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_dir = PathJoinSubstitution([FindPackageShare("embertrace_sim"), "config"])

    def namespaced_remaps():
        # Return fresh launch-action objects for the independent groups.
        return [
            PushRosNamespace("husky1"),
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            SetRemap(src="/map", dst="map"),
            SetRemap(src="/map_updates", dst="map_updates"),
        ]

    slam = GroupAction(scoped=True, actions=namespaced_remaps() + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"
                ])
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "slam_params_file": PathJoinSubstitution([
                    config_dir, "slam_params_husky1.yaml"
                ]),
                "autostart": "true",
                "use_lifecycle_manager": "true",
            }.items(),
        )
    ])

    # Explicit velocity routing prevents a second Nav2 publisher bypassing the guard.
    # Keep Humble's controller -> velocity smoother chain intact.
    params = PathJoinSubstitution([config_dir, "nav2_params_husky1.yaml"])
    stack = [
        ("nav2_controller", "controller_server", "controller_server"),
        ("nav2_smoother", "smoother_server", "smoother_server"),
        ("nav2_planner", "planner_server", "planner_server"),
        ("nav2_behaviors", "behavior_server", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
        ("nav2_waypoint_follower", "waypoint_follower", "waypoint_follower"),
        ("nav2_velocity_smoother", "velocity_smoother", "velocity_smoother"),
    ]
    actions = namespaced_remaps()
    for package, executable, name in stack:
        remaps = []
        if name in ("controller_server", "behavior_server"):
            remaps = [("cmd_vel", "cmd_vel_nav")]
        elif name == "velocity_smoother":
            remaps = [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "/husky1/autonomy_cmd_vel")]
        actions.append(Node(package=package, executable=executable, name=name,
                            output="screen", parameters=[params, {"use_sim_time": use_sim_time}],
                            remappings=remaps))
    actions.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                        name="lifecycle_manager_navigation", output="screen",
                        parameters=[{"use_sim_time": use_sim_time, "autostart": True,
                                     "node_names": [name for _, _, name in stack]}]))
    nav2 = GroupAction(scoped=True, actions=actions)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        slam,
        nav2,
    ])

