"""Start namespaced SLAM Toolbox and Nav2 for the EmberTrace Husky."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap
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

    nav2 = GroupAction(scoped=True, actions=namespaced_remaps() + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"
                ])
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": PathJoinSubstitution([
                    config_dir, "nav2_params_husky1.yaml"
                ]),
                "autostart": "true",
            }.items(),
        )
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        slam,
        nav2,
    ])
