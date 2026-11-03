## Random roll (normal operation) — 0-30, husky if >15, quad if <15
#ros2 launch embertrace_bringup platform_select.launch.py
#
## Force Husky (no randomness)
#ros2 launch embertrace_bringup platform_select.launch.py platform:=husky
#
## Force quadcopter (no randomness)
#ros2 launch embertrace_bringup platform_select.launch.py platform:=quad

import random

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PLATFORMS = {
    "husky": {
        "namespace": "husky1",
        "map_frame": "husky1_map",
        "odom_frame": "husky1_odom",
        "odom_topic": "/husky1/odometry",
        "action_name": "/husky1/navigate_to_pose",
    },
    "parrot": {
        "namespace": "parrot1",
        "map_frame": "parrot1_map",
        "odom_frame": "parrot1_odom",
        "odom_topic": "/parrot1/odometry",
        "action_name": "/parrot1/navigate_to_pose",
    },
}

def _tf_remappings(namespace):
    #Nodes that consume TF must listen to the *selected* robot's TF tree.
    return [("/tf", f"/{namespace}/tf"), ("/tf_static", f"/{namespace}/tf_static")]


def _adapter_node(platform, parameters, use_sim_time):
    #Nav2 goal adapter configured for the selected platform.
    cfg = PLATFORMS[platform]
    return Node(
        package="embertrace_core",
        executable="nav2_goal_adapter_node",
        name="nav2_goal_adapter",
        output="screen",
        parameters=[
            parameters,
            {
                "use_sim_time": use_sim_time,
                "robot_namespace": cfg["namespace"],
                "map_frame": cfg["map_frame"],
                "odom_frame": cfg["odom_frame"],
                "odom_topic": cfg["odom_topic"],
                "action_name": cfg["action_name"],
            },
        ],
        remappings=_tf_remappings(cfg["namespace"]),
    )


def _core_nodes(platform, parameters, patrol_waypoints_file, use_sim_time):
    #Core stack, with TF remappings derived from the resolved platform.
    remaps = _tf_remappings(PLATFORMS[platform]["namespace"])
    common = {"use_sim_time": use_sim_time}

    return[
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
            remappings=remaps,
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
            remappings=remaps,
        ),
    ]


def _select_platform(context, *args, **kwargs):
    requested = LaunchConfiguration("platform").perform(context).strip()

    if requested in PLATFORMS:
        return requested, f"[platform_select] forced platform: {requested}"

    if requested != "auto":
        return "husky", (
            f"[platform_select] invalid platform '{requested}' "
            f"(expected 'auto', 'husky' or 'quad'); defaulting to 'husky'"
        )

    rng = random.Random()
    roll = rng.randint(0, 30)
    # Rolls >= 15 pick Husky: 15 m/s winds are too fast for the quadcopter.
    platform = "husky" if roll >= 15 else "quad"
    return platform, (
        f"[platform_select] rolled {roll} (0-30) -> {platform}"
    )

def _selected_platform_actions(context, parameters, patrol_waypoints_file, use_sim_time):
    #Build the platform-dependent half of the launch description.
    platform, message = _select_platform(context)
    banner = "HUSKY" if platform == "husky" else "QUADCOPTER"

    return [
        LogInfo(msg=message),
        LogInfo(msg=f"[platform_select] launching {banner}"),
        TimerAction(
            period=2.0,
            actions=_core_nodes(platform, parameters, patrol_waypoints_file, use_sim_time)
            + [_adapter_node(platform, parameters, use_sim_time)],
        ),
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_rviz = LaunchConfiguration("use_rviz")
    use_dashboard = LaunchConfiguration("use_dashboard")
    platform = LaunchConfiguration("platform")

    parameters = PathJoinSubstitution(
        [FindPackageShare("embertrace_bringup"), "config", "embertrace.yaml"]
    )
    patrol_waypoints_file = PathJoinSubstitution(
        [FindPackageShare("embertrace_bringup"), "config", "patrol_waypoints.yaml"]
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("embertrace_sim"),
                    "launch",
                    "simulation.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_nav2": use_nav2,
            "use_rviz": use_rviz,
            "platform": platform,
        }.items(),
    )

    dashboard = TimerAction(
        period=5.0,
        condition=IfCondition(use_dashboard),
        actions=[
            Node(
                package="embertrace_ui",
                executable="operator_dashboard",
                name="operator_dashboard",
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_nav2", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_dashboard", default_value="true"),
            DeclareLaunchArgument(
                "platform",
                default_value="auto",
                description="'auto' = random roll, or force 'husky' / 'quad'",
            ),
            OpaqueFunction(
                function=lambda context: _selected_platform_actions(
                    context, parameters, patrol_waypoints_file, use_sim_time
                )
            ),
            simulation,
            dashboard,
        ]
    )
