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

def _husky_node():
    return Node(
        package='embertrace_core',
        executable='nav2_goal_adapter_node',
        name='nav2_goal_adapter',
        output='screen',
        parameters=[{
            'robot_namespace': 'husky1',
            'map_frame': 'husky1_map',
            'odom_frame': 'husky1_odom',
            'odom_topic': '/husky1/odometry',
            'action_name': '/husky1/navigate_to_pose',
        }],
    )


def _quad_node():
    return Node(
        package='embertrace_core',
        executable='nav2_goal_adapter_node',
        name='nav2_goal_adapter',
        output='screen',
        parameters=[{
            'robot_namespace': 'parrot1',
            'map_frame': 'parrot1_map',
            'odom_frame': 'parrot1_odom',
            'odom_topic': '/parrot1/odometry',
            'action_name': '/parrot1/navigate_to_pose',
        }],
    )


def _select_platform(context, *args, **kwargs):
    platform = LaunchConfiguration('platform').perform(context)
    seed = LaunchConfiguration('seed').perform(context)

    actions = []

    if platform == 'auto':
        
        try:
            rng = random.Random(int(seed)) if seed else random.Random()
        except ValueError:
            actions.append(
                LogInfo(msg=f"[platform_select] invalid seed '{seed}', using random")
            )
            rng = random.Random()

        roll = rng.randint(0, 30)
        
        actions.append(
            LogInfo(msg=f"[platform_select] rolled {roll} (range 0-30, seed={seed or 'random'})")
        )

        #15m/s winds are too fast for some quadcopters, so select husky
        if roll >= 15:
            platform = 'husky'
        elif roll < 15:
            platform = 'quad'
        else:
            return actions

    if platform == 'husky':
        actions.append(LogInfo(msg="[platform_select] launching HUSKY"))
        actions.append(_husky_node())
    elif platform == 'quad':
        actions.append(LogInfo(msg="[platform_select] launching QUADCOPTER"))
        actions.append(_quad_node())
    else:
    
        raise RuntimeError(
            f"[platform_select] invalid platform '{platform}' "
            "(expected 'auto', 'husky', or 'quad')"
        )

    return actions


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_rviz = LaunchConfiguration("use_rviz")
    use_dashboard = LaunchConfiguration("use_dashboard")
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
            #remappings=[("/tf", "/husky1/tf"), ("/tf_static", "/husky1/tf_static")],
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
            #remappings=[("/tf", "/husky1/tf"), ("/tf_static", "/husky1/tf_static")],
        ),
        Node(
            package="embertrace_core",
            executable="nav2_goal_adapter_node",
            name="nav2_goal_adapter_node",
            output="screen",
            condition=IfCondition(use_nav2),
            parameters=[parameters, {"use_sim_time": use_sim_time}],
            #remappings=[("/tf", "/husky1/tf"), ("/tf_static", "/husky1/tf_static")],
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
                DeclareLaunchArgument("use_nav2", default_value="false"),
                DeclareLaunchArgument("use_rviz", default_value="true"),
                DeclareLaunchArgument("use_dashboard", default_value="true"),
                simulation,
                core_nodes,
                dashboard,
        DeclareLaunchArgument(
            'platform',
            default_value='auto',
            description="'auto' = random roll, or force 'husky' / 'quad'",
        ),
        DeclareLaunchArgument(
            'seed',
            default_value='',
            description='Integer seed for a reproducible roll. Empty = fresh randomness.',
        ),
        OpaqueFunction(function=_select_platform),
    ])