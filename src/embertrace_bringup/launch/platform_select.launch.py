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
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
            'robot_namespace': 'quad1',
            'map_frame': 'quad1_map',
            'odom_frame': 'quad1_odom',
            'odom_topic': '/quad1/odometry',
            'action_name': '/quad1/navigate_to_pose',
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
    return LaunchDescription([
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