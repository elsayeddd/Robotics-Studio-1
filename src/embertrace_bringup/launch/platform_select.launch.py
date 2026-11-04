"""Convenience entry point for the complete, measured-wind mission system."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def select(context):
    platform = LaunchConfiguration("platform").perform(context)
    wind = LaunchConfiguration("wind_speed_mps").perform(context)
    if platform not in ("auto", "husky", "quad"):
        raise ValueError("platform must be auto, husky or quad")
    # Explicit demonstration presets still pass through the same wind/sensor policy.
    if platform == "husky":
        wind = "7.0"
    elif platform == "quad":
        wind = "2.5"
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("embertrace_bringup"),
                        "launch",
                        "embertrace_demo.launch.py",
                    ]
                )
            ),
            launch_arguments={"wind_speed_mps": wind, "mission_platform": {"husky": "rover", "quad": "drone", "auto": "auto"}[platform]}.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("platform", default_value="auto"),
            DeclareLaunchArgument("wind_speed_mps", default_value="2.5"),
            OpaqueFunction(function=select),
        ]
    )
