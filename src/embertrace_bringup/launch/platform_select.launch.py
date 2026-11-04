"""Select the complete mission stack; auto uses the configured measured-wind fixture."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import math


def _select(context):
    platform = LaunchConfiguration('platform').perform(context)
    wind = float(LaunchConfiguration('wind_speed_mps').perform(context))
    if not math.isfinite(wind) or wind < 0:
        raise ValueError('wind_speed_mps must be finite and non-negative')
    if platform == 'auto':
        if wind > 12:
            raise ValueError('Wind exceeds both vehicle limits; no mission can start')
        platform = 'drone' if wind <= 5 else 'rover'
    platform = {'husky': 'rover', 'quad': 'drone', 'parrot': 'drone'}.get(platform, platform)
    return [LogInfo(msg=f'Starting {platform} mission at {wind:.1f} m/s wind'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('embertrace_bringup'), 'launch', 'embertrace_demo.launch.py'])),
                launch_arguments={'mission_platform': platform,
                                  'wind_speed_mps': str(wind)}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('platform', default_value='auto',
                              choices=['auto', 'husky', 'quad', 'parrot', 'rover', 'drone', 'combined']),
        DeclareLaunchArgument('wind_speed_mps', default_value='2.5'),
        OpaqueFunction(function=_select),
    ])
