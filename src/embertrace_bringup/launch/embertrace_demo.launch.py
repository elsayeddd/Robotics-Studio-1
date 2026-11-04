"""Run a standalone rover, standalone aircraft, or coordinated mission."""
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEFAULTS = {
    'mission_platform': 'combined', 'use_drone': 'true', 'use_nav2': 'true',
    'use_sim_time': 'true', 'use_thermal': 'true', 'use_rviz': 'true',
    'use_dashboard': 'true', 'gazebo_gui': 'true', 'world_file': 'embertrace_forest.sdf',
    'scenario': 'fire', 'random_seed': '42', 'drone_random_seed': '43',
    'dropout_start_s': '25.0', 'dropout_duration_s': '8.0',
    'rover_x': '-14.0', 'rover_y': '-14.0', 'drone_x': '-14.0', 'drone_y': '-12.0',
    'wind_speed_mps': '2.5',
}


def _stack(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    enabled = lambda key: value(key).lower() == 'true'
    platform = value('mission_platform')
    rover = platform != 'drone'
    drone = platform != 'rover' and enabled('use_drone')
    if platform == 'drone' and not drone:
        raise ValueError('A standalone drone mission requires use_drone:=true')
    wind = float(value('wind_speed_mps'))
    if not math.isfinite(wind) or wind < 0:
        raise ValueError('wind_speed_mps must be finite and non-negative')
    nav2 = rover and enabled('use_nav2')
    sim_time, thermal = enabled('use_sim_time'), enabled('use_thermal')
    parameters = PathJoinSubstitution([FindPackageShare('embertrace_bringup'), 'config', 'embertrace.yaml'])
    route = PathJoinSubstitution([FindPackageShare('embertrace_bringup'), 'config', 'patrol_waypoints.yaml'])
    tf = [('/tf', '/husky1/tf'), ('/tf_static', '/husky1/tf_static')]
    primary = [] if rover else [
        ('/embertrace/environment', '/drone1/environment'),
        ('/embertrace/perception_status', '/drone1/perception_status'),
        ('/drone1/environment', '/embertrace/unused_scout_environment'),
    ]
    def core(executable, name=None, extra=None, remaps=None):
        return Node(package='embertrace_core', executable=executable, name=name or executable,
                    output='screen', parameters=[parameters, {'use_sim_time': sim_time}, extra or {}],
                    remappings=remaps or [])

    nodes = []
    for aerial, active in [(False, rover), (True, drone)]:
        if not active:
            continue
        prefix = 'drone_' if aerial else ''
        if thermal:
            nodes.append(core('thermal_verifier_node', prefix + 'thermal_verifier_node'))
        nodes += [
            core('sensor_health_node', prefix + 'sensor_health_node', {'require_thermal': thermal}),
            core('environment_simulator_node', prefix + 'environment_simulator_node', {
                'scenario': value('scenario'),
                'random_seed': int(value('drone_random_seed' if aerial else 'random_seed')),
                'dropout_start_s': float(value('dropout_start_s')),
                'dropout_duration_s': float(value('dropout_duration_s')),
                'wind_speed_mps': float(value('wind_speed_mps')),
            }),
            core('camera_verifier_node', prefix + 'camera_verifier_node'),
            core('perception_node', prefix + 'perception_node', {'require_thermal': thermal}),
        ]
    if rover:
        nodes += [core('velocity_guard_node'),
                  core('patrol_manager_node', extra={'waypoints_file': route}, remaps=tf)]
    if drone:
        nodes.append(core('drone_scout_node', extra={
            'autonomous_mission': not rover,
            'base_x': float(value('drone_x')), 'base_y': float(value('drone_y')),
        }, remaps=tf))
    nodes += [
        core('mission_manager_node', extra={
            'require_thermal_confirmation': thermal, 'require_scout_return': rover and drone,
            'aerial_mission': not rover,
        }, remaps=primary),
        core('search_planner_node', extra={'require_map': rover}, remaps=tf + primary),
        core('heatmap_node', remaps=primary),
    ]
    if nav2:
        nodes.append(core('nav2_goal_adapter_node', remaps=tf))
    simulation = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare('embertrace_sim'), 'launch', 'simulation.launch.py'])),
        launch_arguments={
            **{key: value(key) for key in ('world_file', 'scenario', 'gazebo_gui',
                                         'rover_x', 'rover_y', 'drone_x', 'drone_y',
                                         'use_sim_time', 'use_rviz')},
            'use_rover': str(rover).lower(), 'use_drone': str(drone).lower(),
            'use_nav2': str(nav2).lower(),
        }.items())
    actions = [simulation, TimerAction(period=2.0, actions=nodes)]
    if enabled('use_dashboard'):
        actions.append(TimerAction(period=5.0, actions=[Node(
            package='embertrace_ui', executable='operator_dashboard', name='operator_dashboard',
            output='screen', parameters=[{'use_sim_time': sim_time}], remappings=primary)]))
    return actions


def generate_launch_description():
    arguments = [DeclareLaunchArgument(key, default_value=value,
        **({'choices': ['combined', 'rover', 'drone']} if key == 'mission_platform' else {}))
        for key, value in DEFAULTS.items()]
    return LaunchDescription(arguments + [OpaqueFunction(function=_stack)])
