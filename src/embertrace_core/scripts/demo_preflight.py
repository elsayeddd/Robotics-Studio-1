#!/usr/bin/env python3
"""Read-only demo readiness check. Uses wall timeout, including when /clock is absent."""
import argparse
import math
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.srv import GetState
from embertrace_interfaces.msg import PerceptionStatus, MissionState
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=30)
    parser.add_argument('--no-drone', action='store_true')
    parser.add_argument('--no-nav2', action='store_true')
    parser.add_argument('--no-thermal', action='store_true', help='Explicit reduced sensor debug mode')
    parser.add_argument('--waypoints-file')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node('demo_preflight', cli_args=[
        '--ros-args', '-r', '/tf:=/husky1/tf', '-r', '/tf_static:=/husky1/tf_static'])
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    checks = {'rover sensors': False, 'rover perception': False, 'mission heartbeat': False}
    if not args.no_drone:
        checks.update({'drone sensors': False, 'drone perception': False})
    if not args.no_nav2:
        checks.update({'occupancy map': False, 'map to odom TF': False, 'Nav2 action': False,
                       'Nav2 lifecycle active': False, 'first patrol path': False})
        waypoints_file = args.waypoints_file or str(Path(get_package_share_directory('embertrace_bringup')) /
                                                   'config/patrol_waypoints.yaml')
        route = yaml.safe_load(Path(waypoints_file).read_text())
        waypoint = route['waypoints'][min(1, len(route['waypoints']) - 1)]
    last_seen = {}
    subscriptions = []
    def update(key, valid):
        checks[key] = valid
        last_seen[key] = time.monotonic()
    for key, topic in [('rover sensors', '/embertrace/sensors_ready'), ('drone sensors', '/drone1/sensors_ready')]:
        if key in checks:
            subscriptions.append(node.create_subscription(Bool, topic,
                lambda msg, key=key: update(key, msg.data), 10))
    for key, topic in [('rover perception', '/embertrace/perception_status'), ('drone perception', '/drone1/perception_status')]:
        if key in checks:
            subscriptions.append(node.create_subscription(PerceptionStatus, topic,
                lambda msg, key=key: update(key, msg.environment_fresh and msg.co2_valid and (args.no_thermal or msg.thermal_fresh)), 10))
    transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions.append(node.create_subscription(MissionState, '/embertrace/mission_state',
        lambda msg: update('mission heartbeat', True), transient))
    if not args.no_nav2:
        subscriptions.append(node.create_subscription(OccupancyGrid, '/husky1/map',
            lambda msg: update('occupancy map', bool(msg.data) and msg.header.frame_id == 'husky1_map'), transient))
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    action = ActionClient(node, NavigateToPose, '/husky1/navigate_to_pose')
    planner = ActionClient(node, ComputePathToPose, '/husky1/compute_path_to_pose')
    path_goal = path_result = None
    last_path_attempt = 0
    clients = [] if args.no_nav2 else [node.create_client(GetState, '/husky1/' + name + '/get_state')
        for name in ('controller_server', 'planner_server', 'smoother_server', 'behavior_server',
                     'bt_navigator', 'waypoint_follower', 'velocity_smoother')]
    pending = {}
    active = {client: False for client in clients}
    last_poll = 0
    start = time.monotonic()
    while rclpy.ok() and time.monotonic() - start < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if not args.no_nav2:
            checks['Nav2 action'] = action.server_is_ready()
            checks['map to odom TF'] = buffer.can_transform('husky1_map', 'husky1_odom', rclpy.time.Time())
            for client, future in list(pending.items()):
                if future.done():
                    result = future.result() if future.exception() is None else None
                    active[client] = result is not None and result.current_state.id == 3
                    del pending[client]
            if time.monotonic() - last_poll >= 1:
                for client in clients:
                    if client not in pending and client.service_is_ready():
                        pending[client] = client.call_async(GetState.Request())
                last_poll = time.monotonic()
            checks['Nav2 lifecycle active'] = all(active.values())
            # An active planner can still have an incomplete initial SLAM map.
            # Prove the first moving patrol leg is plannable before START.
            if path_goal is not None and path_goal.done():
                handle = path_goal.result() if path_goal.exception() is None else None
                path_result = handle.get_result_async() if handle is not None and handle.accepted else None
                path_goal = None
            if path_result is not None and path_result.done():
                result = path_result.result() if path_result.exception() is None else None
                checks['first patrol path'] = (result is not None and result.status == 4 and
                                                len(result.result.path.poses) > 1)
                path_result = None
            if (checks['Nav2 lifecycle active'] and not checks['first patrol path'] and
                    path_goal is None and path_result is None and planner.server_is_ready() and
                    time.monotonic() - last_path_attempt >= 2 and
                    buffer.can_transform('husky1_map', route['frame_id'], rclpy.time.Time())):
                transform = buffer.lookup_transform('husky1_map', route['frame_id'], rclpy.time.Time())
                q, t = transform.transform.rotation, transform.transform.translation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                goal = ComputePathToPose.Goal()
                goal.goal.header.frame_id = 'husky1_map'
                goal.goal.header.stamp = node.get_clock().now().to_msg()
                x, y = float(waypoint['x']), float(waypoint['y'])
                goal.goal.pose.position.x = t.x + math.cos(yaw) * x - math.sin(yaw) * y
                goal.goal.pose.position.y = t.y + math.sin(yaw) * x + math.cos(yaw) * y
                angle = yaw + float(waypoint.get('yaw', 0))
                goal.goal.pose.orientation.z = math.sin(angle / 2)
                goal.goal.pose.orientation.w = math.cos(angle / 2)
                goal.planner_id = 'GridBased'
                goal.use_start = False
                path_goal = planner.send_goal_async(goal)
                last_path_attempt = time.monotonic()
        for key, seen in list(last_seen.items()):
            if key != 'occupancy map' and time.monotonic() - seen > 2:
                checks[key] = False
        if all(checks.values()):
            break
    for key, valid in checks.items():
        print(('PASS' if valid else 'FAIL') + ': ' + key)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if all(checks.values()) else 1)


if __name__ == '__main__':
    main()
