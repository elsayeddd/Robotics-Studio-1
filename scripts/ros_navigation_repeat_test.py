#!/usr/bin/env python3
"""Exercise repeated targets, changed targets, stop and retry on a real Nav2 action."""
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_prefix
from embertrace_interfaces.msg import MissionState
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


def main():
    rclpy.init()
    node = Node('navigation_repeat_regression')
    targets = node.create_publisher(PoseStamped, '/embertrace/navigation_goal', 10)
    mission = node.create_publisher(MissionState, '/embertrace/mission_state',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    accepted, cancelled = [], []
    statuses = []
    status_subscription = node.create_subscription(String, '/embertrace/navigation_status',
        lambda msg: statuses.append(msg.data), 10)
    finish = threading.Event()

    def execute(handle):
        accepted.append(handle.request.pose)
        while rclpy.ok() and not finish.is_set() and not handle.is_cancel_requested:
            time.sleep(.01)
        if handle.is_cancel_requested:
            cancelled.append(handle.request.pose)
            handle.canceled()
        else:
            handle.succeed()
        return NavigateToPose.Result()

    server = ActionServer(node, NavigateToPose, '/husky1/navigate_to_pose', execute,
        callback_group=ReentrantCallbackGroup(), goal_callback=lambda _: GoalResponse.ACCEPT,
        cancel_callback=lambda _: CancelResponse.ACCEPT)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def state(allowed=True, epoch=1):
        msg = MissionState()
        msg.state = msg.TRACK if allowed else msg.PAUSED
        msg.active_platform = msg.PLATFORM_ROVER
        msg.motion_allowed = allowed
        msg.mission_epoch = epoch
        mission.publish(msg)
        time.sleep(.15)

    def goal(x=5.):
        msg = PoseStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'husky1_map'
        msg.pose.position.x = x
        msg.pose.orientation.w = 1.
        targets.publish(msg)

    def wait_for(predicate, failure):
        end = time.monotonic()+4
        while not predicate() and time.monotonic() < end:
            time.sleep(.02)
        assert predicate(), failure

    root = Path('log/navigation-repeat-regression')
    root.mkdir(parents=True, exist_ok=True)
    executable = Path(get_package_prefix('embertrace_core')) / 'lib/embertrace_core/nav2_goal_adapter_node'
    with (root / 'adapter.log').open('w') as log:
        process = subprocess.Popen([str(executable), '--ros-args', '-p', 'boundary_frame:=husky1_map'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            wait_for(lambda: targets.get_subscription_count() and mission.get_subscription_count(), 'Adapter not discovered')
            state()
            goal()
            wait_for(lambda: len(accepted) == 1, 'First action not accepted')
            for _ in range(12):
                goal(5.02)
                time.sleep(.1)
            assert len(accepted) == 1 and not cancelled, 'Repeated target restarted the action/progress watchdog'
            goal(7.)
            wait_for(lambda: len(accepted) == 2 and len(cancelled) == 1, 'Changed target did not replace the action')
            state(False)
            wait_for(lambda: len(cancelled) == 2, 'PAUSE did not cancel the action')
            state(epoch=2)
            goal(7.)
            wait_for(lambda: len(accepted) == 3, 'New mission could not reuse a previous target')
            finish.set()
            wait_for(lambda: 'SUCCEEDED' in statuses, 'Terminal result was not reported')
            goal(7.)
            wait_for(lambda: len(accepted) == 4, 'Terminal action prevented retry of the same target')
            print('Navigation repeat DDS regression passed: duplicates preserved; changes, PAUSE, epochs and terminal retries honoured', flush=True)
        finally:
            finish.set()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            executor.shutdown(timeout_sec=3)
            thread.join(timeout=3)
            server.destroy()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
