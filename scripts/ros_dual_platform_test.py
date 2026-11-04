#!/usr/bin/env python3
"""Exercise both real ROS perception pipelines and wind-based mission selection."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Image
from embertrace_interfaces.msg import EnvironmentalReading, MissionState, FireAlert, PerceptionStatus
from embertrace_interfaces.srv import OperatorCommand


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw-delay', type=float, default=0)
    args = parser.parse_args()
    if not 0 <= args.raw_delay <= 1: parser.error('Raw delay must be between zero and one second')
    pending_raw = []
    rclpy.init()
    node = Node("dual_platform_test")
    state = {"mission": None, "alerts": [], "readings": {}, "health": {}}
    transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(
            MissionState,
            "/embertrace/mission_state",
            lambda message: state.update(mission=message),
            transient,
        ),
        node.create_subscription(
            FireAlert,
            "/embertrace/fire_alert",
            lambda message: state["alerts"].append(message),
            transient,
        ),
    ]
    for root in ("/embertrace", "/drone1"):
        subscriptions.append(node.create_subscription(EnvironmentalReading, root + "/environment",
            lambda m, root=root: state["readings"].update({root: (m.header.stamp, m.thermal_valid, m.co2_valid, m.fused_confidence)}), qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(PerceptionStatus, root + "/perception_status",
            lambda m, root=root: state["health"].update({root: (m.environment_fresh, m.thermal_fresh, m.co2_valid, m.rejected_samples)}), 10))
    wind = node.create_publisher(
        Vector3Stamped, "/embertrace/wind", qos_profile_sensor_data
    )
    raw, rgb, ir = [], [], []
    for root, robot in [("/embertrace", "/husky1"), ("/drone1", "/drone1")]:
        raw.append(
            node.create_publisher(
                EnvironmentalReading, root + "/raw_environment", qos_profile_sensor_data
            )
        )
        rgb.append(
            node.create_publisher(
                Image, robot + "/camera/image", qos_profile_sensor_data
            )
        )
        ir.append(
            node.create_publisher(
                Image, robot + "/thermal/image", qos_profile_sensor_data
            )
        )
    client = node.create_client(OperatorCommand, "/embertrace/operator_command")

    def pump(seconds, speed=2.5, fires=(False, False), visual=False):
        deadline, next_sample = time.monotonic() + seconds, 0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_sample:
                stamp = node.get_clock().now().to_msg()
                ambient = Vector3Stamped()
                ambient.header.stamp = stamp
                ambient.header.frame_id = "husky1_odom"
                ambient.vector.x = float(speed)
                wind.publish(ambient)
                for index in range(2):
                    reading = EnvironmentalReading()
                    reading.header.stamp = stamp
                    reading.header.frame_id = "husky1_odom"
                    reading.sample_pose.header = reading.header
                    reading.sample_pose.pose.orientation.w = 1.0
                    reading.smoke_ppm = 60.0 if fires[index] else 1.0
                    reading.temperature_c = 80.0 if fires[index] else 22.0
                    reading.co2_ppm = 1500.0 if fires[index] else 420.0
                    reading.co2_valid = True
                    reading.wind_speed_mps = float(speed)
                    pending_raw.append((time.monotonic() + args.raw_delay, index, reading))
                    frame = ("husky1" if index == 0 else "drone1") + "_camera_link"
                    image = Image()
                    image.header.stamp = stamp
                    image.header.frame_id = frame
                    image.width = image.height = 32
                    image.step = 96
                    image.encoding = "rgb8"
                    image.data = bytes([255, 100, 10] if visual else [0, 0, 0]) * 1024
                    rgb[index].publish(image)
                    thermal = Image()
                    thermal.header = image.header
                    thermal.width = thermal.height = 32
                    thermal.step = 64
                    thermal.encoding = "mono16"
                    value = round(((100 if fires[index] else 22) + 273.15) * 100)
                    thermal.data = value.to_bytes(2, "little") * 1024
                    ir[index].publish(thermal)
                next_sample = time.monotonic() + 0.2
            while pending_raw and pending_raw[0][0] <= time.monotonic():
                _, index, reading = pending_raw.pop(0)
                raw[index].publish(reading)
            rclpy.spin_once(node, timeout_sec=0.02)

    def confirmed(platform, speed, fires):
        deadline = time.monotonic() + 12
        previous_id = max((a.alert_id for a in state["alerts"]), default=0)
        while time.monotonic() < deadline:
            pump(.2, speed=speed, fires=fires)
            if (state["mission"] is not None and state["mission"].state == MissionState.ALERT
                    and any(a.confirmed and a.platform == platform and a.alert_id > previous_id
                            for a in state["alerts"])):
                return
        raise AssertionError(f"No confirmed alert for {platform}; mission={state['mission']}; alerts={state['alerts']}; readings={state['readings']}; health={state['health']}")

    def command(text):
        request = OperatorCommand.Request()
        request.command = text
        began = time.monotonic()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        assert future.done() and future.exception() is None, "Command failed: " + text
        print(f"Command {text}: {future.result().message} ({time.monotonic()-began:.3f}s)", flush=True)
        return future.result().accepted

    def start_when_ready(speed=2.5):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            pump(.4, speed=speed)
            if command("START"):
                return
        raise AssertionError(f"No mission readiness: readings={state['readings']}; health={state['health']}")

    with tempfile.TemporaryDirectory(prefix="embertrace_dual_") as directory:
        processes, handles = [], []

        def start(executable, name, parameters):
            handle = (Path(directory) / (name + ".log")).open("w")
            handles.append(handle)
            args = [
                "ros2",
                "run",
                "embertrace_core",
                executable,
                "--ros-args",
                "-r",
                "__node:=" + name,
            ]
            for key, value in parameters.items():
                args.extend(["-p", key + ":=" + str(value)])
            processes.append(
                subprocess.Popen(
                    args,
                    stdout=handle,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            )

        try:
            for root, robot in [("/embertrace", "husky1"), ("/drone1", "drone1")]:
                start(
                    "perception_node",
                    robot + "_perception_test",
                    dict(
                        input_topic=root + "/raw_environment",
                        output_topic=root + "/environment",
                        visual_topic=root + "/visual_cue",
                        thermal_topic=root + "/thermal_cue",
                        status_topic=root + "/perception_status",
                        expected_camera_frame=robot + "_camera_link",
                        require_co2="true",
                        require_thermal="true",
                        # This fixture serialises both platforms' fake image streams.
                        # Allow host scheduling stalls; expiry is checked separately.
                        environment_timeout_s=5.0,
                        visual_timeout_s=5.0,
                        thermal_timeout_s=5.0,
                    ),
                )
                start(
                    "camera_verifier_node",
                    robot + "_rgb_test",
                    dict(
                        image_topic="/" + robot + "/camera/image",
                        output_topic=root + "/visual_cue",
                        legacy_topic=root + "/legacy_visual",
                        image_timeout_s=5.0,
                    ),
                )
                start(
                    "thermal_verifier_node",
                    robot + "_ir_test",
                    dict(
                        image_topic="/" + robot + "/thermal/image",
                        output_topic=root + "/thermal_cue",
                        image_timeout_s=5.0,
                    ),
                )
            start(
                "mission_manager_node",
                "mission_dual_test",
                dict(wind_dwell_s=0.4, drone_enabled="true", reading_timeout_s=5.0, wind_timeout_s=5.0),
            )
            assert client.wait_for_service(
                timeout_sec=15
            ), "Mission service unavailable"
            start_when_ready()
            pump(1)
            assert state["mission"].active_platform == MissionState.PLATFORM_DRONE
            pump(4, fires=(True, False))
            assert not state[
                "alerts"
            ], "Unselected rover evidence confirmed a drone fire"
            confirmed(MissionState.PLATFORM_DRONE, 2.5, (False, True))
            assert command("E_STOP") and not command("START")
            assert command("RESET")
            start_when_ready(speed=7)
            pump(1, speed=7)
            assert state["mission"].active_platform == MissionState.PLATFORM_ROVER
            confirmed(MissionState.PLATFORM_ROVER, 7, (True, False))
            assert command("RESET")
            pump(2, speed=20)
            assert not command("START"), "Unsafe wind permitted deployment"
            assert all(
                process.poll() is None for process in processes
            ), "A tested ROS node exited"
            print("Dual-platform wind selection and local confirmation passed")
        finally:
            for process in processes:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
            for process in processes:
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            for handle in handles:
                handle.close()
            for log in Path(directory).glob("*.log"):
                print(log.name + "\n" + log.read_text()[-1500:])
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
