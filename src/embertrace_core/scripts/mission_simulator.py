#!/usr/bin/env python3
"""Publish configurable wind and per-vehicle battery inputs in simulation time."""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import BatteryState
from embertrace_interfaces.msg import MissionState


class MissionSimulator(Node):
    def __init__(self):
        super().__init__("mission_simulator")
        defaults = dict(
            wind_speed_mps=2.5,
            wind_direction_rad=-2.35619449,
            initial_battery_fraction=1.0,
            active_drain_per_second=0.0006,
            idle_drain_per_second=0.00002,
        )
        self.values = {
            name: self.declare_parameter(name, value).value
            for name, value in defaults.items()
        }
        self.validate(self.values)
        self.add_on_set_parameters_callback(self.parameters_changed)
        self.wind = self.create_publisher(
            Vector3Stamped, "/embertrace/wind", qos_profile_sensor_data
        )
        self.batteries = [
            self.create_publisher(BatteryState, "/" + robot + "/battery", 10)
            for robot in ("husky1", "drone1")
        ]
        self.fractions = [self.values["initial_battery_fraction"]] * 2
        self.active = 0
        self.moving = False
        self.last_time = None
        self.create_subscription(
            MissionState,
            "/embertrace/mission_state",
            self.state,
            rclpy.qos.QoSProfile(
                depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )
        self.create_timer(0.2, self.publish)

    @staticmethod
    def validate(values):
        if not all(math.isfinite(float(v)) for v in values.values()):
            raise ValueError("Simulation parameters must be finite")
        if (
            not 0 <= values["wind_speed_mps"] <= 100
            or not 0 <= values["initial_battery_fraction"] <= 1
        ):
            raise ValueError("Wind speed or initial battery outside range")
        if values["active_drain_per_second"] < 0 or values["idle_drain_per_second"] < 0:
            raise ValueError("Battery drain must be non-negative")

    def parameters_changed(self, parameters):
        proposed = dict(self.values)
        for parameter in parameters:
            if parameter.name in proposed:
                proposed[parameter.name] = parameter.value
        try:
            self.validate(proposed)
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        if (
            proposed["initial_battery_fraction"]
            != self.values["initial_battery_fraction"]
        ):
            self.fractions = [proposed["initial_battery_fraction"]] * 2
        self.values = proposed
        return SetParametersResult(successful=True)

    def state(self, message):
        self.active = message.active_platform
        self.moving = message.motion_allowed or message.manual_control

    def publish(self):
        stamp = self.get_clock().now()
        seconds = stamp.nanoseconds / 1e9
        if self.last_time is not None and seconds <= self.last_time:
            if seconds < self.last_time:
                self.last_time = None
                self.fractions = [self.values["initial_battery_fraction"]] * 2
            return
        elapsed = 0 if self.last_time is None else seconds - self.last_time
        self.last_time = seconds
        message = Vector3Stamped()
        message.header.stamp = stamp.to_msg()
        message.header.frame_id = "husky1_odom"
        speed, direction = (
            self.values["wind_speed_mps"],
            self.values["wind_direction_rad"],
        )
        message.vector.x = speed * math.cos(direction)
        message.vector.y = speed * math.sin(direction)
        self.wind.publish(message)
        for index, publisher in enumerate(self.batteries):
            active = self.moving and self.active == index + 1
            drain = self.values[
                "active_drain_per_second" if active else "idle_drain_per_second"
            ]
            self.fractions[index] = max(0.0, self.fractions[index] - drain * elapsed)
            battery = BatteryState()
            battery.header = message.header
            battery.percentage = self.fractions[index]
            battery.present = True
            battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            publisher.publish(battery)


def main():
    rclpy.init()
    node = MissionSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
