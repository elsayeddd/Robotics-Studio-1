#!/usr/bin/env python3
"""Append mission transitions and alerts to a timestamped, bounded CSV event log."""

import csv
from datetime import datetime, timezone
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from embertrace_interfaces.msg import FireAlert
from rclpy.qos import QoSProfile, DurabilityPolicy


class EventLogger(Node):
    def __init__(self):
        super().__init__("mission_event_logger")
        directory = Path(
            self.declare_parameter("directory", "log/mission-events").value
        ).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        self.limit = int(self.declare_parameter("maximum_rows", 100000).value)
        if self.limit < 1:
            raise ValueError("maximum_rows must be positive")
        name = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ") + ".csv"
        self.csv_handle = (directory / name).open("x", newline="")
        self.writer = csv.writer(self.csv_handle)
        self.writer.writerow(["received_utc", "kind", "detail"])
        self.csv_handle.flush()
        self.rows = 0
        self.get_logger().info(f"Mission event log: {directory / name}")
        self.create_subscription(
            String,
            "/embertrace/mission_event",
            lambda m: self.write("mission", m.data),
            50,
        )
        self.create_subscription(
            FireAlert,
            "/embertrace/fire_alert",
            lambda m: self.write("alert", f"{m.alert_id}: {m.evidence}"),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

    def write(self, kind, detail):
        if self.rows >= self.limit:
            return
        self.writer.writerow([datetime.now(timezone.utc).isoformat(), kind, detail])
        self.csv_handle.flush()
        self.rows += 1


def main():
    rclpy.init()
    node = EventLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_handle.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
