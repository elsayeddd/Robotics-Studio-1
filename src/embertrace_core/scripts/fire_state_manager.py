#!/usr/bin/env python3
"""Change one simulated tree through Fortress's bounded create/remove services."""

import json
from pathlib import Path
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from embertrace_interfaces.msg import FireState
from embertrace_interfaces.srv import SetFireState
from embertrace_tools.fire_scene import state_model, STATES


class FireStateManager(Node):
    def __init__(self):
        super().__init__("fire_state_manager")
        self.world = self.declare_parameter("world_name", "embertrace_forest").value
        self.tree_name = self.declare_parameter("tree_name", "fire_source_visual").value
        self.pose = self.declare_parameter("tree_pose", "12 7 0 0 0 0").value
        self.current = self.declare_parameter("initial_state", "burning").value
        if self.current not in STATES:
            raise ValueError("Invalid initial tree state")
        models = Path(get_package_share_directory("embertrace_sim")) / "models"
        self.tree = (models / "ember_tree/model.sdf").read_text()
        self.flame = (models / "fire_source/model.sdf").read_text()
        self.rendered = False
        self.initialised = False
        self.publisher = self.create_publisher(
            FireState,
            "/embertrace/fire_state",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_service(SetFireState, "/embertrace/set_fire_state", self.change)
        self.create_timer(2.0, self.initialise)

    def publish(self):
        message = FireState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.tree_name = self.tree_name
        message.state = STATES[self.current]
        message.rendered = self.rendered
        self.publisher.publish(message)

    def request(self, action, request_type, data):
        result = subprocess.run(
            [
                "ign",
                "service",
                "-s",
                f"/world/{self.world}/{action}",
                "--reqtype",
                request_type,
                "--reptype",
                "ignition.msgs.Boolean",
                "--timeout",
                "2500",
                "--req",
                data,
            ],
            capture_output=True,
            text=True,
            timeout=4,
        )
        return result.returncode == 0 and "data: true" in result.stdout

    def render(self, target):
        self.rendered = False
        self.publish()  # Suspend simulated sensing while geometry is changing.
        # Removing a missing model is harmless when a previous create failed.
        self.request(
            "remove",
            "ignition.msgs.Entity",
            f"name: {json.dumps(self.tree_name)} type: MODEL",
        )
        model = state_model(self.tree, self.flame, target, self.tree_name, self.pose)
        if not self.request(
            "create", "ignition.msgs.EntityFactory", "sdf: " + json.dumps(model)
        ):
            raise RuntimeError(
                "Gazebo did not confirm tree creation; check world and UserCommands plugin"
            )
        self.current, self.rendered, self.initialised = target, True, True
        self.publish()

    def initialise(self):
        if not self.initialised:
            try:
                self.render(self.current)
            except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
                self.get_logger().warning(str(error))
        self.publish()

    def change(self, request, response):
        target = request.state.strip().lower()
        if target not in STATES:
            response.message = "Use healthy, burning or burnt"
            return response
        try:
            self.render(target)
            response.accepted = True
            response.message = "Tree state: " + target
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
            self.rendered = False
            self.publish()
            response.message = str(error)
        return response


def main():
    rclpy.init()
    node = FireStateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
