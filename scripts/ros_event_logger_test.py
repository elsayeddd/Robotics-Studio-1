#!/usr/bin/env python3
"""Check real event/alert transport, CSV persistence, row limits and logger shutdown."""
import csv
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_prefix
from embertrace_interfaces.msg import FireAlert
from std_msgs.msg import String


def main():
    rclpy.init()
    node = Node('event_logger_integration')
    events = node.create_publisher(String, '/embertrace/mission_event', 50)
    alerts = node.create_publisher(FireAlert, '/embertrace/fire_alert',
                                  QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    executable = Path(get_package_prefix('embertrace_core'))/'lib/embertrace_core/mission_event_logger.py'
    with tempfile.TemporaryDirectory(prefix='embertrace_event_logger_') as directory:
        output = Path(directory)
        with (output/'console.log').open('w') as console:
            process = subprocess.Popen([str(executable), '--ros-args', '-p', 'directory:='+str(output),
                '-p', 'maximum_rows:=2'], stdout=console, stderr=subprocess.STDOUT, start_new_session=True)
            def until(predicate, timeout=8):
                deadline = time.monotonic()+timeout
                while time.monotonic()<deadline:
                    assert process.poll() is None, (output/'console.log').read_text()
                    rclpy.spin_once(node, timeout_sec=.02)
                    if predicate(): return
                raise AssertionError('Logger deadline: '+(output/'console.log').read_text())
            def rows():
                files = list(output.glob('*.csv'))
                return list(csv.DictReader(files[0].open())) if files else []
            try:
                until(lambda: events.get_subscription_count()>0 and alerts.get_subscription_count()>0)
                events.publish(String(data='7 | PATROL | rover | selected'))
                until(lambda: len(rows())==1)
                alerts.publish(FireAlert(alert_id=42, confirmed=True, evidence='Confirmed local gas and heat'))
                until(lambda: len(rows())==2)
                events.publish(String(data='This event exceeds the configured row limit'))
                end = time.monotonic()+.3
                while time.monotonic()<end: rclpy.spin_once(node, timeout_sec=.02)
                result = rows()
                assert len(result)==2 and [r['kind'] for r in result]==['mission','alert'], result
                assert result[0]['detail']=='7 | PATROL | rover | selected'
                assert result[1]['detail']=='42: Confirmed local gas and heat'
                assert all(r['received_utc'] for r in result)
                os.killpg(process.pid, signal.SIGINT)
                assert process.wait(timeout=5)==0, (output/'console.log').read_text()
                assert rows()==result, 'Shutdown lost persisted events'
                print('Event logger passed: mission/alert transport, persisted CSV, bounded rows and clean shutdown')
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
                node.destroy_node()
                if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
