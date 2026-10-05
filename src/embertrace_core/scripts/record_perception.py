#!/usr/bin/env python3
"""Record acquisition timestamps, optional vision and evaluation-only labels to CSV."""
import argparse
import csv
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from embertrace_interfaces.msg import EnvironmentalReading, TrialTruth, VisualCue

HEADER = ['trial', 'scenario', 'now_s', 'stamp_s', 'smoke_ppm', 'temperature_c',
          'wind_speed_mps', 'wind_direction_rad', 'visual_stamp_s', 'visual_confidence',
          'visual_valid', 'environment_available', 'fire_present']


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Recorder(Node):
    def __init__(self, args):
        super().__init__('record_perception')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        root = '/embertrace' if args.robot == 'husky1' else '/drone1'
        path = Path(args.output).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = path.open('w', newline='')
        self.writer = csv.writer(self.handle)
        self.writer.writerow(HEADER)
        self.trial = args.trial
        self.raw, self.truth = {}, {}
        self.visual = None
        self.rows = 0
        self.subscriptions_owned = [
            self.create_subscription(EnvironmentalReading, root + '/raw_environment', self.on_raw, qos_profile_sensor_data),
            self.create_subscription(VisualCue, root + '/visual_cue', self.on_visual, qos_profile_sensor_data),
            self.create_subscription(TrialTruth, root + '/validation/truth', self.on_truth, 10),
        ]
        self.create_timer(0.1, self.drain)

    def on_raw(self, msg):
        self.raw[round(stamp(msg), 9)] = (msg, self.get_clock().now().nanoseconds * 1e-9)
        self.drain()

    def on_visual(self, msg):
        self.visual = msg

    def on_truth(self, msg):
        self.truth[round(stamp(msg), 9)] = (msg, time.monotonic())
        self.drain()

    def drain(self, force=False):
        for key, (truth, received) in sorted(list(self.truth.items())):
            raw = self.raw.get(key)
            if raw is None and not force and time.monotonic() - received < 0.4:
                continue
            now_s = self.get_clock().now().nanoseconds * 1e-9
            v = self.visual
            # A later camera frame cannot be replayed backwards into this sample.
            visual_valid = v is not None and v.valid and 0 <= now_s - stamp(v) <= 1.0
            values = [key, 0, 22, 0, 0]
            if raw is not None:
                msg, now_s = raw
                values = [stamp(msg), msg.smoke_ppm, msg.temperature_c, msg.wind_speed_mps, msg.wind_direction_rad]
                visual_valid = v is not None and v.valid and 0 <= now_s - stamp(v) <= 1.0
            self.writer.writerow([self.trial, truth.scenario, now_s, *values,
                                  stamp(v) if visual_valid else 0,
                                  v.confidence if visual_valid else 0,
                                  int(visual_valid), int(raw is not None), int(truth.fire_present)])
            self.rows += 1
            self.raw.pop(key, None)
            self.truth.pop(key, None)
        # Bound pending packets even when a label stream disappears.
        if len(self.raw) > 200:
            for key in sorted(self.raw)[:-200]:
                self.raw.pop(key)
        self.handle.flush()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', choices=['husky1', 'drone1'], default='husky1')
    parser.add_argument('--output', required=True)
    parser.add_argument('--trial', default='gazebo_01')
    parser.add_argument('--duration', type=float, default=120)
    args, ros_args = parser.parse_known_args()
    if args.duration <= 0:
        parser.error('--duration must be positive')
    rclpy.init(args=ros_args)
    node = Recorder(args)
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.drain(force=True)
        node.handle.close()
        count = node.rows
        node.destroy_node()
        rclpy.shutdown()
    if count == 0:
        raise SystemExit('No labelled samples recorded; check the simulator and truth topic')
    print(f'Recorded {count} labelled samples to {args.output}')


if __name__ == '__main__':
    main()
