#!/usr/bin/env python3
"""Record independent autonomous trials without feeding evaluation truth into decisions."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt32
from geometry_msgs.msg import PoseWithCovarianceStamped
from embertrace_interfaces.msg import FireAlert, MissionState, TrialTruth
from embertrace_interfaces.srv import OperatorCommand


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def trial(args, output, number):
    robot = ("drone1" if number % 2 == 0 else "husky1") if args.platform == "alternating" else ("drone1" if args.platform == "drone" else "husky1")
    seed = args.seed_start + number
    domain = args.domain_base + number
    name = f"{robot}_{seed}"
    config_root = Path(get_package_share_directory('embertrace_bringup'))/'config'
    config = yaml.safe_load((config_root/'embertrace.yaml').read_text())
    required_loops = config['mission_manager_node']['ros__parameters']['patrol_loops']
    if robot == 'drone1':
        flight = config['drone_scout_node']['ros__parameters']
        route = [dict(name=f'sweep_{i+1}', x=x, y=y) for i, (x, y) in
                 enumerate(zip(flight['sweep_x'], flight['sweep_y']))]
    else:
        route = yaml.safe_load((config_root/'patrol_waypoints.yaml').read_text())['waypoints']
    expected_visits = len(route)*required_loops + int(robot == 'husky1')
    visit_tolerance = 1.0 if robot == 'drone1' else 1.7
    result = dict(
        trial=name,
        source_revision=args.source_revision,
        harness_revision=args.harness_revision,
        seed=seed,
        ros_domain_id=domain,
        platform=robot,
        success=False,
        mission_completed=False,
        preflight_passed=False,
        preflight_time_s=None,
        patrol_loops=0,
        required_patrol_loops=required_loops,
        expected_patrol_visits=expected_visits,
        visited_patrol_waypoints=[],
        visit_tolerance_m=visit_tolerance,
        initial_position=None,
        final_position=None,
        return_error_m=None,
        return_reason=None,
        mission_time_s=None,
        path_length_m=0.0,
        detection_latency_s=None,
        source_resolution_latency_s=None,
        first_confirmation_stamp_s=None,
        local_exposure_stamp_s=None,
        source_error_m=None,
        source_resolved=False,
        alerts=0,
        false_alerts=0,
        early_detection=False,
        recoveries=0,
        states=[],
        latest_estimate=None,
        error=None,
    )
    rclpy.init(domain_id=domain)
    node = Node("mission_campaign_recorder")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    messages = {
        "state": None,
        "alert": None,
        "previous_pose": None,
        "onset": None,
        "start": None,
        "finished": False,
        "last_phase": None,
        "alert_ids": set(),
        "first_confirmation": None,
    }
    transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def state(message):
        previous = messages["last_phase"]
        if message.state != previous: result["states"].append(message.state_name)
        if previous == MissionState.RECOVER and message.state == MissionState.TRACK:
            result["recoveries"] += 1
        messages["last_phase"] = message.state
        messages["state"] = message
        if message.state == MissionState.RETURN_TO_BASE and previous != message.state:
            result['return_reason'] = message.detail
        if messages["start"] is not None and previous == MissionState.RETURN_TO_BASE and message.state == MissionState.IDLE:
            messages["finished"] = True

    def alert(message):
        if messages["start"] is None or not message.confirmed:
            return
        new_incident = message.alert_id not in messages["alert_ids"]
        messages["alert_ids"].add(message.alert_id)
        result["alerts"] += int(new_incident)
        result["false_alerts"] += int(new_incident and args.scenario == "no_fire")
        confirmation_stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if messages["first_confirmation"] is None:
            messages["first_confirmation"] = confirmation_stamp
            result["first_confirmation_stamp_s"] = confirmation_stamp
        result["early_detection"] = messages["onset"] is None or messages["first_confirmation"] < messages["onset"]
        if message.source_resolved:
            result["source_resolution_latency_s"] = confirmation_stamp - messages["first_confirmation"]
        messages["alert"] = message
        result["source_resolved"] = message.source_resolved
        point = message.estimated_source.pose.position
        result["reported_position"] = [point.x, point.y]
        if message.source_resolved:
            point = message.estimated_source.pose.position
            # Source coordinates are used only to score the recorded estimate.
            result["source_error_m"] = math.hypot(
                point.x - args.source_x, point.y - args.source_y
            )
        if messages["onset"] is not None:
            result["detection_latency_s"] = max(0, messages["first_confirmation"] - messages["onset"])

    def pose(message):
        p = message.pose.pose.position
        current = (p.x, p.y, p.z)
        if result['initial_position'] is None:
            result['initial_position'] = list(current)
        result['final_position'] = list(current)
        if messages["start"] is not None and messages["previous_pose"] is not None:
            result["path_length_m"] += math.dist(current, messages["previous_pose"])
        messages["previous_pose"] = current
        visited = result['visited_patrol_waypoints']
        if messages['start'] is not None and len(visited)<expected_visits:
            target = route[len(visited)%len(route)]
            at_altitude = robot != 'drone1' or p.z >= flight['altitude_m']-.8
            if at_altitude and math.hypot(p.x-target['x'], p.y-target['y'])<=visit_tolerance:
                visited.append(target['name'])

    def truth(message):
        if (
            message.fire_present
            and messages["onset"] is None
            and messages["start"] is not None
        ):
            messages["onset"] = (
                message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            )
            result["local_exposure_stamp_s"] = messages["onset"]
            if messages["first_confirmation"] is not None:
                result["early_detection"] = messages["first_confirmation"] < messages["onset"]
                result["detection_latency_s"] = max(0, messages["first_confirmation"] - messages["onset"])

    def estimate(message):
        p = message.pose.pose.position
        c = message.pose.covariance
        result["latest_estimate"] = dict(x=p.x, y=p.y, variance_x=c[0], variance_y=c[7])

    root = "/embertrace" if robot == "husky1" else "/drone1"
    subscriptions = [
        node.create_subscription(UInt32, '/'+robot+'/patrol_loops',
            lambda m: result.update(patrol_loops=m.data), 10),
        node.create_subscription(PoseWithCovarianceStamped, "/embertrace/source_estimate", estimate, 10),
        node.create_subscription(
            MissionState, "/embertrace/mission_state", state, transient
        ),
        node.create_subscription(FireAlert, "/embertrace/fire_alert", alert, transient),
        node.create_subscription(
            Odometry, "/" + robot + "/odometry", pose, qos_profile_sensor_data
        ),
        node.create_subscription(TrialTruth, root + "/validation/truth", truth, 10),
    ]
    client = node.create_client(OperatorCommand, "/embertrace/operator_command")
    environment = dict(os.environ)
    environment["ROS_DOMAIN_ID"] = str(domain)
    environment["IGN_PARTITION"] = f"embertrace_{name}_{os.getpid()}"
    environment["GZ_PARTITION"] = environment["IGN_PARTITION"]
    with (output / (name + ".log")).open("x") as log:
        command = [
            "ros2",
            "launch",
            "embertrace_bringup",
            "embertrace_demo.launch.py",
            "mission_platform:=" + ("drone" if robot == "drone1" else "rover"),
            "use_thermal:=true",
            "use_rviz:=false",
            "use_dashboard:=false",
            "gazebo_gui:=false",
            f"scenario:={args.scenario}",
            f"random_seed:={seed}",
            f"drone_random_seed:={seed+1000}",
            "wind_speed_mps:=" + ("2.5" if robot == "drone1" else "7.0"),
            "use_nav2:=" + ("false" if robot == "drone1" else "true"),
        ]
        process = subprocess.Popen(
            command,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env=environment,
        )
        startup = time.monotonic()
        preflight_log = (output/(name+'.preflight.log')).open('x')
        preflight = subprocess.Popen(['ros2', 'run', 'embertrace_core', 'demo_preflight.py',
            '--platform', 'drone' if robot == 'drone1' else 'rover', '--timeout', '120'],
            stdout=preflight_log, stderr=subprocess.STDOUT, start_new_session=True, env=environment)
        reader = (output/(name+'.log')).open()
        pending_log = ''
        deadline = time.monotonic() + args.wall_timeout
        pending = None
        last_attempt = -10
        last_report = time.monotonic()
        try:
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError("Launch exited before mission completed")
                pending_log += reader.read()
                lines = pending_log.split('\n')
                pending_log = lines.pop()
                for line in lines:
                    if 'process has died' in line:
                        raise RuntimeError('A launched subsystem exited: '+line)
                if not result['preflight_passed'] and preflight.poll() is not None:
                    if preflight.returncode != 0:
                        raise RuntimeError('Preflight failed; see '+str(output/(name+'.preflight.log')))
                    result['preflight_passed'] = True
                    result['preflight_time_s'] = time.monotonic()-startup
                rclpy.spin_once(node, timeout_sec=0.05)
                now_s = node.get_clock().now().nanoseconds / 1e9
                mission = messages["state"]
                if time.monotonic() - last_report >= 15:
                    print(json.dumps(dict(trial=name, state=mission.state_name if mission else None,
                          detail=mission.detail if mission else None, path_length_m=result["path_length_m"],
                          mission_time_s=result["mission_time_s"])), flush=True)
                    last_report = time.monotonic()
                if pending is not None and pending.done():
                    response = pending.result()
                    if response.accepted:
                        messages["start"] = now_s
                    pending = None
                expected = (
                    MissionState.PLATFORM_DRONE
                    if robot == "drone1"
                    else MissionState.PLATFORM_ROVER
                )
                if (
                    messages["start"] is None
                    and result['preflight_passed']
                    and pending is None
                    and mission is not None
                    and mission.preferred_platform == expected
                    and client.service_is_ready()
                    and time.monotonic() - last_attempt > 2
                ):
                    request = OperatorCommand.Request()
                    request.command = "START"
                    pending = client.call_async(request)
                    last_attempt = time.monotonic()
                if messages["start"] is not None:
                    result["mission_time_s"] = now_s - messages["start"]
                    if mission.state in (MissionState.PAUSED, MissionState.EMERGENCY_STOP):
                        raise RuntimeError("Autonomous mission stopped: " + mission.detail)
                    if messages["finished"]:
                        result['mission_completed'] = True
                        if result['initial_position'] is None or result['final_position'] is None:
                            raise RuntimeError('Mission ended without measured odometry')
                        result['return_error_m'] = math.dist(result['initial_position'], result['final_position'])
                        result["success"] = result["false_alerts"] == 0 and (
                            result["alerts"] > 0 and result['source_resolved']
                            if args.scenario == "fire"
                            else result["alerts"] == 0 and result['patrol_loops']>=required_loops
                                and len(result['visited_patrol_waypoints'])==expected_visits
                        )
                        result['success'] &= result['return_error_m'] < (0.8 if robot=='drone1' else 1.7)
                        if not result['success']:
                            result['error'] = ('Mission ended but source, route coverage or measured return failed acceptance')
                        break
                    if now_s - messages["start"] > args.sim_timeout:
                        raise TimeoutError("Simulation-time mission deadline exceeded")
            else:
                raise TimeoutError("Wall-time mission deadline exceeded")
        except (RuntimeError, TimeoutError) as error:
            result["error"] = str(error)
        finally:
            stop(preflight)
            preflight_log.close()
            reader.close()
            stop(process)
            node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
    (output / (name + ".json")).write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="log/autonomous-campaign")
    parser.add_argument("--domain-base", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "20")))
    parser.add_argument("--platform", choices=["alternating", "drone", "rover"], default="alternating")
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--seed-start", type=int, default=300)
    parser.add_argument("--sim-timeout", type=float, default=480)
    parser.add_argument("--wall-timeout", type=float, default=900)
    parser.add_argument("--scenario", choices=["fire", "no_fire"], default="fire")
    parser.add_argument("--source-x", type=float, default=12)
    parser.add_argument("--source-y", type=float, default=7)
    args = parser.parse_args()
    if (
        args.trials < 1
        or args.domain_base < 0
        or args.domain_base + args.trials - 1 > 200
        or not all(
            math.isfinite(value)
            for value in (
                args.sim_timeout,
                args.wall_timeout,
                args.source_x,
                args.source_y,
            )
        )
        or args.sim_timeout <= 0
        or args.wall_timeout <= 0
    ):
        parser.error("Trials/deadlines must be positive; reserve trial DDS domains within 0..200")
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    args.source_revision = subprocess.check_output(["git", "log", "-1", "--format=%H", "--", "src"], text=True).strip()
    args.harness_revision = subprocess.check_output(["git", "log", "-1", "--format=%H", "--", "scripts/run_mission_campaign.py"], text=True).strip()
    results = [trial(args, output, i) for i in range(args.trials)]
    errors = sorted(
        r["source_error_m"] for r in results if r["source_error_m"] is not None
    )
    import statistics

    summary = dict(
        scenario=args.scenario,
        trials=len(results),
        completed=sum(r['mission_completed'] for r in results),
        successful=sum(r["success"] for r in results),
        resolved_sources=len(errors),
        median_source_error_m=statistics.median(errors) if errors else None,
        results=results,
    )
    summary["platforms"] = {}
    for robot in sorted({r["platform"] for r in results}):
        cohort = [r for r in results if r["platform"] == robot]
        positions = [r["source_error_m"] for r in cohort if r["source_error_m"] is not None]
        summary["platforms"][robot] = dict(trials=len(cohort),
            successful=sum(r["success"] for r in cohort), resolved_sources=len(positions),
            median_source_error_m=statistics.median(positions) if positions else None)
    summary["passed"] = summary["successful"] == args.trials and (args.scenario == "no_fire" or (len(errors) == args.trials and summary["median_source_error_m"] < 2))
    if args.scenario == "fire":
        summary["passed"] &= all(p["median_source_error_m"] is not None and p["median_source_error_m"] < 2 for p in summary["platforms"].values())
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(
        f"Recorded {len(results)} autonomous trials; {summary['completed']} completed, "
        f"{summary['successful']} accepted. Failures are retained."
    )

    raise SystemExit(0 if summary["passed"] else 1)


if __name__ == "__main__":
    main()
