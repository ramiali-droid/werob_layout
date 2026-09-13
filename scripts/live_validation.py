#!/usr/bin/env python3
"""Run an isolated live acceptance check and retain measured evidence."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from uuid import uuid4

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String
from patrol_simulation.patrol_logic import scan_points, pose_xyyaw


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['beside', 'blocking', 'outdoor', 'mixed', 'none', 'physics', 'preview'], default='beside')
    parser.add_argument('--suite', action='store_true')
    parser.add_argument('--gui', action='store_true', help='Test the actual Gazebo GUI launch as well as its server.')
    parser.add_argument('--label', help='Evidence filename prefix for a diagnostic run.')
    parser.add_argument('--timeout', type=float, default=420)
    args = parser.parse_args()
    if args.suite:
        for mode in ('beside', 'none', 'blocking', 'physics'):
            subprocess.run([sys.executable, __file__, '--mode', mode, '--timeout', str(args.timeout)]
                           + (['--gui'] if args.gui else []), check=True)
        return
    # Gazebo's Ruby launcher can leave its server child alive after termination.
    # Use a separate transport partition for every case, even in the same suite.
    os.environ['GZ_PARTITION'] = 'werob_validation_' + uuid4().hex
    evidence = Path('validation')
    evidence.mkdir(exist_ok=True)
    diagnostics = Path('archive/validation')
    diagnostics.mkdir(parents=True, exist_ok=True)
    label = args.label or (('gui_' if args.gui else '') + args.mode)
    rclpy.init()
    node = Node('patrol_acceptance_check')
    durable = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state = {'pose': None, 'scan': None, 'images': 0, 'rounds': [], 'alarms': [], 'stops': [],
             'reference': False, 'spawned': False, 'stop_time': None, 'max_x': -math.inf,
             'physical_contacts': 0}
    output = (evidence / f'{label}_events.jsonl').open('w')
    trajectory = (evidence / f'{label}_trajectory.csv').open('w')
    scans = (diagnostics / f'{label}_scans.jsonl').open('w')
    trajectory.write('time_s,x_m,y_m,yaw_rad\n')
    last_trajectory = -1
    last_scan_record = -1

    def odom(msg):
        nonlocal last_trajectory
        state['pose'] = pose_xyyaw(msg.pose.pose)
        state['max_x'] = max(state['max_x'], state['pose'][0])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp-last_trajectory > 0.2:
            trajectory.write(f'{stamp:.3f},' + ','.join(f'{v:.5f}' for v in state['pose']) + '\n')
            last_trajectory = stamp

    def scan(msg):
        nonlocal last_scan_record
        state['scan'] = msg
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = state['pose']
        if pose and stamp-last_scan_record > 0.5:
            scans.write(json.dumps({'time_s': stamp, 'pose': pose, 'ranges': list(msg.ranges),
                                    'angle_min': msg.angle_min, 'angle_increment': msg.angle_increment,
                                    'range_min': msg.range_min, 'range_max': msg.range_max}) + '\n')
            scans.flush()
            last_scan_record = stamp

    def camera(msg):
        state['images'] += 1

    def contact(msg):
        state['physical_contacts'] += len(msg.contacts)

    def event(msg):
        body = json.loads(msg.data)
        output.write(json.dumps(body) + '\n')
        output.flush()
        name = body.get('event', body.get('event_type'))
        print(name, json.dumps(body), flush=True)
        if name == 'round_completed':
            state['rounds'].append(body)
        elif name == 'obstacle.unexpected_object':
            state['alarms'].append(body)
        elif name in ('collision_risk', 'contact_detected'):
            state['stops'].append(body)
            state['stop_time'] = time.monotonic()
        elif name == 'reference_frozen':
            state['reference'] = True
        elif name == 'obstacle_spawned':
            state['spawned'] = True

    node.create_subscription(Odometry, 'odom', odom, qos_profile_sensor_data)
    node.create_subscription(LaserScan, 'scan', scan, qos_profile_sensor_data)
    node.create_subscription(Image, 'camera/image', camera, qos_profile_sensor_data)
    node.create_subscription(Contacts, 'patrol/contacts', contact, qos_profile_sensor_data)
    drive = node.create_publisher(Twist, 'cmd_vel', 10)
    for topic in ('patrol/status', 'patrol/reference_observation', 'patrol/scenario', 'patrol/events'):
        node.create_subscription(String, topic, event, durable)
    log = (diagnostics / f'{label}_launch.log').open('w')
    launch_args = ['ros2', 'launch', 'patrol_simulation', 'bringup.launch.py',
                   f'gui:={str(args.gui).lower()}', 'render_engine:=ogre',
                   f'evidence_dir:={evidence.resolve() / (label+"_incidents")}',
                   f"obstacle_mode:={'none' if args.mode in ('physics', 'preview') else args.mode}"]
    if args.mode in ('physics', 'preview'):
        launch_args.append('controller:=false')
    process = subprocess.Popen(launch_args,
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    start, last_progress, success = time.monotonic(), 0, False
    blocked_at = None
    try:
        while time.monotonic()-start < args.timeout and process.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            if args.mode == 'preview' and state['images'] >= 10 and state['scan'] and time.monotonic()-start > 10:
                success = bool(list(scan_points(state['scan'])))
                if args.gui:
                    screenshot = evidence / (label+'_model.png')
                    captured = subprocess.run([sys.executable, 'scripts/capture_gazebo.py',
                                               '--focus-model', 'husky_patrol', '--output', str(screenshot)],
                                              capture_output=True, text=True, timeout=15)
                    success &= captured.returncode == 0 and screenshot.is_file()
                    if captured.returncode:
                        print(captured.stderr, flush=True)
                break
            if args.mode == 'physics':
                command = Twist()
                # Controlled impact at low speed; cut motor power after actual
                # contact so this test does not try to climb the fence.
                command.linear.x = 0.0 if state['physical_contacts'] else (
                    0.15 if state['pose'] and state['pose'][0] > 12.5 else 0.75)
                drive.publish(command)
                if state['physical_contacts']:
                    if blocked_at is None:
                        blocked_at = (time.monotonic(), state['pose'])
                    if time.monotonic()-blocked_at[0] > 2:
                        success = (13.35 < state['max_x'] < 13.60
                                   and math.dist(state['pose'][:2], blocked_at[1][:2]) < 0.1)
                        break
            if time.monotonic()-last_progress > 10:
                points = sorted(scan_points(state['scan']), key=lambda p: math.hypot(p[1],p[2]))[:4] if state['scan'] else []
                print('PROGRESS', {'wall_s': round(time.monotonic()-start), 'pose': state['pose'],
                                   'images': state['images'], 'nearest_scan_points': points}, flush=True)
                last_progress = time.monotonic()
            if (args.mode == 'blocking' and state['alarms'] and state['stops']
                    and time.monotonic()-state['stop_time'] > 2):
                if blocked_at is None:
                    blocked_at = (time.monotonic(), state['pose'])
                if time.monotonic()-blocked_at[0] > 4:
                    success = math.dist(state['pose'][:2], blocked_at[1][:2]) < 0.05 and len(state['rounds']) == 1
                    break
            elif args.mode in ('beside', 'outdoor', 'mixed', 'none') and len(state['rounds']) >= 2:
                success = state['reference'] and state['images'] > 0 and not state['stops']
                expected = {'none': [], 'beside': [(-3.5,-5.7)], 'outdoor': [(8,-0.85)],
                            'mixed': [(8,-0.85), (-3.5,-5.7)]}[args.mode]
                observed = [(e['location']['map_x_m'], e['location']['map_y_m']) for e in state['alarms']]
                success &= len(observed) == len(expected)
                success &= all(any(math.dist(p, q) < 0.7 for q in observed) for p in expected)
                success &= all(e['evidence']['media_saved'] and Path(e['evidence']['camera_image_path']).is_file()
                               for e in state['alarms'])
                break
    finally:
        if process.poll() is None:
            # Let ROS launch forward one interrupt; signalling the whole group
            # as well would interrupt nodes again during their cleanup.
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
        # This group was created by this validator, never by a user's launch.
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        output.close()
        trajectory.close()
        scans.close()
        log.close()
    result = {'mode': args.mode, 'passed': bool(success), 'completed_rounds': len(state['rounds']),
              'anomalies': len(state['alarms']), 'collision_stops': len(state['stops']), 'camera_frames': state['images']}
    if args.mode == 'physics':
        result.update(controller_enabled=False, impact_command_m_s=0.15, physical_contacts=state['physical_contacts'], max_robot_x_m=state['max_x'],
                      fence_near_face_x_m=13.925)
    result['gui'] = args.gui
    (evidence / f'{label}_result.json').write_text(json.dumps(result, indent=2) + '\n')
    print('RESULT', json.dumps(result), flush=True)
    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
