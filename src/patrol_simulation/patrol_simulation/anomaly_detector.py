"""Learn the observed patrol route, detect LiDAR changes and save camera evidence."""
import json
import math
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String

from patrol_simulation.patrol_logic import pose_xyyaw, scan_saturated, scan_usable, stamp_seconds
from patrol_simulation.route_detection import RouteComparator, IncidentRegistry
from patrol_simulation.incident_evidence import camera_projection, save_camera_frame


class AnomalyDetector(Node):
    def __init__(self):
        super().__init__('anomaly_detector')
        durable = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.events = self.create_publisher(String, 'patrol/events', durable)
        self.reference = self.create_publisher(String, 'patrol/reference_observation', durable)
        self.create_subscription(Odometry, 'odom', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Image, 'camera/image', self.on_image, qos_profile_sensor_data)
        self.create_subscription(String, 'patrol/status', self.on_status, durable)
        config = Path(get_package_share_directory('patrol_simulation')) / 'reference/monitored_zones.yaml'
        contract = yaml.safe_load(config.read_text())
        self.zones = contract['monitored_zones']
        settings = contract['detection']
        self.comparator = RouteComparator(settings['difference_threshold_m'], settings['persistence_scans'])
        self.registry = IncidentRegistry(settings['incident_separation_m'])
        self.images = deque(maxlen=10)
        self.evidence_dir = Path(self.declare_parameter('evidence_dir', 'incidents').value).expanduser().resolve()
        self.evidence_dir /= datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'_'+uuid4().hex[:6]
        self.poses = deque(maxlen=100)
        self.round_number = 0
        self.camera_time = -math.inf
        self.camera_frames = 0
        self.last_scan_time = -math.inf
        self.checkpoint_samples = 0
        self.reference_ready = False
        self.lidar_fault = False

    def seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, msg):
        body = json.loads(msg.data)
        if body.get('event') == 'round_started':
            self.round_number = int(body['round_id'].split('-')[-1])
            self.comparator._miss()
            self.get_logger().info(f'Round {self.round_number}: ' +
                                   ('LEARNING clear-site LiDAR reference' if self.round_number == 1
                                    else 'comparing live LiDAR against the first lap'))
        elif body.get('event') == 'round_completed' and body.get('round_id') == 'SIM-0001':
            if self.reference_ready:
                self.publish_reference('reference_frozen')
            else:
                self.get_logger().error('REFERENCE MISSING: pallet insertion remains disabled. Check /scan and /odom.')

    def on_pose(self, msg):
        self.poses.append((stamp_seconds(msg.header.stamp), pose_xyyaw(msg.pose.pose)))

    def on_image(self, msg):
        self.camera_time = stamp_seconds(msg.header.stamp)
        self.camera_frames += 1
        self.images.append((self.camera_time, msg))

    def publish_reference(self, event):
        body = {'event': event, 'round_id': 'SIM-0001', 'coverage': 'observed_indoor_and_outdoor_route',
                'reference_scans': len(self.comparator.samples),
                'observed_free_cells': len(self.comparator.free),
                'checkpoint_scans': self.checkpoint_samples,
                'camera_frames_received': self.camera_frames,
                'simulation_time_s': round(self.seconds(), 2)}
        self.reference.publish(String(data=json.dumps(body)))
        self.get_logger().info(event.upper() + ' ' + json.dumps(body))

    def on_scan(self, scan):
        stamp = stamp_seconds(scan.header.stamp)
        if scan_saturated(scan):
            if not self.lidar_fault:
                self.get_logger().error(
                    'LIDAR RENDERING ERROR: every beam is stuck at the minimum range. '
                    'Reference learning and anomaly detection are suspended. '
                    'Restart with software_rendering:=true (the default).')
            self.lidar_fault = True
            self.comparator._miss()
            return
        if self.lidar_fault and scan_usable(scan):
            self.get_logger().info('LIDAR RECOVERED: usable scene scans received.')
            self.lidar_fault = False
        if not scan_usable(scan) or stamp <= self.last_scan_time or not self.poses or self.seconds()-stamp > 0.5:
            return
        self.last_scan_time = stamp
        pose_stamp, pose = min(self.poses, key=lambda p: abs(p[0]-stamp))
        if abs(pose_stamp-stamp) > 0.12:
            return
        if self.round_number == 1:
            self.comparator.learn(pose, scan)
            if -5.0 < pose[0] < -2.0 and abs(pose[1]+5.0) < 0.2:
                self.checkpoint_samples += 1
                if self.checkpoint_samples >= 5 and not self.reference_ready:
                    self.reference_ready = True
                    self.publish_reference('reference_learned')
        elif self.round_number > 1 and self.reference_ready:
            for result in self.comparator.compare_all(pose, scan):
                if self.registry.register(result['object_x_m'], result['object_y_m']):
                    self.emit_alarm(pose, result, stamp)

    def emit_alarm(self, pose, result, scan_time):
        location = (result['object_x_m'], result['object_y_m'])
        zone = 'outdoor_route'
        for region in self.zones:
            xmin, xmax, ymin, ymax = region['bounds_xy_m']
            if xmin <= location[0] <= xmax and ymin <= location[1] <= ymax:
                zone = region['id']
                break
        event = {
            'schema_version': '1.3', 'event_id': str(uuid4()),
            'event_type': 'obstacle.unexpected_object', 'severity': 'medium', 'status': 'new',
            'round_id': f'SIM-{self.round_number:04d}',
            'detected_at_utc': datetime.now(timezone.utc).isoformat(),
            'simulation_time_s': round(scan_time, 2),
            'location': {'zone': zone, 'map_x_m': result.pop('object_x_m'),
                         'map_y_m': result.pop('object_y_m')},
            'robot_pose': {'map_x_m': round(pose[0], 3), 'map_y_m': round(pose[1], 3),
                           'yaw_rad': round(pose[2], 3), 'source': 'gazebo_world_odometry'},
            'assessment': {'reference_state': 'clear volume observed on round 1',
                           'observed_state': 'persistent new LiDAR surface',
                           'sensor_sources': ['lidar'], **result},
            'evidence': {'camera_stream_available': 0 <= scan_time-self.camera_time < 1.0,
                         'camera_topic': '/camera/image', 'media_saved': False,
                         'privacy_processing_implemented': False},
            'recommended_action': 'review_image_and_object_location'}
        evidence = event['evidence']
        evidence.update(camera_image_path=None, camera_capture_status='no_synchronized_frame',
                        object_projected_in_camera=False, visual_confirmation_implemented=False)
        closest = min(self.images, key=lambda item: abs(item[0]-scan_time), default=None)
        try:
            self.evidence_dir.mkdir(parents=True, exist_ok=True)
            if closest and abs(closest[0]-scan_time) <= 0.5:
                camera_time, frame = closest
                pose_sample = min(self.poses, key=lambda item: abs(item[0]-camera_time))
                pixel = (camera_projection(pose_sample[1], location, frame.width, frame.height)
                         if abs(pose_sample[0]-camera_time) <= 0.12 else None)
                image_path = self.evidence_dir / (event['event_id']+'.png')
                save_camera_frame(frame, image_path)
                evidence.update(media_saved=True, camera_image_path=str(image_path),
                                camera_simulation_time_s=round(camera_time, 3),
                                camera_robot_pose_xyyaw=list(pose_sample[1]),
                                camera_time_offset_s=round(camera_time-scan_time, 3),
                                object_projected_in_camera=pixel is not None,
                                projected_pixel_xy=pixel,
                                camera_capture_status='projected_object_view' if pixel else 'context_only')
            record = self.evidence_dir / (event['event_id']+'.json')
            evidence['incident_record_path'] = str(record)
            record.write_text(json.dumps(event, indent=2)+'\n')
        except (OSError, ValueError) as exc:
            evidence['save_error'] = str(exc)
            self.get_logger().error(f'Incident evidence could not be saved: {exc}')
        self.events.publish(String(data=json.dumps(event, separators=(',', ':'))))
        self.get_logger().warn('ANOMALY DETECTED: unexpected object at '
                               f"({event['location']['map_x_m']}, {event['location']['map_y_m']}); "
                               f"LiDAR difference {result['difference_m']:.2f} m on "
                               f"{result['consecutive_scans']} consecutive scans.")
        self.get_logger().warn(json.dumps(event))
        if evidence['media_saved']:
            self.get_logger().info(f"CAMERA IMAGE SAVED ({evidence['camera_capture_status']}): "
                                   + evidence['camera_image_path'])


def main():
    rclpy.init()
    node = AnomalyDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
