"""Follow the accepted route using wheel velocities and measured world pose."""
import json
import math
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from ros_gz_interfaces.msg import Contacts

from patrol_simulation.patrol_logic import WAYPOINTS, collision_risk, pose_xyyaw, scan_saturated, scan_usable, stamp_seconds, wrap


class PatrolController(Node):
    def __init__(self):
        super().__init__('patrol_controller')
        durable = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, 'patrol/status', durable)
        self.pose_pub = self.create_publisher(PoseStamped, 'patrol/robot_pose', 20)
        self.velocity = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Contacts, 'patrol/contacts', self.on_contact, qos_profile_sensor_data)
        self.create_subscription(String, 'patrol/scenario', self.on_scenario, durable)
        self.max_speed = float(self.declare_parameter('max_speed', 0.75).value)
        self.pose = self.scan = None
        self.pose_time = self.scan_time = -math.inf
        self.contact_time = -math.inf
        self.lidar_fault = False
        self.index, self.round_number = 0, 1
        self.started = False
        self.round_start = None
        self.scenario_ready = False
        self.hold_reason = None
        self.turning = True
        self.turn_start_yaw = None
        self.create_timer(0.05, self.step)

    def seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        self.pose = pose_xyyaw(msg.pose.pose)
        self.pose_time = stamp_seconds(msg.header.stamp)
        pose = PoseStamped(header=msg.header, pose=msg.pose.pose)
        pose.header.frame_id = 'map'
        self.pose_pub.publish(pose)

    def on_scan(self, msg):
        self.lidar_fault = scan_saturated(msg)
        if scan_usable(msg):
            self.scan, self.scan_time = msg, stamp_seconds(msg.header.stamp)

    def on_contact(self, msg):
        if msg.contacts:
            self.contact_time = self.seconds()

    def on_scenario(self, msg):
        body = json.loads(msg.data)
        if body.get('event') in ('obstacle_spawned', 'baseline_only'):
            self.scenario_ready = True

    def emit(self, event, **extra):
        x, y, yaw = self.pose or (0.0, 0.0, 0.0)
        body = {'event': event, 'round_id': f'SIM-{self.round_number:04d}',
                'timestamp_utc': datetime.now(timezone.utc).isoformat(),
                'simulation_time_s': round(self.seconds(), 2),
                'map_pose': {'map_x_m': round(x, 3), 'map_y_m': round(y, 3), 'yaw_rad': round(yaw, 3)},
                'pose_source': 'gazebo_world_odometry', **extra}
        self.status.publish(String(data=json.dumps(body, separators=(',', ':'))))
        self.get_logger().info(json.dumps(body))

    def command(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.velocity.publish(msg)

    def hold(self, reason, **extra):
        self.command()
        if reason != self.hold_reason:
            self.hold_reason = reason
            self.emit(reason, **extra)
            if reason in ('collision_risk', 'contact_detected'):
                self.get_logger().warn('COLLISION STOP: route held; no waypoint skipped. ' + reason)

    def step(self):
        now = self.seconds()
        if self.lidar_fault:
            self.hold('lidar_rendering_error', detail='All beams are at the minimum range. Restart with software_rendering:=true.')
            return
        if self.pose is None or self.scan is None or now-self.pose_time > 0.5 or now-self.scan_time > 0.5:
            self.hold('waiting_for_sensors')
            return
        if not self.started:
            self.started = True
            self.round_start = now
            self.emit('round_started', waypoint=WAYPOINTS[0][0])
        if self.round_number == 2 and self.index == 0 and not self.scenario_ready:
            self.hold('waiting_for_scenario')
            return
        if now-self.contact_time < 0.5:
            self.hold('contact_detected')
            return

        x, y, yaw = self.pose
        name, tx, ty = WAYPOINTS[self.index]
        dx, dy = tx-x, ty-y
        distance = math.hypot(dx, dy)
        if distance < 0.09:
            self.command()
            self.emit('waypoint_reached', waypoint=name, waypoint_index=self.index,
                      target={'x': tx, 'y': ty}, position_error_m=round(distance, 3))
            self.index += 1
            self.turning = True
            if self.index == len(WAYPOINTS):
                self.emit('round_completed', result='completed_as_planned',
                          elapsed_simulation_s=round(now-self.round_start, 2), waypoints_reached=len(WAYPOINTS))
                self.index, self.round_number = 0, self.round_number+1
                self.round_start = now
                self.emit('round_started', waypoint=WAYPOINTS[0][0])
            return

        error = wrap(math.atan2(dy, dx)-yaw)
        if abs(error) > 0.3:
            self.turning = True
            if self.turn_start_yaw is None:
                self.turn_start_yaw = yaw
        if abs(error) < 0.045:
            self.turning = False
            if self.turn_start_yaw is not None:
                self.emit('turn_completed', toward_waypoint=name,
                          measured_turn_deg=round(math.degrees(wrap(yaw-self.turn_start_yaw)), 2),
                          heading_error_deg=round(math.degrees(error), 2))
                self.turn_start_yaw = None
        angular = max(-1.0, min(1.0, 2.2*error))
        linear = 0.0 if self.turning else min(self.max_speed, 1.5*distance)
        clearance = collision_risk(self.scan, linear, turning=self.turning)
        if clearance is not None:
            self.hold('collision_risk', clearance_m=round(clearance, 3), waypoint=name)
            return
        if self.hold_reason:
            self.emit('patrol_resumed', previous_hold=self.hold_reason)
            self.hold_reason = None
        self.command(linear, angular)


def main():
    rclpy.init()
    node = PatrolController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.command()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
