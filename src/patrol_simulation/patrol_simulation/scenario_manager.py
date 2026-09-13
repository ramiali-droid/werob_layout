"""Insert the test object only after a completed, successfully learned baseline lap."""
import json
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from gz.transport13 import Node as GzNode
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.boolean_pb2 import Boolean


class ScenarioManager(Node):
    def __init__(self):
        super().__init__('scenario_manager')
        durable = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, 'patrol/scenario', durable)
        self.create_subscription(String, 'patrol/status', self.on_status, durable)
        self.create_subscription(String, 'patrol/reference_observation', self.on_reference, durable)
        self.mode = self.declare_parameter('obstacle_mode', 'beside').value
        placements = {'beside': [(-3.5, -5.7)], 'blocking': [(-3.5, -5.0)],
                      'outdoor': [(8.0, -0.85)],
                      'mixed': [(8.0, -0.85), (-3.5, -5.7)], 'none': []}
        if self.mode not in placements:
            raise ValueError('Unknown obstacle_mode')
        self.placements = placements[self.mode]
        self.spawned = set()
        self.reference_ready = self.lap_complete = self.done = False
        self.gz = GzNode()
        self.pallet = str(Path(get_package_share_directory('patrol_simulation')) / 'models/unexpected_pallet/model.sdf')
        self.create_timer(0.5, self.step)

    def on_status(self, msg):
        body = json.loads(msg.data)
        if body.get('event') == 'round_completed' and body.get('round_id') == 'SIM-0001':
            self.lap_complete = True

    def on_reference(self, msg):
        self.reference_ready |= json.loads(msg.data).get('event') == 'reference_frozen'

    def step(self):
        if self.done or not (self.lap_complete and self.reference_ready):
            return
        body = {'event': 'baseline_only', 'after_round': 'SIM-0001'}
        for index, (x, y) in enumerate(self.placements):
            if index in self.spawned:
                continue
            request = EntityFactory()
            request.sdf_filename = self.pallet
            request.name = 'unexpected_pallet' if len(self.placements) == 1 else f'unexpected_pallet_{index+1}'
            request.allow_renaming = False
            request.pose.position.x = x
            request.pose.position.y = y
            request.pose.position.z = 0.35
            request.pose.orientation.w = 1.0
            try:
                ok, response = self.gz.request('/world/patrol_pilot/create', request, EntityFactory, Boolean, 1000)
            except Exception as exc:
                self.get_logger().error(f'Pallet insertion failed: {exc}')
                return
            if not ok or not response.data:
                self.get_logger().warn('Waiting for Gazebo to insert the pallet; round 2 is held at the dock.')
                return
            self.spawned.add(index)
        if self.placements:
            body.update(event='obstacle_spawned', mode=self.mode,
                        map_pose={'x': self.placements[0][0], 'y': self.placements[0][1], 'z': 0.35},
                        objects=[{'x': x, 'y': y, 'z': 0.35} for x, y in self.placements])
        self.done = True
        self.publisher.publish(String(data=json.dumps(body)))
        self.get_logger().info('SCENARIO READY ' + json.dumps(body))


def main():
    rclpy.init()
    node = ScenarioManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
