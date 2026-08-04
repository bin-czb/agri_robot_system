"""Publish surveyed tree landmarks and keep-out radii in RViz."""

import csv
import math
import os

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from visualization_msgs.msg import Marker, MarkerArray


class TreeMarkerPublisher(Node):
    """Publish static tree cylinders, labels and keep-out circles."""

    def __init__(self) -> None:
        super().__init__('tree_marker_publisher')
        self.declare_parameter('csv_path', '')
        self.declare_parameter('topic', '/tree_markers')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('tree_height_m', 1.8)
        self.declare_parameter('circle_segments', 48)

        csv_path = os.path.expanduser(
            str(self.get_parameter('csv_path').value))
        if not csv_path or not os.path.isfile(csv_path):
            raise FileNotFoundError(f'Tree landmark CSV not found: {csv_path}')
        self.rows = self._load_rows(csv_path)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('topic').value),
            qos,
        )
        self.timer = self.create_timer(0.7, self._publish_once)
        self.get_logger().info(
            f'Loaded {len(self.rows)} enabled tree landmarks from {csv_path}')

    @staticmethod
    def _load_rows(path):
        enabled_rows = []
        with open(path, 'r', encoding='utf-8-sig', newline='') as stream:
            for row in csv.DictReader(stream):
                if str(row.get('enabled', '1')).strip() in ('1', 'true', 'True'):
                    enabled_rows.append(row)
        return enabled_rows

    def _publish_once(self) -> None:
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        stamp = self.get_clock().now().to_msg()
        frame_id = str(self.get_parameter('frame_id').value)
        tree_height = float(self.get_parameter('tree_height_m').value)
        segments = max(12, int(
            self.get_parameter('circle_segments').value))

        for index, row in enumerate(self.rows):
            x = float(row['tree_map_x'])
            y = float(row['tree_map_y'])
            tree_radius = float(row.get('tree_radius_m', 0.20))
            keepout_radius = float(
                row.get('keepout_radius_m', tree_radius))

            trunk = Marker()
            trunk.header.frame_id = frame_id
            trunk.header.stamp = stamp
            trunk.ns = 'tree_trunks'
            trunk.id = index
            trunk.type = Marker.CYLINDER
            trunk.action = Marker.ADD
            trunk.pose.position.x = x
            trunk.pose.position.y = y
            trunk.pose.position.z = tree_height * 0.5
            trunk.pose.orientation.w = 1.0
            trunk.scale.x = tree_radius * 2.0
            trunk.scale.y = tree_radius * 2.0
            trunk.scale.z = tree_height
            trunk.color.r = 0.16
            trunk.color.g = 0.55
            trunk.color.b = 0.18
            trunk.color.a = 0.85
            array.markers.append(trunk)

            keepout = Marker()
            keepout.header = trunk.header
            keepout.ns = 'tree_keepout'
            keepout.id = index
            keepout.type = Marker.LINE_STRIP
            keepout.action = Marker.ADD
            keepout.pose.orientation.w = 1.0
            keepout.scale.x = 0.06
            keepout.color.r = 0.95
            keepout.color.g = 0.25
            keepout.color.b = 0.10
            keepout.color.a = 0.95
            for segment in range(segments + 1):
                angle = 2.0 * math.pi * segment / segments
                keepout.points.append(Point(
                    x=x + keepout_radius * math.cos(angle),
                    y=y + keepout_radius * math.sin(angle),
                    z=0.04,
                ))
            array.markers.append(keepout)

            label = Marker()
            label.header = trunk.header
            label.ns = 'tree_labels'
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = tree_height + 0.25
            label.pose.orientation.w = 1.0
            label.scale.z = 0.32
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = str(row['tree_id'])
            array.markers.append(label)

        self.publisher.publish(array)
        self.timer.cancel()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TreeMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
