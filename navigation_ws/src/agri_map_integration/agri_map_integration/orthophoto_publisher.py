"""Publish a georeferenced orthophoto as a static colored PointCloud2."""

import os
import struct
from typing import Dict

from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Trigger
import yaml


def load_map_metadata(path: str) -> Dict:
    with open(path, 'r', encoding='utf-8') as stream:
        metadata = yaml.safe_load(stream)
    if not isinstance(metadata, dict):
        raise ValueError(f'Invalid map metadata: {path}')
    return metadata


def build_orthophoto_cloud(
        image: Image.Image,
        frame_id: str,
        resolution: float,
        z_offset: float,
        pixel_stride: int = 1,
        alpha_threshold: int = 1) -> PointCloud2:
    """Convert non-transparent orthophoto pixels into a colored XY cloud."""
    if resolution <= 0.0:
        raise ValueError('resolution must be positive')
    if pixel_stride < 1:
        raise ValueError('pixel_stride must be at least 1')

    rgba = image.convert('RGBA')
    width, height = rgba.size
    pixels = rgba.load()
    records = bytearray()
    point_count = 0

    for row in range(0, height, pixel_stride):
        y = (height - row - 0.5) * resolution
        for column in range(0, width, pixel_stride):
            red, green, blue, alpha = pixels[column, row]
            if alpha < alpha_threshold:
                continue
            x = (column + 0.5) * resolution
            rgb = (int(red) << 16) | (int(green) << 8) | int(blue)
            records.extend(struct.pack('<fffI', x, y, z_offset, rgb))
            point_count += 1

    message = PointCloud2()
    message.header = Header(frame_id=frame_id)
    message.height = 1
    message.width = point_count
    message.fields = [
        PointField(
            name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(
            name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(
            name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(
            name='rgb', offset=12, datatype=PointField.UINT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 16
    message.row_step = message.point_step * point_count
    message.data = bytes(records)
    message.is_dense = True
    return message


class OrthophotoPublisher(Node):
    """Load the test GeoTIFF once and keep it available with transient QoS."""

    def __init__(self) -> None:
        super().__init__('orthophoto_publisher')
        self.declare_parameter('image_path', '')
        self.declare_parameter('metadata_path', '')
        self.declare_parameter('topic', '/orthophoto/cloud')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('resolution', 0.0)
        self.declare_parameter('z_offset', -0.05)
        self.declare_parameter('pixel_stride', 1)
        self.declare_parameter('alpha_threshold', 1)

        image_path = os.path.expanduser(
            str(self.get_parameter('image_path').value))
        metadata_path = os.path.expanduser(
            str(self.get_parameter('metadata_path').value))
        if not image_path or not os.path.isfile(image_path):
            raise FileNotFoundError(f'Orthophoto not found: {image_path}')
        if not metadata_path or not os.path.isfile(metadata_path):
            raise FileNotFoundError(
                f'Map metadata not found: {metadata_path}')

        metadata = load_map_metadata(metadata_path)
        resolution = float(self.get_parameter('resolution').value)
        if resolution <= 0.0:
            resolution = float(metadata['raster']['nav2_resolution'])

        with Image.open(image_path) as image:
            expected_width = int(metadata['raster']['width'])
            expected_height = int(metadata['raster']['height'])
            if image.size != (expected_width, expected_height):
                raise ValueError(
                    f'Orthophoto size {image.size} does not match metadata '
                    f'{(expected_width, expected_height)}')
            self.cloud = build_orthophoto_cloud(
                image=image,
                frame_id=str(self.get_parameter('frame_id').value),
                resolution=resolution,
                z_offset=float(self.get_parameter('z_offset').value),
                pixel_stride=int(
                    self.get_parameter('pixel_stride').value),
                alpha_threshold=int(
                    self.get_parameter('alpha_threshold').value),
            )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('topic').value),
            qos,
        )
        self.create_service(Trigger, '~/republish', self._republish)
        self.timer = self.create_timer(0.5, self._publish_once)
        self.get_logger().info(
            f'Loaded orthophoto {image_path}: {self.cloud.width} colored '
            f'pixels at {resolution:.3f} m/pixel')

    def _publish_once(self) -> None:
        self._publish()
        self.timer.cancel()

    def _publish(self) -> None:
        self.cloud.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.cloud)

    def _republish(self, request, response):
        del request
        self._publish()
        response.success = True
        response.message = f'Published {self.cloud.width} orthophoto points'
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrthophotoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
