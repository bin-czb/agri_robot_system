"""Publish occupied RTAB-Map cells without RViz's opaque unknown-map area."""

import math
from typing import Iterable, List

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from agri_map_integration.geometry import quaternion_to_yaw


def occupied_cell_centers(
        data: Iterable[int],
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        origin_z: float,
        origin_yaw: float,
        occupied_threshold: int,
        z_offset: float = 0.0) -> List[Point]:
    """Convert occupied grid entries to cell centers in the grid frame."""
    values = list(data)
    if width <= 0 or height <= 0 or resolution <= 0.0:
        raise ValueError('Grid dimensions and resolution must be positive')
    if len(values) != width * height:
        raise ValueError('Occupancy data size does not match width * height')

    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)
    points = []

    for index, value in enumerate(values):
        if value < occupied_threshold:
            continue

        row, column = divmod(index, width)
        local_x = (column + 0.5) * resolution
        local_y = (row + 0.5) * resolution

        point = Point()
        point.x = origin_x + cos_yaw * local_x - sin_yaw * local_y
        point.y = origin_y + sin_yaw * local_x + cos_yaw * local_y
        point.z = origin_z + z_offset
        points.append(point)

    return points


class OccupancyGridOverlay(Node):
    """Convert an OccupancyGrid to background-free occupied GridCells."""

    def __init__(self) -> None:
        super().__init__('occupancy_grid_overlay')

        self.declare_parameter('input_topic', '/rtabmap/grid_map')
        self.declare_parameter(
            'output_topic', '/rtabmap/occupied_cells')
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('z_offset', 0.10)

        input_topic = str(
            self.get_parameter('input_topic').value)
        output_topic = str(
            self.get_parameter('output_topic').value)

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher = self.create_publisher(
            GridCells, output_topic, output_qos)
        self.subscription = self.create_subscription(
            OccupancyGrid,
            input_topic,
            self._map_callback,
            input_qos,
        )

        self.get_logger().info(
            f'Visual occupancy overlay: {input_topic} -> {output_topic}')

    def _map_callback(self, message: OccupancyGrid) -> None:
        origin = message.info.origin
        yaw = quaternion_to_yaw(
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        )

        try:
            cells = occupied_cell_centers(
                data=message.data,
                width=message.info.width,
                height=message.info.height,
                resolution=message.info.resolution,
                origin_x=origin.position.x,
                origin_y=origin.position.y,
                origin_z=origin.position.z,
                origin_yaw=yaw,
                occupied_threshold=int(
                    self.get_parameter('occupied_threshold').value),
                z_offset=float(
                    self.get_parameter('z_offset').value),
            )
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        overlay = GridCells()
        overlay.header = message.header
        overlay.cell_width = message.info.resolution
        overlay.cell_height = message.info.resolution
        overlay.cells = cells
        self.publisher.publish(overlay)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OccupancyGridOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

