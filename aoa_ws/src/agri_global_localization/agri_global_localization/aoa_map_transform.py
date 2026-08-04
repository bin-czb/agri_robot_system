"""Convert absolute AOA UTM poses into the georeferenced ROS map frame."""

import math
from typing import Iterable, List, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from rclpy.node import Node


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(quaternion: Sequence[float]) -> float:
    """Return planar yaw from an x, y, z, w quaternion."""
    if len(quaternion) != 4 or not all(math.isfinite(v) for v in quaternion):
        raise ValueError('Quaternion must contain four finite values')
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-12:
        raise ValueError('Quaternion norm is zero')
    x, y, z, w = (x / norm, y / norm, z / norm, w / norm)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Build a planar quaternion."""
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw * 0.5)
    quaternion.w = math.cos(yaw * 0.5)
    return quaternion


def transform_utm_position(
        position: Sequence[float],
        origin: Sequence[float],
        map_yaw_rad: float,
        two_d_mode: bool) -> Tuple[float, float, float]:
    """Transform an absolute UTM position into map coordinates."""
    if len(position) != 3 or len(origin) != 3:
        raise ValueError('Position and origin must contain three values')
    values = tuple(position) + tuple(origin) + (map_yaw_rad,)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Position, origin and map yaw must be finite')

    delta_east = position[0] - origin[0]
    delta_north = position[1] - origin[1]
    cosine = math.cos(map_yaw_rad)
    sine = math.sin(map_yaw_rad)

    map_x = cosine * delta_east + sine * delta_north
    map_y = -sine * delta_east + cosine * delta_north
    map_z = 0.0 if two_d_mode else position[2] - origin[2]
    return map_x, map_y, map_z


def rotate_pose_covariance(
        covariance: Iterable[float],
        map_yaw_rad: float) -> List[float]:
    """Rotate a 6x6 pose covariance from UTM ENU axes into map axes."""
    values = list(covariance)
    if len(values) != 36 or not all(math.isfinite(v) for v in values):
        raise ValueError('Pose covariance must contain 36 finite values')

    cosine = math.cos(map_yaw_rad)
    sine = math.sin(map_yaw_rad)
    rotation = (
        (cosine, sine, 0.0),
        (-sine, cosine, 0.0),
        (0.0, 0.0, 1.0),
    )
    jacobian = [[0.0] * 6 for _ in range(6)]
    for block_offset in (0, 3):
        for row in range(3):
            for column in range(3):
                jacobian[block_offset + row][block_offset + column] = (
                    rotation[row][column])

    matrix = [values[row * 6:(row + 1) * 6] for row in range(6)]
    intermediate = [
        [
            sum(jacobian[row][k] * matrix[k][column] for k in range(6))
            for column in range(6)
        ]
        for row in range(6)
    ]
    transformed = [
        [
            sum(
                intermediate[row][k] * jacobian[column][k]
                for k in range(6)
            )
            for column in range(6)
        ]
        for row in range(6)
    ]
    return [value for row in transformed for value in row]


class AoaMapTransform(Node):
    """Adapt the AOA base-link UTM constraint to a ROS map pose."""

    def __init__(self) -> None:
        super().__init__('aoa_utm_to_map')
        self.declare_parameter(
            'input_topic', '/ground_station/absolute_pose_utm_cov')
        self.declare_parameter(
            'output_topic', '/global_pose/aoa_raw_map')
        self.declare_parameter('expected_input_frame', 'utm')
        self.declare_parameter('output_frame', 'map')
        self.declare_parameter('origin_easting', 0.0)
        self.declare_parameter('origin_northing', 0.0)
        self.declare_parameter('origin_altitude', 0.0)
        self.declare_parameter('map_yaw_deg', 0.0)
        self.declare_parameter('two_d_mode', True)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.expected_input_frame = str(
            self.get_parameter('expected_input_frame').value)
        self.output_frame = str(self.get_parameter('output_frame').value)
        self.origin = (
            float(self.get_parameter('origin_easting').value),
            float(self.get_parameter('origin_northing').value),
            float(self.get_parameter('origin_altitude').value),
        )
        self.map_yaw_rad = math.radians(
            float(self.get_parameter('map_yaw_deg').value))
        self.two_d_mode = bool(self.get_parameter('two_d_mode').value)

        if not self.expected_input_frame or not self.output_frame:
            raise ValueError('Input and output frame names must not be empty')
        if not all(math.isfinite(value) for value in self.origin):
            raise ValueError('UTM map origin must be finite')
        if not math.isfinite(self.map_yaw_rad):
            raise ValueError('Map yaw must be finite')

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.output_topic, 10)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.input_topic,
            self._pose_callback,
            10,
        )
        self.accepted = 0
        self.rejected = 0

        self.get_logger().info(
            f'AOA UTM->map: {self.input_topic} -> {self.output_topic}; '
            f'origin=({self.origin[0]:.4f}, {self.origin[1]:.4f}), '
            f'yaw={math.degrees(self.map_yaw_rad):.3f} deg; no TF output')

    def _pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        if message.header.frame_id != self.expected_input_frame:
            self.rejected += 1
            self.get_logger().warning(
                f'Rejected AOA pose in frame {message.header.frame_id!r}; '
                f'expected {self.expected_input_frame!r}',
                throttle_duration_sec=2.0)
            return

        pose = message.pose.pose
        try:
            position = transform_utm_position(
                (
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ),
                self.origin,
                self.map_yaw_rad,
                self.two_d_mode,
            )
            input_yaw = quaternion_to_yaw((
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ))
            covariance = rotate_pose_covariance(
                message.pose.covariance, self.map_yaw_rad)
        except ValueError as error:
            self.rejected += 1
            self.get_logger().warning(
                f'Rejected invalid AOA UTM pose: {error}',
                throttle_duration_sec=2.0)
            return

        output = PoseWithCovarianceStamped()
        output.header = message.header
        output.header.frame_id = self.output_frame
        output.pose.pose.position.x = position[0]
        output.pose.pose.position.y = position[1]
        output.pose.pose.position.z = position[2]
        output.pose.pose.orientation = yaw_to_quaternion(
            normalize_angle(input_yaw - self.map_yaw_rad))
        output.pose.covariance = covariance
        self.publisher.publish(output)
        self.accepted += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AoaMapTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
