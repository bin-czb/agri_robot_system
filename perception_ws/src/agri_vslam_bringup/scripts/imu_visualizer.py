#!/usr/bin/python3

import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker, MarkerArray


def rotate_vector(quaternion, vector):
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def clamp_vector(vector, maximum_length):
    length = math.sqrt(sum(component * component for component in vector))
    if length <= maximum_length or length == 0.0:
        return vector
    scale = maximum_length / length
    return tuple(component * scale for component in vector)


class ImuVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')
        self.declare_parameter('fixed_frame', 'imu_world')
        self.declare_parameter('input_topic', '/camera/imu/data')
        self.declare_parameter('pose_topic', '/camera/imu/pose')
        self.declare_parameter('marker_topic', '/camera/imu/markers')
        self.declare_parameter('accel_scale', 0.08)
        self.declare_parameter('gyro_scale', 0.35)

        self.fixed_frame = self.get_parameter('fixed_frame').value
        input_topic = self.get_parameter('input_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        self.accel_scale = self.get_parameter('accel_scale').value
        self.gyro_scale = self.get_parameter('gyro_scale').value

        self.pose_publisher = self.create_publisher(PoseStamped, pose_topic, 10)
        self.marker_publisher = self.create_publisher(MarkerArray, marker_topic, 10)
        self.create_subscription(Imu, input_topic, self.imu_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'Visualizing {input_topic} in {self.fixed_frame} as {pose_topic} and {marker_topic}'
        )

    def imu_callback(self, message):
        q = message.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1.0e-6 or message.orientation_covariance[0] < 0.0:
            return

        quaternion = (q.x / norm, q.y / norm, q.z / norm, q.w / norm)

        pose = PoseStamped()
        pose.header.stamp = message.header.stamp
        pose.header.frame_id = self.fixed_frame
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        self.pose_publisher.publish(pose)

        acceleration = rotate_vector(
            quaternion,
            (
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ),
        )
        angular_velocity = rotate_vector(
            quaternion,
            (
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ),
        )

        acceleration = clamp_vector(
            tuple(value * self.accel_scale for value in acceleration), 1.5
        )
        angular_velocity = clamp_vector(
            tuple(value * self.gyro_scale for value in angular_velocity), 1.5
        )

        markers = MarkerArray()
        markers.markers.append(
            self.make_arrow(message, 0, 'linear_acceleration', acceleration, (1.0, 0.8, 0.1))
        )
        markers.markers.append(
            self.make_arrow(message, 1, 'angular_velocity', angular_velocity, (0.1, 0.8, 1.0))
        )
        markers.markers.append(self.make_text(message, acceleration, angular_velocity))
        self.marker_publisher.publish(markers)

    def make_arrow(self, message, marker_id, namespace, vector, color):
        marker = Marker()
        marker.header.stamp = message.header.stamp
        marker.header.frame_id = self.fixed_frame
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=vector[0], y=vector[1], z=vector[2])]
        marker.scale.x = 0.035
        marker.scale.y = 0.07
        marker.scale.z = 0.10
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        return marker

    def make_text(self, message, acceleration, angular_velocity):
        marker = Marker()
        marker.header.stamp = message.header.stamp
        marker.header.frame_id = self.fixed_frame
        marker.ns = 'imu_values'
        marker.id = 2
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = -0.9
        marker.pose.position.z = 1.25
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.12
        marker.color.r = 0.95
        marker.color.g = 0.95
        marker.color.b = 0.95
        marker.color.a = 1.0

        accel_magnitude = math.sqrt(sum(value * value for value in acceleration)) / self.accel_scale
        gyro_magnitude = math.sqrt(sum(value * value for value in angular_velocity)) / self.gyro_scale
        marker.text = (
            f'IMU  accel={accel_magnitude:.2f} m/s^2  '
            f'gyro={gyro_magnitude:.3f} rad/s\n'
            'yellow: acceleration    cyan: angular velocity'
        )
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = ImuVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
