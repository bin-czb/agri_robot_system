#!/usr/bin/python3
"""Apply an accepted six-position Orbbec IMU profile."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu
import yaml


def covariance_from_diagonal(diagonal):
    matrix = np.zeros((3, 3), dtype=float)
    np.fill_diagonal(matrix, np.asarray(diagonal, dtype=float))
    return matrix.reshape(-1).tolist()


class ImuBiasCorrector(Node):
    def __init__(self, args):
        super().__init__('orbbec_imu_bias_corrector')
        with args.profile.open('r', encoding='utf-8') as stream:
            profile = yaml.safe_load(stream) or {}
        calibration = profile.get('imu')
        if not calibration:
            raise RuntimeError(
                f'{args.profile} has no imu section; run the six-position calibration first')
        validation = calibration.get('validation')
        if not validation or validation.get('passed') is not True:
            raise RuntimeError(
                f'{args.profile} contains an old or rejected IMU calibration; '
                'refusing to apply it')

        self.accel_bias = np.asarray(
            calibration['accelerometer_bias_mps2'], dtype=float).reshape(3)
        self.accel_gain = np.asarray(
            calibration['accelerometer_gain'], dtype=float).reshape(3)
        self.gyro_bias = np.asarray(
            calibration['gyroscope_bias_rad_s'], dtype=float).reshape(3)
        self.accel_covariance = covariance_from_diagonal(
            calibration['accelerometer_noise_variance'])
        self.gyro_covariance = covariance_from_diagonal(
            calibration['gyroscope_noise_variance'])

        self.publisher = self.create_publisher(
            Imu, args.output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Imu, args.input_topic, self._callback, qos_profile_sensor_data)
        self.get_logger().info(
            f'Applying accepted IMU profile {args.profile}: '
            f'{args.input_topic} -> {args.output_topic}')

    def _callback(self, source: Imu):
        acceleration = np.array([
            source.linear_acceleration.x,
            source.linear_acceleration.y,
            source.linear_acceleration.z,
        ])
        angular_velocity = np.array([
            source.angular_velocity.x,
            source.angular_velocity.y,
            source.angular_velocity.z,
        ])
        acceleration = (acceleration - self.accel_bias) * self.accel_gain
        angular_velocity = angular_velocity - self.gyro_bias

        target = Imu()
        target.header = source.header
        target.orientation = source.orientation
        target.orientation_covariance = list(source.orientation_covariance)
        target.linear_acceleration.x = float(acceleration[0])
        target.linear_acceleration.y = float(acceleration[1])
        target.linear_acceleration.z = float(acceleration[2])
        target.angular_velocity.x = float(angular_velocity[0])
        target.angular_velocity.y = float(angular_velocity[1])
        target.angular_velocity.z = float(angular_velocity[2])
        target.linear_acceleration_covariance = self.accel_covariance
        target.angular_velocity_covariance = self.gyro_covariance
        self.publisher.publish(target)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--profile', type=Path,
        default=Path('~/.ros/agri_rig_calibration.yaml').expanduser())
    parser.add_argument('--input-topic', default='/camera/gyro_accel/sample')
    parser.add_argument('--output-topic', default='/camera/imu/calibrated_raw')
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    rclpy.init()
    node = ImuBiasCorrector(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
