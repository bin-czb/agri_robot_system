"""Differential/skid-steer kinematics shared by command and odometry paths."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class WheelTargets:
    left_mps: float
    right_mps: float
    left_motor_rpm: float
    right_motor_rpm: float
    left_command: int
    right_command: int


class DifferentialKinematics:
    def __init__(
        self,
        *,
        wheel_radius_m: float,
        track_width_m: float,
        gear_ratio: float,
        driver_max_rpm: float,
    ) -> None:
        if wheel_radius_m <= 0.0:
            raise ValueError("wheel_radius_m must be > 0")
        if track_width_m <= 0.0:
            raise ValueError("track_width_m must be > 0")
        if gear_ratio <= 0.0:
            raise ValueError("gear_ratio must be > 0")
        if driver_max_rpm <= 0.0:
            raise ValueError("driver_max_rpm must be > 0")

        self.wheel_radius_m = float(wheel_radius_m)
        self.track_width_m = float(track_width_m)
        self.gear_ratio = float(gear_ratio)
        self.driver_max_rpm = float(driver_max_rpm)

    @property
    def meters_per_wheel_revolution(self) -> float:
        return 2.0 * math.pi * self.wheel_radius_m

    def body_twist_to_targets(
        self,
        linear_mps: float,
        angular_radps: float,
        *,
        left_command_sign: float = 1.0,
        right_command_sign: float = 1.0,
    ) -> WheelTargets:
        left_mps = float(linear_mps) - float(angular_radps) * self.track_width_m * 0.5
        right_mps = float(linear_mps) + float(angular_radps) * self.track_width_m * 0.5

        left_wheel_rpm = left_mps / self.meters_per_wheel_revolution * 60.0
        right_wheel_rpm = right_mps / self.meters_per_wheel_revolution * 60.0
        left_motor_rpm = left_wheel_rpm * self.gear_ratio
        right_motor_rpm = right_wheel_rpm * self.gear_ratio

        peak = max(abs(left_motor_rpm), abs(right_motor_rpm))
        if peak > self.driver_max_rpm:
            scale = self.driver_max_rpm / peak
            left_mps *= scale
            right_mps *= scale
            left_motor_rpm *= scale
            right_motor_rpm *= scale

        left_command = round(
            left_motor_rpm / self.driver_max_rpm * 10000.0 * float(left_command_sign)
        )
        right_command = round(
            right_motor_rpm / self.driver_max_rpm * 10000.0 * float(right_command_sign)
        )
        left_command = max(-10000, min(10000, int(left_command)))
        right_command = max(-10000, min(10000, int(right_command)))

        return WheelTargets(
            left_mps=left_mps,
            right_mps=right_mps,
            left_motor_rpm=left_motor_rpm,
            right_motor_rpm=right_motor_rpm,
            left_command=left_command,
            right_command=right_command,
        )

    def feedback_motor_rpm_to_body_twist(
        self,
        left_motor_rpm: float,
        right_motor_rpm: float,
    ) -> tuple[float, float, float, float]:
        left_wheel_rpm = float(left_motor_rpm) / self.gear_ratio
        right_wheel_rpm = float(right_motor_rpm) / self.gear_ratio
        left_mps = left_wheel_rpm / 60.0 * self.meters_per_wheel_revolution
        right_mps = right_wheel_rpm / 60.0 * self.meters_per_wheel_revolution
        linear_mps = 0.5 * (left_mps + right_mps)
        angular_radps = (right_mps - left_mps) / self.track_width_m
        return left_mps, right_mps, linear_mps, angular_radps


def integrate_midpoint(
    x: float,
    y: float,
    yaw: float,
    linear_mps: float,
    angular_radps: float,
    dt: float,
) -> tuple[float, float, float]:
    if dt <= 0.0:
        return float(x), float(y), float(yaw)
    delta_yaw = float(angular_radps) * float(dt)
    mid_yaw = float(yaw) + 0.5 * delta_yaw
    x_new = float(x) + float(linear_mps) * math.cos(mid_yaw) * float(dt)
    y_new = float(y) + float(linear_mps) * math.sin(mid_yaw) * float(dt)
    yaw_new = math.atan2(math.sin(float(yaw) + delta_yaw), math.cos(float(yaw) + delta_yaw))
    return x_new, y_new, yaw_new
