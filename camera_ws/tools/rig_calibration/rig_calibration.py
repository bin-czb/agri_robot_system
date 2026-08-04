#!/usr/bin/python3
"""Calibration collector for the Orbbec IMU/camera and ground AOA rig.

The tool deliberately keeps calibration acquisition separate from the runtime
localization nodes.  It writes a human-readable profile and can export the AOA
subset as a ROS 2 parameter file.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import math
from pathlib import Path
import sys
import time
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple

import numpy as np
import yaml

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, Imu
    from std_msgs.msg import Float32MultiArray
except ImportError:  # Math self-tests and report/export remain usable without ROS.
    rclpy = None
    Node = object
    CameraInfo = Image = Imu = Float32MultiArray = object
    qos_profile_sensor_data = None


G_MPS2 = 9.80665
SCHEMA_VERSION = 1
DEFAULT_PROFILE = Path("~/.ros/agri_rig_calibration.yaml").expanduser()


def wrap_deg(value):
    values = np.asarray(value, dtype=float)
    wrapped = (values + 180.0) % 360.0 - 180.0
    return float(wrapped) if wrapped.ndim == 0 else wrapped


def circular_mean_deg(values: Iterable[float]) -> float:
    angles = np.radians(np.asarray(list(values), dtype=float))
    if angles.size == 0:
        raise ValueError("cannot compute a circular mean from no samples")
    return wrap_deg(math.degrees(math.atan2(
        float(np.mean(np.sin(angles))), float(np.mean(np.cos(angles))))))


def circular_std_deg(values: Iterable[float]) -> float:
    angles = np.radians(np.asarray(list(values), dtype=float))
    if angles.size < 2:
        return 0.0
    resultant = math.hypot(
        float(np.mean(np.cos(angles))), float(np.mean(np.sin(angles))))
    resultant = min(1.0, max(1e-12, resultant))
    return math.degrees(math.sqrt(max(0.0, -2.0 * math.log(resultant))))


def fit_aoa_axis(stage_samples: Mapping[str, Tuple[float, Sequence[float]]]) -> dict:
    """Fit expected_body = sign * raw + offset on the circle."""
    if len(stage_samples) < 2:
        raise ValueError("AOA axis calibration needs at least two directions")
    candidates = []
    for sign in (-1, 1):
        offsets = []
        observations = []
        for name, (expected_deg, raw_samples) in stage_samples.items():
            raw = np.asarray(raw_samples, dtype=float)
            if raw.size == 0:
                raise ValueError(f"AOA stage {name!r} has no samples")
            offsets.extend(wrap_deg(expected_deg - sign * raw).tolist())
            observations.append((name, expected_deg, raw))
        offset = circular_mean_deg(offsets)
        residuals = []
        stage_report = {}
        for name, expected_deg, raw in observations:
            errors = wrap_deg(sign * raw + offset - expected_deg)
            residuals.extend(errors.tolist())
            stage_report[name] = {
                "expected_body_deg": float(expected_deg),
                "raw_mean_deg": circular_mean_deg(raw),
                "raw_std_deg": circular_std_deg(raw),
                "residual_mean_deg": float(np.mean(errors)),
                "residual_std_deg": float(np.std(errors, ddof=1)) if raw.size > 1 else 0.0,
            }
        residuals_np = np.asarray(residuals, dtype=float)
        candidates.append({
            "azimuth_sign": int(sign),
            "mount_yaw_offset_deg": float(offset),
            "rms_error_deg": float(np.sqrt(np.mean(residuals_np ** 2))),
            "max_abs_error_deg": float(np.max(np.abs(residuals_np))),
            "stages": stage_report,
        })
    return min(candidates, key=lambda item: item["rms_error_deg"])


def validate_aoa_axis_fit(fit: Mapping, max_rms_error_deg: float,
                          max_abs_error_deg: float,
                          max_stage_std_deg: float) -> dict:
    """Return explicit acceptance evidence for an AOA axis fit."""
    stage_std = {
        name: float(stage["raw_std_deg"])
        for name, stage in fit["stages"].items()
    }
    observed_max_stage_std = max(stage_std.values())
    checks = {
        "rms_error": float(fit["rms_error_deg"]) <= max_rms_error_deg,
        "max_abs_error": float(fit["max_abs_error_deg"]) <= max_abs_error_deg,
        "stage_stability": observed_max_stage_std <= max_stage_std_deg,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "limits": {
            "max_rms_error_deg": float(max_rms_error_deg),
            "max_abs_error_deg": float(max_abs_error_deg),
            "max_stage_std_deg": float(max_stage_std_deg),
        },
        "observed": {
            "rms_error_deg": float(fit["rms_error_deg"]),
            "max_abs_error_deg": float(fit["max_abs_error_deg"]),
            "max_stage_std_deg": float(observed_max_stage_std),
            "stage_std_deg": stage_std,
        },
    }


def fit_linear_range(true_m: Sequence[float], measured_raw_m: Sequence[float]) -> dict:
    """Fit raw = scale * true + bias for r_cal=(raw-bias)/scale."""
    true_values = np.asarray(true_m, dtype=float)
    measured = np.asarray(measured_raw_m, dtype=float)
    if true_values.size < 2 or true_values.size != measured.size:
        raise ValueError("range calibration needs at least two matched distances")
    design = np.column_stack((true_values, np.ones_like(true_values)))
    scale, bias = np.linalg.lstsq(design, measured, rcond=None)[0]
    if scale <= 0.0:
        raise ValueError(f"fitted range scale must be positive, got {scale}")
    calibrated = (measured - bias) / scale
    residuals = calibrated - true_values
    return {
        "range_scale": float(scale),
        "range_bias_m": float(bias),
        "rms_error_m": float(np.sqrt(np.mean(residuals ** 2))),
        "max_abs_error_m": float(np.max(np.abs(residuals))),
        "points": [
            {
                "true_m": float(t),
                "raw_mean_m": float(r),
                "calibrated_m": float(c),
                "residual_m": float(e),
            }
            for t, r, c, e in zip(true_values, measured, calibrated, residuals)
        ],
    }


def fit_imu_six_position(samples: Mapping[str, np.ndarray]) -> dict:
    required = {"+x", "-x", "+y", "-y", "+z", "-z"}
    if set(samples) != required:
        raise ValueError(f"six-position IMU data must contain {sorted(required)}")

    means = {key: np.mean(value[:, :3], axis=0) for key, value in samples.items()}
    accel_bias = np.zeros(3)
    accel_gain = np.zeros(3)
    for axis_index, axis_name in enumerate("xyz"):
        positive = means[f"+{axis_name}"][axis_index]
        negative = means[f"-{axis_name}"][axis_index]
        half_span = (positive - negative) / 2.0
        if half_span < 0.5 * G_MPS2:
            raise ValueError(
                f"IMU {axis_name} span is too small ({half_span:.3f} m/s^2); "
                "repeat the six stable orientations")
        accel_bias[axis_index] = (positive + negative) / 2.0
        accel_gain[axis_index] = G_MPS2 / half_span

    all_samples = np.concatenate(list(samples.values()), axis=0)
    gyro_bias = np.mean(all_samples[:, 3:6], axis=0)
    gyro_corrected = all_samples[:, 3:6] - gyro_bias

    corrected_norms = {}
    accel_noise_chunks = []
    for key, values in samples.items():
        corrected = (values[:, :3] - accel_bias) * accel_gain
        corrected_norms[key] = float(np.mean(np.linalg.norm(corrected, axis=1)))
        accel_noise_chunks.append(corrected - np.mean(corrected, axis=0))
    accel_noise = np.concatenate(accel_noise_chunks, axis=0)

    return {
        "gravity_mps2": G_MPS2,
        "accelerometer_bias_mps2": accel_bias.tolist(),
        "accelerometer_gain": accel_gain.tolist(),
        "gyroscope_bias_rad_s": gyro_bias.tolist(),
        "accelerometer_noise_variance": np.var(accel_noise, axis=0, ddof=1).tolist(),
        "gyroscope_noise_variance": np.var(gyro_corrected, axis=0, ddof=1).tolist(),
        "corrected_gravity_norm_by_pose_mps2": corrected_norms,
        "model": "axis_aligned_six_position",
        "warning": (
            "This estimates bias and diagonal gain only. The Orbbec factory "
            "camera-IMU extrinsic and timestamp calibration remain authoritative."
        ),
    }


def message_stamp_seconds(message) -> float:
    stamp = message.header.stamp
    value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return value if value > 0.0 else time.monotonic()


def estimate_rate(stamps: Sequence[float]) -> float:
    if len(stamps) < 2:
        return 0.0
    elapsed = stamps[-1] - stamps[0]
    return float(len(stamps) - 1) / elapsed if elapsed > 0.0 else 0.0


def nearest_sync_statistics(first: Sequence[float], second: Sequence[float]) -> dict:
    if not first or not second:
        return {"pairs": 0}
    second_values = np.asarray(sorted(second), dtype=float)
    differences = []
    for stamp in first:
        index = int(np.searchsorted(second_values, stamp))
        options = []
        if index < second_values.size:
            options.append(abs(second_values[index] - stamp))
        if index > 0:
            options.append(abs(second_values[index - 1] - stamp))
        differences.append(min(options))
    differences_np = np.asarray(differences)
    return {
        "pairs": int(differences_np.size),
        "median_abs_offset_s": float(np.median(differences_np)),
        "p95_abs_offset_s": float(np.percentile(differences_np, 95)),
        "max_abs_offset_s": float(np.max(differences_np)),
    }


def depth_image_median_m(message: Image, roi_fraction: float = 0.2) -> float | None:
    encoding = str(message.encoding).upper()
    if encoding in ("16UC1", "MONO16"):
        dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
        scale = 0.001
    elif encoding == "32FC1":
        dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
        scale = 1.0
    else:
        return None

    item_size = dtype.itemsize
    row_items = int(message.step) // item_size
    raw = np.frombuffer(message.data, dtype=dtype)
    if raw.size < int(message.height) * row_items:
        return None
    image = raw[: int(message.height) * row_items].reshape(int(message.height), row_items)
    image = image[:, : int(message.width)].astype(np.float64) * scale
    half_h = max(1, int(message.height * roi_fraction / 2.0))
    half_w = max(1, int(message.width * roi_fraction / 2.0))
    center_h = int(message.height) // 2
    center_w = int(message.width) // 2
    roi = image[center_h - half_h:center_h + half_h,
                center_w - half_w:center_w + half_w]
    valid = roi[np.isfinite(roi) & (roi > 0.05)]
    return float(np.median(valid)) if valid.size else None


class CalibrationCollector(Node):
    def __init__(self, args):
        super().__init__("agri_rig_calibration")
        self.imu_samples: List[List[float]] = []
        self.imu_stamps: List[float] = []
        self.imu_frame = ""
        self.aoa_samples: List[List[float]] = []
        self.aoa_stamps: List[float] = []
        self.color_stamps: List[float] = []
        self.depth_stamps: List[float] = []
        self.depth_medians: List[float] = []
        self.color_info = None
        self.depth_info = None

        self.create_subscription(
            Imu, args.imu_topic, self._imu_callback, qos_profile_sensor_data)
        self.create_subscription(
            Float32MultiArray, args.aoa_topic, self._aoa_callback, 50)
        self.create_subscription(
            Image, args.color_topic, self._color_callback, qos_profile_sensor_data)
        self.create_subscription(
            Image, args.depth_topic, self._depth_callback, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, args.color_info_topic, self._color_info_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, args.depth_info_topic, self._depth_info_callback,
            qos_profile_sensor_data)

    def _imu_callback(self, message: Imu):
        self.imu_stamps.append(message_stamp_seconds(message))
        self.imu_frame = message.header.frame_id
        self.imu_samples.append([
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        ])

    def _aoa_callback(self, message: Float32MultiArray):
        if len(message.data) >= 4:
            self.aoa_stamps.append(time.monotonic())
            self.aoa_samples.append([float(value) for value in message.data])

    def _color_callback(self, message: Image):
        self.color_stamps.append(message_stamp_seconds(message))

    def _depth_callback(self, message: Image):
        self.depth_stamps.append(message_stamp_seconds(message))
        median = depth_image_median_m(message)
        if median is not None:
            self.depth_medians.append(median)

    def _color_info_callback(self, message: CameraInfo):
        self.color_info = message

    def _depth_info_callback(self, message: CameraInfo):
        self.depth_info = message

    def clear_imu(self):
        self.imu_samples.clear()
        self.imu_stamps.clear()

    def clear_aoa(self):
        self.aoa_samples.clear()
        self.aoa_stamps.clear()

    def clear_depth(self):
        self.depth_medians.clear()
        self.depth_stamps.clear()


def require_ros():
    if rclpy is None:
        raise RuntimeError(
            "ROS 2 Python modules are unavailable. Source /opt/ros/humble/setup.bash "
            "and the Orbbec/navigation workspaces, then use /usr/bin/python3.")


def collect_for(node: CalibrationCollector, duration: float, label: str):
    print(f"采集 {label}: {duration:.1f} 秒，请保持当前状态...")
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def camera_info_dict(message: CameraInfo | None) -> dict:
    if message is None:
        return {"received": False}
    return {
        "received": True,
        "width": int(message.width),
        "height": int(message.height),
        "frame_id": str(message.header.frame_id),
        "distortion_model": str(message.distortion_model),
        "d": [float(value) for value in message.d],
        "k": [float(value) for value in message.k],
        "r": [float(value) for value in message.r],
        "p": [float(value) for value in message.p],
    }


def load_profile(path: Path) -> dict:
    if not path.exists():
        return {"schema_version": SCHEMA_VERSION, "created_utc": now_utc()}
    with path.open("r", encoding="utf-8") as stream:
        profile = yaml.safe_load(stream) or {}
    if not isinstance(profile, dict):
        raise ValueError(f"profile {path} is not a YAML mapping")
    return profile


def now_utc() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def save_profile(path: Path, profile: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    profile["schema_version"] = SCHEMA_VERSION
    profile["updated_utc"] = now_utc()
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(profile, stream, sort_keys=False, allow_unicode=True)
    temporary.replace(path)
    print(f"已写入标定记录: {path}")


def update_profile(path: Path, section: str, result: dict):
    profile = load_profile(path)
    profile[section] = result
    save_profile(path, profile)


def run_with_collector(args, callback):
    require_ros()
    rclpy.init(args=None)
    node = CalibrationCollector(args)
    try:
        return callback(node, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def command_check(node: CalibrationCollector, args):
    collect_for(node, args.duration, "相机/IMU/AOA状态")
    imu_values = np.asarray(node.imu_samples, dtype=float)
    result = {
        "captured_utc": now_utc(),
        "topics": {
            "color": args.color_topic,
            "depth": args.depth_topic,
            "imu": args.imu_topic,
            "aoa": args.aoa_topic,
        },
        "rates_hz": {
            "color": estimate_rate(node.color_stamps),
            "depth": estimate_rate(node.depth_stamps),
            "imu": estimate_rate(node.imu_stamps),
            "aoa": estimate_rate(node.aoa_stamps),
        },
        "sample_counts": {
            "color": len(node.color_stamps),
            "depth": len(node.depth_stamps),
            "imu": len(node.imu_stamps),
            "aoa": len(node.aoa_stamps),
        },
        "rgb_depth_sync": nearest_sync_statistics(
            node.color_stamps, node.depth_stamps),
        "color_camera_info": camera_info_dict(node.color_info),
        "depth_camera_info": camera_info_dict(node.depth_info),
        "imu_frame_id": node.imu_frame,
    }
    if imu_values.size:
        result["stationary_snapshot"] = {
            "acceleration_mean_mps2": np.mean(imu_values[:, :3], axis=0).tolist(),
            "acceleration_norm_mean_mps2": float(np.mean(
                np.linalg.norm(imu_values[:, :3], axis=1))),
            "angular_velocity_mean_rad_s": np.mean(imu_values[:, 3:6], axis=0).tolist(),
        }
    update_profile(args.profile, "device_check", result)
    print(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))


def command_imu_six(node: CalibrationCollector, args):
    print("IMU六面标定：每次将刚性传感器架稳定放置在不同表面。")
    print("每个姿态必须让一个IMU轴尽量竖直，任意倾斜的六个静止角度不满足六面法。")
    print("脚本会自动识别IMU消息坐标的 +X/-X/+Y/-Y/+Z/-Z，无需猜物理轴。")
    collected: Dict[str, np.ndarray] = {}
    pose_statistics = {}
    while len(collected) < 6:
        print(f"\n已完成: {', '.join(sorted(collected)) or '无'}")
        input("换到一个尚未采集的稳定姿态，静置后按 Enter 开始...")
        node.clear_imu()
        collect_for(node, args.duration, "静态IMU")
        values = np.asarray(node.imu_samples, dtype=float)
        if values.shape[0] < args.min_samples:
            print(f"样本不足: {values.shape[0]} < {args.min_samples}，请重试。")
            continue
        accel_mean = np.mean(values[:, :3], axis=0)
        accel_std = np.std(values[:, :3], axis=0)
        gyro_mean = np.mean(values[:, 3:6], axis=0)
        gyro_norm = float(np.linalg.norm(gyro_mean))
        acceleration_norm = float(np.linalg.norm(accel_mean))
        axis_index = int(np.argmax(np.abs(accel_mean)))
        sign = "+" if accel_mean[axis_index] >= 0.0 else "-"
        key = f"{sign}{'xyz'[axis_index]}"
        alignment_ratio = min(
            1.0, abs(float(accel_mean[axis_index])) / max(acceleration_norm, 1e-9))
        tilt_deg = math.degrees(math.acos(alignment_ratio))
        print(
            f"识别姿态 {key}: accel_mean={accel_mean.round(5).tolist()}, "
            f"accel_std={accel_std.round(5).tolist()}, gyro_mean_norm={gyro_norm:.6f}, "
            f"主轴偏离重力={tilt_deg:.2f}°")
        if gyro_norm > args.max_gyro_mean or float(np.max(accel_std)) > args.max_accel_std:
            print("静止性检查未通过，采集期间可能发生移动，请重试。")
            continue
        if tilt_deg > args.max_tilt_deg:
            print(
                f"姿态对准检查未通过: {tilt_deg:.2f}° > {args.max_tilt_deg:.2f}°。"
                "请让相机架的一个面完全贴合水平桌面后重试。")
            continue
        if key in collected:
            print(f"姿态 {key} 已采集，请换到相反或另一轴朝上的表面。")
            continue
        collected[key] = values
        pose_statistics[key] = {
            "samples": int(values.shape[0]),
            "acceleration_mean_mps2": accel_mean.tolist(),
            "acceleration_norm_mps2": acceleration_norm,
            "acceleration_std_mps2": accel_std.tolist(),
            "gyroscope_mean_rad_s": gyro_mean.tolist(),
            "dominant_axis_tilt_deg": tilt_deg,
        }

    fit = fit_imu_six_position(collected)
    fit["captured_utc"] = now_utc()
    fit["topic"] = args.imu_topic
    fit["frame_id"] = node.imu_frame
    fit["samples_per_pose"] = {key: int(value.shape[0]) for key, value in collected.items()}
    fit["pose_statistics"] = pose_statistics
    gravity_errors = {
        key: abs(value - G_MPS2)
        for key, value in fit["corrected_gravity_norm_by_pose_mps2"].items()
    }
    max_gravity_error = max(gravity_errors.values())
    fit["validation"] = {
        "gravity_error_by_pose_mps2": gravity_errors,
        "max_gravity_error_mps2": max_gravity_error,
        "limit_mps2": args.max_corrected_gravity_error,
        "passed": bool(max_gravity_error <= args.max_corrected_gravity_error),
    }
    if not fit["validation"]["passed"]:
        update_profile(args.profile, "imu_rejected_candidate", fit)
        raise RuntimeError(
            f"IMU candidate rejected: corrected gravity error "
            f"{max_gravity_error:.4f} m/s^2 exceeds "
            f"{args.max_corrected_gravity_error:.4f} m/s^2. "
            "The accepted imu section was not overwritten.")
    update_profile(args.profile, "imu", fit)
    print(yaml.safe_dump(fit, sort_keys=False, allow_unicode=True))


def command_aoa_axis(node: CalibrationCollector, args):
    stages = [
        ("front", 0.0, "将AOA标签放在相机正前方中心线上"),
        ("left", 90.0, "将标签放在传感器架正左侧"),
        ("right", -90.0, "将标签放在传感器架正右侧"),
    ]
    if not args.skip_back:
        stages.append(("back", 180.0, "将标签放在传感器架正后方"))
    samples = {}
    for name, expected, instruction in stages:
        print(f"\n{name}: {instruction}，AOA与标签尽量同高，距离建议2米以上。")
        input("位置稳定后按 Enter 开始采集...")
        node.clear_aoa()
        collect_for(node, args.duration, f"AOA {name}")
        values = np.asarray(node.aoa_samples, dtype=float)
        if values.shape[0] < args.min_samples:
            raise RuntimeError(
                f"AOA {name} only received {values.shape[0]} samples; "
                "check /aoa/measurement and the serial link")
        samples[name] = (expected, values[:, 2].copy())

    fit = fit_aoa_axis(samples)
    fit["captured_utc"] = now_utc()
    fit["topic"] = args.aoa_topic
    fit["validation"] = validate_aoa_axis_fit(
        fit,
        args.max_rms_error_deg,
        args.max_abs_error_deg,
        args.max_stage_std_deg,
    )
    if not fit["validation"]["passed"]:
        update_profile(args.profile, "aoa_axis_rejected_candidate", fit)
        raise RuntimeError(
            "AOA axis candidate rejected: "
            f"RMS={fit['rms_error_deg']:.2f} deg, "
            f"max_abs={fit['max_abs_error_deg']:.2f} deg, "
            "max_stage_std="
            f"{fit['validation']['observed']['max_stage_std_deg']:.2f} deg. "
            "Check line of sight, phase-centre placement and 180-degree flips. "
            "The accepted aoa_axis section was not overwritten."
        )
    update_profile(args.profile, "aoa_axis", fit)
    print(yaml.safe_dump(fit, sort_keys=False, allow_unicode=True))


def command_aoa_range(node: CalibrationCollector, args):
    raw_means = []
    raw_stds = []
    for distance in args.distances:
        print(
            f"\n将AOA相位中心到标签相位中心的真实斜距设置为 {distance:.3f} m。")
        input("测量并固定后按 Enter 开始采集...")
        node.clear_aoa()
        collect_for(node, args.duration, f"AOA range {distance:.3f} m")
        values = np.asarray(node.aoa_samples, dtype=float)
        if values.shape[0] < args.min_samples:
            raise RuntimeError(f"distance {distance} m received too few AOA samples")
        raw_means.append(float(np.mean(values[:, 0])))
        raw_stds.append(float(np.std(values[:, 0], ddof=1)))
    fit = fit_linear_range(args.distances, raw_means)
    fit["captured_utc"] = now_utc()
    fit["raw_std_m"] = raw_stds
    fit["topic"] = args.aoa_topic
    update_profile(args.profile, "aoa_range", fit)
    print(yaml.safe_dump(fit, sort_keys=False, allow_unicode=True))


def command_depth_range(node: CalibrationCollector, args):
    measured = []
    measured_stds = []
    for distance in args.distances:
        print(
            f"\n将相机光学中心正对平整墙面，真实垂直距离设为 {distance:.3f} m。")
        input("墙面位于图像中心且相机静止后按 Enter 开始采集...")
        node.clear_depth()
        collect_for(node, args.duration, f"depth range {distance:.3f} m")
        if len(node.depth_medians) < args.min_samples:
            raise RuntimeError(
                f"distance {distance} m produced only {len(node.depth_medians)} valid depth frames")
        measured.append(float(np.mean(node.depth_medians)))
        measured_stds.append(float(np.std(node.depth_medians, ddof=1)))
    fit = fit_linear_range(args.distances, measured)
    fit["captured_utc"] = now_utc()
    fit["frame_median_std_m"] = measured_stds
    fit["topic"] = args.depth_topic
    fit["interpretation"] = (
        "Validation result only. Keep Orbbec factory depth calibration unless "
        "the residual is repeatable and exceeds the experiment requirement."
    )
    update_profile(args.profile, "camera_depth_range", fit)
    print(yaml.safe_dump(fit, sort_keys=False, allow_unicode=True))


def command_extrinsics(args):
    result = {
        "captured_utc": now_utc(),
        "coordinate_convention": "FLU: x forward, y left, z up; metres and radians",
        "base_to_aoa_body_m": [float(value) for value in args.base_to_aoa],
        "uav_rtk_to_aoa_tag_body_m": [float(value) for value in args.tag_offset],
        "base_to_camera_link": {
            "translation_m": [float(value) for value in args.base_to_camera],
            "rpy_rad": [float(value) for value in args.base_to_camera_rpy],
        },
        "method": "manual rigid-body measurement",
    }
    update_profile(args.profile, "extrinsics", result)
    print(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))


def command_export(args):
    profile = load_profile(args.profile)
    axis = profile.get("aoa_axis", {})
    range_fit = profile.get("aoa_range", {})
    extrinsics = profile.get("extrinsics", {})
    parameters = {}
    if axis:
        if axis.get("validation", {}).get("passed") is not True:
            raise RuntimeError(
                "aoa_axis is missing a passed validation result; repeat "
                "aoa-axis calibration before exporting runtime parameters")
        parameters.update({
            "aoa_azimuth_sign": int(axis["azimuth_sign"]),
            "aoa_mount_yaw_offset_deg": float(axis["mount_yaw_offset_deg"]),
        })
    if range_fit:
        parameters.update({
            "aoa_range_bias_m": float(range_fit["range_bias_m"]),
            "aoa_range_scale": float(range_fit["range_scale"]),
        })
    if extrinsics:
        parameters["base_to_aoa_body_m"] = extrinsics["base_to_aoa_body_m"]
        parameters["tag_offset_body_m"] = extrinsics["uav_rtk_to_aoa_tag_body_m"]
    if not parameters:
        raise RuntimeError("profile does not yet contain AOA or extrinsic calibration")
    output = {"aoa_localization_node": {"ros__parameters": parameters}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(output, stream, sort_keys=False, allow_unicode=True)
    print(f"已生成AOA ROS参数文件: {args.output}")
    print(yaml.safe_dump(output, sort_keys=False, allow_unicode=True))


def command_report(args):
    profile = load_profile(args.profile)
    print(yaml.safe_dump(profile, sort_keys=False, allow_unicode=True))


def command_self_test(_args):
    expected = {"front": 0.0, "left": 90.0, "right": -90.0, "back": 180.0}
    rng = np.random.default_rng(7)
    stages = {}
    true_sign = -1
    true_offset = 13.5
    for name, angle in expected.items():
        raw_center = true_sign * (angle - true_offset)
        stages[name] = (angle, raw_center + rng.normal(0.0, 0.4, 100))
    axis_fit = fit_aoa_axis(stages)
    assert axis_fit["azimuth_sign"] == true_sign
    assert abs(wrap_deg(axis_fit["mount_yaw_offset_deg"] - true_offset)) < 0.2

    true_ranges = np.array([1.0, 2.0, 4.0, 7.0])
    range_fit = fit_linear_range(true_ranges, 1.025 * true_ranges + 0.18)
    assert abs(range_fit["range_scale"] - 1.025) < 1e-9
    assert abs(range_fit["range_bias_m"] - 0.18) < 1e-9

    accel_bias = np.array([0.08, -0.05, 0.12])
    gains = np.array([1.01, 0.98, 1.03])
    imu_samples = {}
    for axis_index, axis_name in enumerate("xyz"):
        for sign, prefix in ((1.0, "+"), (-1.0, "-")):
            values = np.zeros((200, 6))
            values[:, :3] = accel_bias
            values[:, axis_index] += sign * G_MPS2 / gains[axis_index]
            values[:, :3] += rng.normal(0.0, 0.01, (200, 3))
            values[:, 3:6] = np.array([0.002, -0.003, 0.001])
            values[:, 3:6] += rng.normal(0.0, 0.0002, (200, 3))
            imu_samples[f"{prefix}{axis_name}"] = values
    imu_fit = fit_imu_six_position(imu_samples)
    assert np.allclose(imu_fit["accelerometer_bias_mps2"], accel_bias, atol=0.003)
    assert np.allclose(imu_fit["accelerometer_gain"], gains, atol=0.001)
    print("rig_calibration self-test: PASS")


def add_common_ros_arguments(parser):
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--imu-topic", default="/camera/gyro_accel/sample")
    parser.add_argument("--aoa-topic", default="/aoa/measurement")
    parser.add_argument("--color-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--color-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--depth-info-topic", default="/camera/depth/camera_info")


def build_parser():
    parser = argparse.ArgumentParser(
        description="Calibrate and validate the Orbbec camera/IMU plus AOA rigid rig")
    subparsers = parser.add_subparsers(dest="command", required=True)

    check = subparsers.add_parser("check", help="capture topic rates and camera intrinsics")
    add_common_ros_arguments(check)
    check.add_argument("--duration", type=float, default=10.0)
    check.set_defaults(handler=lambda args: run_with_collector(args, command_check))

    imu = subparsers.add_parser("imu-six", help="guided six-position IMU calibration")
    add_common_ros_arguments(imu)
    imu.add_argument("--duration", type=float, default=5.0)
    imu.add_argument("--min-samples", type=int, default=300)
    imu.add_argument("--max-gyro-mean", type=float, default=0.05)
    imu.add_argument("--max-accel-std", type=float, default=0.20)
    imu.add_argument(
        "--max-tilt-deg", type=float, default=8.0,
        help="maximum dominant-axis deviation from gravity for each face")
    imu.add_argument(
        "--max-corrected-gravity-error", type=float, default=0.20,
        help="maximum accepted gravity-norm residual after calibration")
    imu.set_defaults(handler=lambda args: run_with_collector(args, command_imu_six))

    axis = subparsers.add_parser("aoa-axis", help="fit AOA sign and mounting yaw offset")
    add_common_ros_arguments(axis)
    axis.add_argument("--duration", type=float, default=6.0)
    axis.add_argument("--min-samples", type=int, default=10)
    axis.add_argument("--skip-back", action="store_true")
    axis.add_argument("--max-rms-error-deg", type=float, default=8.0)
    axis.add_argument("--max-abs-error-deg", type=float, default=30.0)
    axis.add_argument("--max-stage-std-deg", type=float, default=10.0)
    axis.set_defaults(handler=lambda args: run_with_collector(args, command_aoa_axis))

    aoa_range = subparsers.add_parser("aoa-range", help="fit AOA range scale and bias")
    add_common_ros_arguments(aoa_range)
    aoa_range.add_argument("distances", nargs="+", type=float)
    aoa_range.add_argument("--duration", type=float, default=6.0)
    aoa_range.add_argument("--min-samples", type=int, default=10)
    aoa_range.set_defaults(handler=lambda args: run_with_collector(args, command_aoa_range))

    depth = subparsers.add_parser("depth-range", help="validate center-ROI depth scale")
    add_common_ros_arguments(depth)
    depth.add_argument("distances", nargs="+", type=float)
    depth.add_argument("--duration", type=float, default=4.0)
    depth.add_argument("--min-samples", type=int, default=20)
    depth.set_defaults(handler=lambda args: run_with_collector(args, command_depth_range))

    extrinsics = subparsers.add_parser("set-extrinsics", help="record measured rigid lever arms")
    extrinsics.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    extrinsics.add_argument("--base-to-aoa", nargs=3, type=float, required=True,
                            metavar=("X", "Y", "Z"))
    extrinsics.add_argument("--tag-offset", nargs=3, type=float, required=True,
                            metavar=("X", "Y", "Z"))
    extrinsics.add_argument("--base-to-camera", nargs=3, type=float, required=True,
                            metavar=("X", "Y", "Z"))
    extrinsics.add_argument("--base-to-camera-rpy", nargs=3, type=float, required=True,
                            metavar=("ROLL", "PITCH", "YAW"))
    extrinsics.set_defaults(handler=command_extrinsics)

    export = subparsers.add_parser("export-aoa", help="export AOA ROS 2 parameters")
    export.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    export.add_argument(
        "--output", type=Path,
        default=Path("~/.ros/agri_aoa_calibration.params.yaml").expanduser())
    export.set_defaults(handler=command_export)

    report = subparsers.add_parser("report", help="print the current profile")
    report.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    report.set_defaults(handler=command_report)

    self_test = subparsers.add_parser("self-test", help="run deterministic math tests")
    self_test.set_defaults(handler=command_self_test)
    return parser


def main():
    args = build_parser().parse_args()
    try:
        args.handler(args)
    except (KeyboardInterrupt, EOFError):
        print("\n标定已取消，未完成的阶段不会写入结果。", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"标定失败: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
