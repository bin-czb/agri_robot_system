#!/usr/bin/env python3
"""
AOA Relative Localization Node — GROUND-BASE + UAV-TAG configuration.

Geometry:

  * AOA base station is mounted on the GROUND (PC-connected).
  * AOA Tag is on the UAV.
  * The AOA/camera/IMU sensor rig is rigidly mounted on the ground
    vehicle.  Its body frame is ROS FLU (x forward, y left, z up).
  * A fused base heading rotates the measured body bearing into ENU:
        theta_enu = theta_body + base_yaw_enu
  * /uav/utm_pose (RTK) still provides the UAV absolute position.
  * AOA range is treated as a SLANT range from base → tag.  We first
    project that slant observation to the horizontal plane:
        dz_tag    = altitude_uav + Lz − base_height
        rho_tag   = sqrt(max(r² − dz_tag², 0))
        p_tag_rel = [ rho_tag·cos(theta_enu),
                      rho_tag·sin(theta_enu), dz_tag ]
  * If the tag is offset from the RTK antenna by L_body, we rotate that
    offset into ENU and recover the UAV/RTK reference point:
        p_uav_rel = p_tag_rel − R_enu←flu(ψ, θ, φ) · L_body
    where R_enu←flu is the FULL 3-axis UAV attitude rotation (yaw ψ +
    pitch θ + roll φ), reconstructed locally from the hybrid quaternion
    published by uav_gps_node.py (see _tag_offset_enu for the pitch/roll
    sign-flip rationale).  This is the vector "FROM the base TO the
    UAV/RTK point", expressed in ENU.
  * The ground base_link→AOA phase-centre lever arm is rotated with
    base_yaw_enu before solving the base_link absolute position.

Published topics (semantics are REVERSED vs the UWB frontend):

  /ground_station/relative_pose        —  UAV relative to base (NOT
                                          GS relative to UAV!).
  /ground_station/absolute_pose_utm    —  SOLVED base position in UTM
                                          (primary deliverable).
  /ground_station/absolute_pose_local  —  SOLVED base position in
                                          local_origin frame.
  /ground_station/absolute_pose_optimized_* — comparison-only SciPy
                                          sliding-window robust backend.
  /uav/local_pose                      —  UAV in local_origin frame.
  /ground_station/debug                —  debug[0..2] = p_uav_rel.
  /aoa/measurement                     —  raw per-frame observation.

This is a TEST stage, not a fusion layer:
  - single range+bearing observation (AOA_USE_ELEVATION=False)
  - light EMA smoothing only (no EKF); Z reuses the altitude chain
  - single-frame outlier gates (angle rate, range rate, EMA-singular)
  - ``fixed_east`` remains available for legacy bench tests
  - moving-cart modes require a fresh fused odometry/IMU orientation
"""

import math
import threading
import time
from collections import deque
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Float64
from trunk_interfaces.msg import AoaObservation
from scipy.spatial.transform import Rotation

import serial  # pyserial

from .aoa_config import (
    AOA_SERIAL_PORT, AOA_BAUDRATE, AOA_TIMEOUT,
    AOA_MOUNT_YAW_OFFSET_DEG, AOA_AZIMUTH_SIGN,
    AOA_RANGE_BIAS_M, AOA_RANGE_SCALE,
    AOA_MIN_RANGE, AOA_MAX_RANGE,
    AOA_MEAS_TIMEOUT, AOA_HEARTBEAT_TIMEOUT,
    AOA_USE_ELEVATION,
    AOA_RANGE_ALPHA, AOA_ANGLE_ALPHA,
    AOA_OPT_ENABLE, AOA_OPT_WINDOW_SIZE, AOA_OPT_MIN_FRAMES,
    AOA_OPT_MAX_NFEV, AOA_OPT_SIGMA_RHO_M, AOA_OPT_SIGMA_THETA_DEG,
    AOA_OPT_SIGMA_BETA_PRIOR_DEG, AOA_OPT_BETA_BOUND_DEG,
    AOA_OPT_LOSS, AOA_OPT_F_SCALE, AOA_OPT_REQUIRE_WORLD_ORIGIN,
    AOA_ANGLE_JUMP_MAX_RATE_DPS, AOA_ANGLE_JUMP_SLACK_DEG,
    AOA_ANGLE_JUMP_DT_MAX, AOA_RANGE_JUMP_MAX_MPS,
    AOA_ANGLE_EMA_MIN_MAG, AOA_ANGLE_CONSEC_REJECT_RESET,
    AOA_BASE_HEADING_MODE, AOA_BASE_ODOMETRY_TOPIC, AOA_BASE_IMU_TOPIC,
    AOA_BASE_INITIAL_HEADING_ENU_DEG, AOA_BASE_HEADING_TIMEOUT,
    AOA_BASE_FIXED_YAW_DEG,
    AOA_BASE_TO_ANTENNA_BODY_M,
    AOA_POSITION_SIGMA_RANGE_M, AOA_POSITION_SIGMA_AZIMUTH_DEG,
    AOA_BASE_HEADING_SIGMA_DEG,
    AOA_TAG_OFFSET_BODY_M,
    AOA_BASE_HINT_DIR_ENU,
    AOA_DISAMBIG_HYSTERESIS_FRAMES, AOA_DISAMBIG_MARGIN_M,
    AOA_MIN_GEOM_ELEVATION_DEG,
    ALTITUDE_STD, TAG_HEIGHT, ALTITUDE_TIMEOUT, POSE_TIMEOUT,
    WORLD_ORIGIN_MODE, FIXED_ORIGIN_E, FIXED_ORIGIN_N, FIXED_ORIGIN_Z,
    ORIGIN_STABLE_FRAMES, ORIGIN_PAIRWISE_THRESH, ORIGIN_WINDOW_RADIUS,
)
from .aoa_protocol import (
    AoaFrameParser, CMD_POSITION, CMD_HEARTBEAT,
)
from .aoa_sliding_window_optimizer import AoaSlidingWindowOptimizer
from .aoa_state_history import UavStateHistory
from .aoa_geometry import (
    body_bearing_to_enu,
    rotate_body_xy_to_enu,
    solve_ground_base_position,
    wrap_angle_rad,
)


LINK_STATUS_STALE        = 0  # no heartbeat and no position
LINK_STATUS_HEARTBEAT    = 1  # heartbeat ok but no recent position
LINK_STATUS_POSITION_OK  = 2  # have fresh position frame

# Reasons frames can be dropped.  These codes are exposed on
# /aoa/measurement so external tooling can count gate activity.
GATE_OK                 = 0
GATE_POSE_STALE         = 1
GATE_NO_STATE           = 2
GATE_RANGE_PHYSICS      = 3
GATE_RANGE_RATE         = 4
GATE_ANGLE_RATE         = 5
GATE_EMA_SINGULARITY    = 6
GATE_LOW_ELEVATION      = 7
GATE_BASE_HEADING_STALE = 8


class AoaLocalizationNode(Node):

    def __init__(self, **kwargs):
        super().__init__('aoa_localization_node', **kwargs)

        # ── ROS Parameters ──────────────────────────────────────────
        self.declare_parameter('serial_port',  AOA_SERIAL_PORT)
        self.declare_parameter('baud',         AOA_BAUDRATE)
        self.declare_parameter('tag_height',   TAG_HEIGHT)
        self.declare_parameter('altitude_std', ALTITUDE_STD)
        self.declare_parameter('output_frame', 'enu')
        self.declare_parameter('allow_pose_altitude_fallback', True)
        self.declare_parameter('altitude_timeout', ALTITUDE_TIMEOUT)
        self.declare_parameter('pose_timeout',     POSE_TIMEOUT)
        self.declare_parameter('world_origin_mode', WORLD_ORIGIN_MODE)
        self.declare_parameter('fixed_origin_e', FIXED_ORIGIN_E)
        self.declare_parameter('fixed_origin_n', FIXED_ORIGIN_N)
        self.declare_parameter('fixed_origin_z', FIXED_ORIGIN_Z)
        self.declare_parameter('base_heading_mode', AOA_BASE_HEADING_MODE)
        self.declare_parameter('base_odometry_topic', AOA_BASE_ODOMETRY_TOPIC)
        self.declare_parameter('base_imu_topic', AOA_BASE_IMU_TOPIC)
        self.declare_parameter(
            'base_initial_heading_enu_deg',
            AOA_BASE_INITIAL_HEADING_ENU_DEG)
        self.declare_parameter(
            'base_heading_timeout', AOA_BASE_HEADING_TIMEOUT)
        self.declare_parameter(
            'base_fixed_yaw_enu_deg', AOA_BASE_FIXED_YAW_DEG)
        self.declare_parameter(
            'base_to_aoa_body_m', AOA_BASE_TO_ANTENNA_BODY_M)
        self.declare_parameter(
            'tag_offset_body_m', AOA_TAG_OFFSET_BODY_M)
        self.declare_parameter(
            'position_sigma_range_m', AOA_POSITION_SIGMA_RANGE_M)
        self.declare_parameter(
            'position_sigma_azimuth_deg', AOA_POSITION_SIGMA_AZIMUTH_DEG)
        self.declare_parameter(
            'base_heading_sigma_deg', AOA_BASE_HEADING_SIGMA_DEG)
        self.declare_parameter('aoa_mount_yaw_offset_deg', AOA_MOUNT_YAW_OFFSET_DEG)
        self.declare_parameter('aoa_azimuth_sign', AOA_AZIMUTH_SIGN)
        self.declare_parameter('aoa_range_bias_m', AOA_RANGE_BIAS_M)
        self.declare_parameter('aoa_range_scale',  AOA_RANGE_SCALE)
        self.declare_parameter('aoa_range_alpha',  AOA_RANGE_ALPHA)
        self.declare_parameter('aoa_angle_alpha',  AOA_ANGLE_ALPHA)
        self.declare_parameter('allow_gate_force_reseed', False)
        self.declare_parameter('aoa_opt_enable', AOA_OPT_ENABLE)
        self.declare_parameter('aoa_opt_window_size', AOA_OPT_WINDOW_SIZE)
        self.declare_parameter('aoa_opt_min_frames', AOA_OPT_MIN_FRAMES)
        self.declare_parameter('aoa_opt_sigma_rho_m', AOA_OPT_SIGMA_RHO_M)
        self.declare_parameter('aoa_opt_sigma_theta_deg', AOA_OPT_SIGMA_THETA_DEG)
        self.declare_parameter(
            'aoa_opt_sigma_beta_prior_deg', AOA_OPT_SIGMA_BETA_PRIOR_DEG)
        self.declare_parameter('aoa_opt_beta_bound_deg', AOA_OPT_BETA_BOUND_DEG)
        self.declare_parameter('aoa_opt_loss', AOA_OPT_LOSS)
        self.declare_parameter('aoa_opt_f_scale', AOA_OPT_F_SCALE)
        self.declare_parameter('aoa_opt_max_nfev', AOA_OPT_MAX_NFEV)
        self.declare_parameter(
            'aoa_opt_require_world_origin', AOA_OPT_REQUIRE_WORLD_ORIGIN)

        self.port = self.get_parameter('serial_port').value
        self.baud = int(self.get_parameter('baud').value)
        self.tag_height = self.get_parameter('tag_height').value
        self.alt_std    = self.get_parameter('altitude_std').value
        self.output_frame = self.get_parameter('output_frame').value
        if self.output_frame not in ('heading', 'enu'):
            raise ValueError("output_frame must be 'heading' or 'enu'")
        self.allow_pose_fallback = self.get_parameter('allow_pose_altitude_fallback').value
        self.alt_timeout = self.get_parameter('altitude_timeout').value
        self._pose_timeout = self.get_parameter('pose_timeout').value
        self.origin_mode = self.get_parameter('world_origin_mode').value
        self.base_heading_mode = str(
            self.get_parameter('base_heading_mode').value).strip().lower()
        valid_heading_modes = {'fixed_east', 'odometry_relative', 'imu_relative'}
        if self.base_heading_mode not in valid_heading_modes:
            raise ValueError(
                f"base_heading_mode={self.base_heading_mode!r} is invalid; "
                f"expected one of {sorted(valid_heading_modes)}")
        self.base_odometry_topic = str(
            self.get_parameter('base_odometry_topic').value)
        self.base_imu_topic = str(self.get_parameter('base_imu_topic').value)
        self.base_initial_heading_enu_rad = math.radians(float(
            self.get_parameter('base_initial_heading_enu_deg').value))
        self.base_fixed_yaw_enu_rad = math.radians(float(
            self.get_parameter('base_fixed_yaw_enu_deg').value))
        self.base_heading_timeout = max(0.05, float(
            self.get_parameter('base_heading_timeout').value))
        self.position_sigma_range_m = max(0.0, float(
            self.get_parameter('position_sigma_range_m').value))
        self.position_sigma_azimuth_rad = math.radians(max(0.0, float(
            self.get_parameter('position_sigma_azimuth_deg').value)))
        self.base_heading_sigma_rad = math.radians(max(0.0, float(
            self.get_parameter('base_heading_sigma_deg').value)))
        self.mount_yaw_offset_deg = float(self.get_parameter('aoa_mount_yaw_offset_deg').value)
        self.azimuth_sign = int(self.get_parameter('aoa_azimuth_sign').value)
        if self.azimuth_sign not in (-1, +1):
            self.get_logger().warn(
                f"[AOA] azimuth_sign={self.azimuth_sign} invalid, forcing +1")
            self.azimuth_sign = +1
        self.range_bias  = float(self.get_parameter('aoa_range_bias_m').value)
        self.range_scale = float(self.get_parameter('aoa_range_scale').value)
        if abs(self.range_scale) < 1e-6:
            self.get_logger().warn("[AOA] range_scale too small, forcing 1.0")
            self.range_scale = 1.0
        self.range_alpha = float(self.get_parameter('aoa_range_alpha').value)
        self.angle_alpha = float(self.get_parameter('aoa_angle_alpha').value)
        self.allow_gate_force_reseed = bool(
            self.get_parameter('allow_gate_force_reseed').value)
        self.aoa_opt_enable = bool(
            self.get_parameter('aoa_opt_enable').value)
        if self.aoa_opt_enable and self.base_heading_mode != 'fixed_east':
            self.get_logger().warn(
                "[AOA-OPT] Static-base sliding window disabled in dynamic "
                f"heading mode {self.base_heading_mode!r}")
            self.aoa_opt_enable = False
        self.aoa_opt_require_world_origin = bool(
            self.get_parameter('aoa_opt_require_world_origin').value)
        self.aoa_optimizer = AoaSlidingWindowOptimizer(
            window_size=int(self.get_parameter('aoa_opt_window_size').value),
            min_frames=int(self.get_parameter('aoa_opt_min_frames').value),
            sigma_rho_m=float(
                self.get_parameter('aoa_opt_sigma_rho_m').value),
            sigma_theta_rad=math.radians(float(
                self.get_parameter('aoa_opt_sigma_theta_deg').value)),
            sigma_beta_prior_rad=math.radians(float(
                self.get_parameter('aoa_opt_sigma_beta_prior_deg').value)),
            beta_bound_rad=math.radians(float(
                self.get_parameter('aoa_opt_beta_bound_deg').value)),
            loss=str(self.get_parameter('aoa_opt_loss').value),
            f_scale=float(self.get_parameter('aoa_opt_f_scale').value),
            max_nfev=int(self.get_parameter('aoa_opt_max_nfev').value),
        )

        # ── Tag → RTK lever arm (UAV body FLU, metres) ──────────────
        # Cached as a float64 numpy array so _publish can just rotate
        # it into ENU per-frame.  Fall back to zero vector if the
        # config is malformed so the node does not refuse to start.
        try:
            lever = np.array(
                self.get_parameter('tag_offset_body_m').value,
                dtype=float).reshape(3)
        except Exception:
            self.get_logger().warn(
                "[AOA] tag_offset_body_m is not a 3-vector, "
                "falling back to [0,0,0]")
            lever = np.zeros(3, dtype=float)
        self.tag_offset_body = lever
        self._lever_arm_active = bool(np.linalg.norm(lever) > 1e-6)

        try:
            base_lever = np.array(
                self.get_parameter('base_to_aoa_body_m').value,
                dtype=float).reshape(3)
        except Exception as exc:
            raise ValueError(
                "base_to_aoa_body_m must be [forward, left, up] in metres") \
                from exc
        self.base_to_aoa_body = base_lever
        if not math.isclose(
                float(self.tag_height), float(base_lever[2]), abs_tol=1e-6):
            self.get_logger().warn(
                "[AOA] tag_height is deprecated and differs from "
                "base_to_aoa_body_m[2]; using base_to_aoa_body_m[2]")
        self.tag_height = float(base_lever[2])

        # ── Sensor State (UAV side — identical to UWB node) ─────────
        self.latest_quat: Optional[np.ndarray] = None
        self.latest_altitude: Optional[float] = None
        self.altitude_origin: Optional[float] = None
        self.latest_yaw = 0.0

        # Ground sensor-rig heading.  Dynamic sources are relative-yaw
        # measurements anchored once to base_initial_heading_enu_deg.
        self._base_heading_reference_sensor: Optional[float] = None
        self._base_yaw_enu_rad: Optional[float] = None
        self._last_base_heading_time: Optional[float] = None
        self._base_heading_history: deque = deque(maxlen=400)
        self._base_heading_stale_logged = False
        self._imu_orientation_invalid_logged = False
        if self.base_heading_mode == 'fixed_east':
            self._base_yaw_enu_rad = wrap_angle_rad(
                self.base_fixed_yaw_enu_rad)

        # ── Pose freshness ──────────────────────────────────────────
        self._last_pose_time: Optional[float] = None
        self._pose_stale_logged = False

        # ── Altitude watchdog ───────────────────────────────────────
        self._dedicated_alt_active = False
        self._dedicated_alt_time: float = 0.0
        self._pose_altitude: Optional[float] = None
        self._alt_stale_logged = False

        # ── UAV absolute position (UTM/map) ─────────────────────────
        self.uav_abs: Optional[np.ndarray] = None
        self._uav_state_history = UavStateHistory(maxlen=400)

        # ── World origin ────────────────────────────────────────────
        self.world_origin: Optional[np.ndarray] = None
        self._origin_buffer: list[np.ndarray] = []
        self._origin_last_pos: Optional[np.ndarray] = None
        self._origin_attempt_count = 0

        if self.origin_mode == 'fixed_utm':
            self.world_origin = np.array([
                self.get_parameter('fixed_origin_e').value,
                self.get_parameter('fixed_origin_n').value,
                self.get_parameter('fixed_origin_z').value,
            ])
            self.get_logger().info(
                f"[AOA] World origin (fixed_utm): "
                f"({self.world_origin[0]:.2f}, {self.world_origin[1]:.2f}, "
                f"{self.world_origin[2]:.2f})")

        # ── AOA-specific state ──────────────────────────────────────
        self._range_ema: Optional[float] = None
        self._angle_ux_ema: Optional[float] = None   # cos(theta)
        self._angle_uy_ema: Optional[float] = None   # sin(theta)

        self._last_position_time: Optional[float] = None
        self._last_heartbeat_time: Optional[float] = None
        self._aoa_meas_stale_logged = False
        self._aoa_hb_stale_logged = False
        self._xor_fail_last_report = 0

        self.update_count = 0
        self._last_azimuth_raw_deg: float = 0.0
        self._last_link_status: int = LINK_STATUS_STALE

        # ── Outlier-gate state (angle / range jump, EMA singularity) ──
        self._last_theta_enu_rad: Optional[float] = None
        self._last_theta_time: Optional[float] = None
        self._last_yaw_at_theta: Optional[float] = None
        self._last_range_cal_m: Optional[float] = None
        self._last_range_time: Optional[float] = None

        self._angle_reject_count = 0        # total rejected frames (any gate)
        self._angle_reject_last_report = 0  # throttled log baseline
        self._consec_angle_reject = 0       # current consecutive-reject streak
        self._projection_clamp_count = 0
        self._projection_clamp_last_warn = -1e9

        # ── Queue drop counter (serial thread → main loop backpressure) ──
        self._queue_drop_count = 0
        self._queue_drop_last_report = 0
        self._last_frame_recv_monotonic = -1e30
        self._last_frame_recv_ros_ns = 0

        # ── Front/back disambiguation state ─────────────────────────
        # Coarse ENU prior pointing FROM the locked world_origin (UAV
        # first-fix point) TO the base station.  None disables the
        # whole disambiguation pipeline (legacy behaviour: trust the
        # raw azimuth as-is).  Only direction matters; magnitude is
        # ignored.
        self._hint_dir_enu: Optional[np.ndarray] = None
        if AOA_BASE_HINT_DIR_ENU is not None:
            try:
                hint = np.array(
                    AOA_BASE_HINT_DIR_ENU, dtype=float).reshape(2)
                norm = float(np.linalg.norm(hint))
                if norm < 1e-6:
                    self.get_logger().warn(
                        f"[AOA] AOA_BASE_HINT_DIR_ENU={AOA_BASE_HINT_DIR_ENU!r} "
                        f"has near-zero magnitude, disabling disambiguation")
                else:
                    self._hint_dir_enu = hint / norm
            except Exception:
                self.get_logger().warn(
                    f"[AOA] AOA_BASE_HINT_DIR_ENU={AOA_BASE_HINT_DIR_ENU!r} "
                    f"is not a 2-vector, disabling disambiguation")
        # 0 = keep raw theta, 1 = flip by π.  Sticky across frames so
        # a single noisy sample cannot toggle the published branch.
        self._disambig_last_choice: int = 0
        self._disambig_consec_other: int = 0
        self._disambig_switch_count: int = 0
        self._latest_base_odom_pose: Optional[np.ndarray] = None
        self._solution_base_abs: Optional[np.ndarray] = None
        self._solution_base_odom_pose: Optional[np.ndarray] = None

        # ── Subscribers (identical topics to UWB node) ──────────────
        self.sub_pose = self.create_subscription(
            PoseStamped, '/uav/utm_pose', self._pose_cb, 10)
        self.sub_alt = self.create_subscription(
            Float64, '/uav/altitude_agl', self._alt_cb, 10)
        self.sub_base_heading = None
        if self.base_heading_mode == 'odometry_relative':
            self.sub_base_heading = self.create_subscription(
                Odometry, self.base_odometry_topic,
                self._base_odometry_cb, 20)
        elif self.base_heading_mode == 'imu_relative':
            self.sub_base_heading = self.create_subscription(
                Imu, self.base_imu_topic, self._base_imu_cb,
                qos_profile_sensor_data)

        # ── Publishers (identical topics to UWB node) ───────────────
        self.pub_relative = self.create_publisher(
            PoseStamped, '/ground_station/relative_pose', 10)
        self.pub_debug = self.create_publisher(
            Float32MultiArray, '/ground_station/debug', 10)
        self.pub_abs_utm = self.create_publisher(
            PoseStamped, '/ground_station/absolute_pose_utm', 10)
        self.pub_abs_local = self.create_publisher(
            PoseStamped, '/ground_station/absolute_pose_local', 10)
        self.pub_abs_utm_cov = self.create_publisher(
            PoseWithCovarianceStamped,
            '/ground_station/absolute_pose_utm_cov', 10)
        self.pub_abs_local_cov = self.create_publisher(
            PoseWithCovarianceStamped,
            '/ground_station/absolute_pose_local_cov', 10)
        self.pub_abs_opt_utm = self.create_publisher(
            PoseWithCovarianceStamped,
            '/ground_station/absolute_pose_optimized_utm',
            10)
        self.pub_abs_opt_local = self.create_publisher(
            PoseWithCovarianceStamped,
            '/ground_station/absolute_pose_optimized_local',
            10)
        self.pub_uav_local = self.create_publisher(
            PoseStamped, '/uav/local_pose', 10)

        # Raw AOA observation channel.  Published once per 0x2001
        # frame, BEFORE any gate logic runs, so downstream debug
        # tooling can see the raw bearing/range even when the gates
        # are rejecting frames.  The field layout is:
        #   [0]  distance_raw_m          (wire SLANT range, cm→m)
        #   [1]  distance_cal_m          (same slant range after bias / scale)
        #   [2]  azimuth_raw_deg         (wire azimuth, int16)
        #   [3]  azimuth_body_deg        (after sign + mount offset;
        #                                 0=rig forward, +90=rig left)
        #   [4]  elevation_raw_deg       (wire elevation, int16)
        #   [5]  tag_status              (opaque, float-cast)
        #   [6]  seq                     (0x2001 sequence id)
        #   [7]  anchor_id
        #   [8]  gate_code               (GATE_* enum, see source)
        #   [9]  link_status             (0/1/2, last known)
        #   [10] base_yaw_enu_deg         (NaN until heading is valid)
        #   [11] azimuth_enu_deg          (NaN until heading is valid)
        self.pub_aoa_meas = self.create_publisher(
            Float32MultiArray, '/aoa/measurement', 10)
        self.pub_aoa_observation = self.create_publisher(
            AoaObservation, '/aoa/observation', 10)

        # ── AOA frame queue + serial thread ─────────────────────────
        self._frame_queue: deque = deque(maxlen=256)
        self._queue_lock = threading.Lock()
        self._parser = AoaFrameParser()
        self._serial_conn: Optional[serial.Serial] = None
        self._serial_running = True
        self._serial_thread = threading.Thread(
            target=self._serial_loop, name='aoa_serial', daemon=True)
        self._serial_thread.start()

        # ── 50 Hz drain + watchdog timer ────────────────────────────
        self._timer = self.create_timer(0.02, self._on_timer)

        self.get_logger().info(
            f"[AOA] Started | port={self.port}@{self.baud} "
            f"mount_off={self.mount_yaw_offset_deg:+.1f}° "
            f"sign={self.azimuth_sign:+d} "
            f"range_scale={self.range_scale:.3f} bias={self.range_bias:+.3f}m "
            f"frame={self.output_frame} origin={self.origin_mode}")
        self.get_logger().info(
            "[AOA] mode=base_on_ground | "
            f"heading_mode={self.base_heading_mode} | "
            f"initial_enu={math.degrees(self.base_initial_heading_enu_rad):+.1f}° | "
            f"fixed_enu={math.degrees(self.base_fixed_yaw_enu_rad):+.1f}°")
        if self.base_heading_mode == 'odometry_relative':
            self.get_logger().info(
                f"[AOA] Waiting for fused ground heading on "
                f"{self.base_odometry_topic}")
        elif self.base_heading_mode == 'imu_relative':
            self.get_logger().warn(
                f"[AOA] IMU-relative heading on {self.base_imu_topic}: "
                "relative yaw only; no magnetometer means global yaw must be "
                "initialized and drift is expected")
        self.get_logger().info(
            "[AOA] relative_pose  = UAV relative to base (p_uav_rel, "
            f"output_frame={self.output_frame})")
        self.get_logger().info(
            "[AOA] absolute_pose_utm / _local = SOLVED base position "
            "(uav_abs − p_uav_rel)")
        if self.aoa_opt_enable:
            self.get_logger().info(
                "[AOA-OPT] Sliding-window SciPy backend ENABLED: "
                f"window={self.aoa_optimizer.window_size}, "
                f"min_frames={self.aoa_optimizer.min_frames}, "
                f"loss={self.aoa_optimizer.loss}, "
                f"require_origin={self.aoa_opt_require_world_origin}")
        else:
            self.get_logger().info(
                "[AOA-OPT] Sliding-window SciPy backend DISABLED")
        if self._lever_arm_active:
            L = self.tag_offset_body
            self.get_logger().info(
                f"[AOA] Tag lever arm (body FLU, m): "
                f"fwd={L[0]:+.3f}, left={L[1]:+.3f}, up={L[2]:+.3f} "
                f"(rotated into ENU per-frame using FULL 3-axis UAV "
                f"attitude; pitch/roll FRD→FLU sign flip applied "
                f"locally)")
        else:
            self.get_logger().info(
                "[AOA] Tag lever arm: DISABLED (AOA_TAG_OFFSET_BODY_M ≈ 0); "
                "tag assumed co-located with RTK antenna")
        Lg = self.base_to_aoa_body
        self.get_logger().info(
            f"[AOA] Ground base_link->AOA lever arm (rig FLU, m): "
            f"fwd={Lg[0]:+.3f}, left={Lg[1]:+.3f}, up={Lg[2]:+.3f}")
        if self._hint_dir_enu is not None:
            hint_az_deg = math.degrees(math.atan2(
                self._hint_dir_enu[1], self._hint_dir_enu[0]))
            self.get_logger().info(
                f"[AOA] Front/back disambig: ENABLED  "
                f"hint_dir_enu=({self._hint_dir_enu[0]:+.3f}E, "
                f"{self._hint_dir_enu[1]:+.3f}N) "
                f"≈ {hint_az_deg:+.1f}° (0°=East, 90°=North)  "
                f"hysteresis={AOA_DISAMBIG_HYSTERESIS_FRAMES} frames, "
                f"margin={AOA_DISAMBIG_MARGIN_M:.2f} m, "
                f"min_geom_elev={AOA_MIN_GEOM_ELEVATION_DEG:.1f}°")
        else:
            self.get_logger().info(
                "[AOA] Front/back disambig: DISABLED  "
                "(AOA_BASE_HINT_DIR_ENU is None)  "
                f"min_geom_elev={AOA_MIN_GEOM_ELEVATION_DEG:.1f}°")
        self.get_logger().warn(
            "[AOA] CALIBRATION: tag on rig forward axis -> "
            "measurement[3] near 0 deg; tag on rig left -> near +90 deg. "
            "Then verify measurement[10] base yaw and measurement[11] ENU "
            "bearing against a surveyed/map direction.")

    # ==============================================================
    #  World origin stability logic  (copied from UWB node, verbatim)
    # ==============================================================
    def _try_lock_origin(self, pos: np.ndarray):
        """Lock world_origin after ORIGIN_STABLE_FRAMES frames that satisfy
        both pairwise (adjacent jump) and whole-window (scatter radius) checks."""
        if self._origin_last_pos is not None:
            jump = float(np.linalg.norm(pos[:2] - self._origin_last_pos[:2]))
            if jump > ORIGIN_PAIRWISE_THRESH:
                self._origin_buffer.clear()
        self._origin_last_pos = pos.copy()
        self._origin_buffer.append(pos.copy())

        self._origin_attempt_count += 1
        if self._origin_attempt_count % 100 == 0:
            self.get_logger().warn(
                f"[AOA] Origin not yet locked after "
                f"{self._origin_attempt_count} frames "
                f"(buffer={len(self._origin_buffer)}/{ORIGIN_STABLE_FRAMES})")

        if len(self._origin_buffer) < ORIGIN_STABLE_FRAMES:
            return

        candidate = np.mean(self._origin_buffer, axis=0)
        pts = np.array(self._origin_buffer)
        max_radius = float(np.max(
            np.linalg.norm(pts[:, :2] - candidate[:2], axis=1)))

        if max_radius > ORIGIN_WINDOW_RADIUS:
            self._origin_buffer.pop(0)
            return

        self.world_origin = candidate
        self._origin_buffer.clear()
        self.get_logger().info(
            f"[AOA] World origin locked (first_fix, "
            f"{ORIGIN_STABLE_FRAMES} frames, scatter={max_radius:.3f}m): "
            f"({self.world_origin[0]:.2f}, {self.world_origin[1]:.2f}, "
            f"{self.world_origin[2]:.2f})")

    # ==============================================================
    #  Callbacks  (copied from UWB node, verbatim except log tags)
    # ==============================================================
    def _pose_cb(self, msg: PoseStamped):
        """Receive UAV pose → quaternion + altitude + absolute position."""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        receive_monotonic = time.monotonic()
        self._last_pose_time = now_sec
        self._pose_stale_logged = False

        self.latest_quat = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ])

        self.uav_abs = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])

        # Origin stability check (first_fix mode)
        if self.origin_mode == 'first_fix' and self.world_origin is None:
            self._try_lock_origin(self.uav_abs)

        # Publish /uav/local_pose when origin is available
        if self.world_origin is not None:
            uav_local = self.uav_abs - self.world_origin
            uav_local_msg = PoseStamped()
            uav_local_msg.header.stamp = msg.header.stamp
            uav_local_msg.header.frame_id = 'local_origin'
            uav_local_msg.pose.position.x = float(uav_local[0])
            uav_local_msg.pose.position.y = float(uav_local[1])
            uav_local_msg.pose.position.z = float(uav_local[2])
            uav_local_msg.pose.orientation = msg.pose.orientation
            self.pub_uav_local.publish(uav_local_msg)

        # Pose-derived altitude (always compute, used as fallback)
        pose_z = msg.pose.position.z
        if self.altitude_origin is None:
            self.altitude_origin = pose_z
            self.get_logger().info(
                f"[AOA] Altitude origin set: {pose_z:.2f} m")
        self._pose_altitude = pose_z - self.altitude_origin
        self._uav_state_history.append(
            receive_monotonic, self.uav_abs, self.latest_quat,
            self._pose_altitude)

        # Altitude source selection with watchdog
        if self._dedicated_alt_active:
            age = now_sec - self._dedicated_alt_time
            if age > self.alt_timeout:
                self._dedicated_alt_active = False
                if not self._alt_stale_logged:
                    if self.allow_pose_fallback:
                        self.get_logger().warn(
                            f"[AOA] Dedicated altitude stale ({age:.1f}s), "
                            f"falling back to pose altitude")
                    else:
                        self.get_logger().warn(
                            f"[AOA] Dedicated altitude stale ({age:.1f}s), "
                            f"updates paused (pose altitude fallback disabled)")
                    self._alt_stale_logged = True
                if not self.allow_pose_fallback:
                    self.latest_altitude = None

        if self._dedicated_alt_active:
            pass  # latest_altitude already set by _alt_cb
        elif self.allow_pose_fallback:
            self.latest_altitude = self._pose_altitude
        # else: latest_altitude already set to None above when stale fired

    def _alt_cb(self, msg: Float64):
        """Receive dedicated AGL altitude (mmWave radar etc.)."""
        self.latest_altitude = msg.data
        self._dedicated_alt_time = self.get_clock().now().nanoseconds / 1e9
        self._alt_stale_logged = False
        if not self._dedicated_alt_active:
            self._dedicated_alt_active = True
            self.get_logger().info(
                "[AOA] Dedicated altitude source active")

    @staticmethod
    def _yaw_from_quaternion_xyzw(quaternion: np.ndarray) -> Optional[float]:
        """Return ROS yaw from a finite, non-zero quaternion."""
        q = np.asarray(quaternion, dtype=float).reshape(4)
        norm = float(np.linalg.norm(q))
        if not np.all(np.isfinite(q)) or norm < 1e-9:
            return None
        return float(Rotation.from_quat(q / norm).as_euler('ZYX')[0])

    def _update_base_heading(self, sensor_yaw_rad: float) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if self._base_heading_reference_sensor is None:
            self._base_heading_reference_sensor = sensor_yaw_rad
            self.get_logger().info(
                "[AOA] Ground heading reference locked: "
                f"sensor={math.degrees(sensor_yaw_rad):+.2f} deg -> "
                f"ENU={math.degrees(self.base_initial_heading_enu_rad):+.2f} deg")

        relative_yaw = wrap_angle_rad(
            sensor_yaw_rad - self._base_heading_reference_sensor)
        self._base_yaw_enu_rad = wrap_angle_rad(
            self.base_initial_heading_enu_rad + relative_yaw)
        self._last_base_heading_time = now
        self._base_heading_history.append(
            (time.monotonic(), self._base_yaw_enu_rad))
        if self._base_heading_stale_logged:
            self.get_logger().info("[AOA] Ground heading source recovered")
            self._base_heading_stale_logged = False

    def _base_odometry_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = self._yaw_from_quaternion_xyzw(np.array([
            q.x, q.y, q.z, q.w], dtype=float))
        if yaw is not None:
            self._latest_base_odom_pose = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                yaw,
            ], dtype=float)
            self._update_base_heading(yaw)

    def _base_imu_cb(self, msg: Imu) -> None:
        # REP-145: covariance[0] == -1 means orientation is unavailable.
        if msg.orientation_covariance[0] < 0.0:
            if not self._imu_orientation_invalid_logged:
                self.get_logger().warn(
                    "[AOA] IMU orientation is unavailable (covariance[0] < 0). "
                    "Raw /camera/gyro_accel/sample cannot provide heading; "
                    "use fused odometry or /camera/imu/data from a filter.")
                self._imu_orientation_invalid_logged = True
            return
        q = msg.orientation
        yaw = self._yaw_from_quaternion_xyzw(np.array([
            q.x, q.y, q.z, q.w], dtype=float))
        if yaw is not None:
            self._imu_orientation_invalid_logged = False
            self._update_base_heading(yaw)

    def _base_heading_is_fresh(self, now: float) -> bool:
        if self.base_heading_mode == 'fixed_east':
            return self._base_yaw_enu_rad is not None
        return (
            self._base_yaw_enu_rad is not None
            and self._last_base_heading_time is not None
            and now - self._last_base_heading_time
            <= self.base_heading_timeout)

    def _base_yaw_for_frame(self, frame: dict) -> Optional[float]:
        """Interpolate ground yaw at the AOA frame's serial receive time."""
        if self.base_heading_mode == 'fixed_east':
            return self._base_yaw_enu_rad
        if not self._base_heading_history:
            return None

        target = float(frame.get('_recv_time', time.monotonic()))
        history = list(self._base_heading_history)
        if target <= history[0][0]:
            return float(history[0][1])
        if target >= history[-1][0]:
            return float(history[-1][1])

        for (t0, yaw0), (t1, yaw1) in zip(history, history[1:]):
            if t0 <= target <= t1:
                if t1 - t0 < 1e-9:
                    return float(yaw1)
                alpha = (target - t0) / (t1 - t0)
                return wrap_angle_rad(
                    yaw0 + alpha * wrap_angle_rad(yaw1 - yaw0))
        return float(history[-1][1])

    # ==============================================================
    #  AOA serial loop  (background thread)
    # ==============================================================
    def _serial_loop(self):
        """Open the AOA UART, read chunks, push parsed frames to queue."""
        while self._serial_running:
            try:
                if self._serial_conn is None:
                    try:
                        self._serial_conn = serial.Serial(
                            self.port, self.baud, timeout=AOA_TIMEOUT)
                        self.get_logger().info(
                            f"[AOA] Serial opened: {self.port} @ {self.baud}")
                    except Exception as e:
                        self.get_logger().error(
                            f"[AOA] Failed to open {self.port}: {e}")
                        time.sleep(2.0)
                        continue

                read_start_monotonic = time.monotonic()
                read_start_ros_ns = self.get_clock().now().nanoseconds
                data = self._serial_conn.read(1024)
                if not data:
                    continue

                frames = self._parser.feed(data)
                if not frames:
                    continue

                recv_time = time.monotonic()
                recv_ros_ns = self.get_clock().now().nanoseconds
                count = len(frames)
                for index, fr in enumerate(frames):
                    alpha = float(index + 1) / float(count)
                    frame_monotonic = (
                        read_start_monotonic
                        + alpha * (recv_time - read_start_monotonic))
                    frame_ros_ns = int(
                        read_start_ros_ns
                        + alpha * (recv_ros_ns - read_start_ros_ns))
                    frame_monotonic = max(
                        frame_monotonic,
                        self._last_frame_recv_monotonic + 1e-6)
                    frame_ros_ns = max(
                        frame_ros_ns, self._last_frame_recv_ros_ns + 1)
                    fr['_recv_time'] = frame_monotonic
                    fr['_recv_ros_ns'] = frame_ros_ns
                    self._last_frame_recv_monotonic = frame_monotonic
                    self._last_frame_recv_ros_ns = frame_ros_ns

                with self._queue_lock:
                    # deque(maxlen=N) silently drops from the left when
                    # full.  Count the drop *before* extending so we can
                    # surface upstream backpressure in a log line.
                    max_len = self._frame_queue.maxlen or 0
                    projected = len(self._frame_queue) + len(frames)
                    if max_len and projected > max_len:
                        self._queue_drop_count += projected - max_len
                    self._frame_queue.extend(frames)

            except Exception as e:
                self.get_logger().error(f"[AOA] Serial loop error: {e}")
                try:
                    if self._serial_conn is not None:
                        self._serial_conn.close()
                except Exception:
                    pass
                self._serial_conn = None
                time.sleep(1.0)

    # ==============================================================
    #  Timer: drain queue + watchdog
    # ==============================================================
    def _on_timer(self):
        # ── 1. Drain frames ────────────────────────────────────────
        with self._queue_lock:
            frames = list(self._frame_queue)
            self._frame_queue.clear()

        for f in frames:
            cmd = f.get('cmd')
            if cmd == CMD_POSITION:
                self._on_position_frame(f)
            elif cmd == CMD_HEARTBEAT:
                self._on_heartbeat_frame(f)

        # ── 2. Watchdogs ───────────────────────────────────────────
        self._check_watchdogs()

    # ==============================================================
    #  AOA-specific: heartbeat / position / watchdog
    # ==============================================================
    def _on_heartbeat_frame(self, f: dict):
        now = self.get_clock().now().nanoseconds / 1e9
        self._last_heartbeat_time = now
        if self._aoa_hb_stale_logged:
            self.get_logger().info(
                f"[AOA] Heartbeat recovered (anchor_id={f.get('anchor_id')})")
            self._aoa_hb_stale_logged = False

    # ------------------------------------------------------------------
    @staticmethod
    def _wrap_angle_rad(a: float) -> float:
        """Backward-compatible wrapper around the pure geometry helper."""
        return wrap_angle_rad(a)

    def _tag_offset_enu(self, rot: Rotation) -> np.ndarray:
        """Rotate the configured Tag→RTK lever arm from body FLU to ENU
        using the FULL 3-axis UAV attitude (yaw + pitch + roll).

        The quaternion cached in /uav/utm_pose is a *hybrid* rotation
        built by uav_gps_node.py as

            R_hybrid = R.from_euler('ZYX',
                [enu_yaw,  pitch_ned/frd,  roll_ned/frd])

        i.e. yaw is already corrected NED→ENU, but pitch and roll still
        carry the MAVLink NED/FRD sign convention.  Applying that
        rotation directly to an FLU-defined lever arm would mis-sign the
        pitch/roll contribution and, for a sub-meter lever arm at
        moderate bank angles, this is easily a ~0.1–0.2 m error in XY.

        To recover the correct ENU←FLU rotation we therefore decompose
        the hybrid rotation back to Euler ZYX and FLIP the signs of
        pitch and roll LOCALLY here.  This keeps uav_gps_node.py (and
        hence the UWB pipeline) untouched while giving AOA a
        mathematically sound lever-arm compensation.

        For hover (pitch≈roll≈0) this collapses to the original
        yaw-only formula exactly.
        """
        if not self._lever_arm_active:
            return np.zeros(3, dtype=float)

        yaw, pitch_mav, roll_mav = rot.as_euler('ZYX')
        r_enu_from_flu = Rotation.from_euler(
            'ZYX', [yaw, -pitch_mav, -roll_mav])
        return r_enu_from_flu.apply(self.tag_offset_body)

    def _publish_aoa_measurement(self, f: dict,
                                 distance_cal_m: float,
                                 azimuth_body_deg: float,
                                 gate_code: int) -> None:
        """Emit a raw observation on /aoa/measurement.

        Called for every 0x2001 frame (accepted or rejected) so that
        downstream debug tools see the uncorrupted wire data plus the
        gate verdict applied to it.
        """
        base_yaw_deg = float('nan')
        azimuth_enu_deg = float('nan')
        frame_base_yaw = self._base_yaw_for_frame(f)
        if frame_base_yaw is not None:
            base_yaw_deg = math.degrees(frame_base_yaw)
            azimuth_enu_deg = math.degrees(body_bearing_to_enu(
                math.radians(azimuth_body_deg), frame_base_yaw))
        msg = Float32MultiArray()
        msg.data = [
            float(f['distance_m']),             # [0] distance_raw_m (slant)
            float(distance_cal_m),              # [1] distance_cal_m (slant)
            float(f['azimuth_raw_deg']),        # [2] azimuth_raw_deg
            float(azimuth_body_deg),            # [3] azimuth_body_deg
            float(f['elevation_raw_deg']),      # [4] elevation_raw_deg
            float(f['tag_status']),             # [5] tag_status
            float(f['seq']),                    # [6] seq
            float(f['anchor_id']),              # [7] anchor_id
            float(gate_code),                   # [8] gate_code
            float(self._last_link_status),      # [9] link_status
            float(base_yaw_deg),                # [10] ground base yaw in ENU
            float(azimuth_enu_deg),              # [11] AOA azimuth in ENU
        ]
        self.pub_aoa_meas.publish(msg)

        observation = AoaObservation()
        receive_ns = int(f.get(
            '_recv_ros_ns', self.get_clock().now().nanoseconds))
        observation.header.stamp = Time(nanoseconds=receive_ns).to_msg()
        observation.header.frame_id = 'sensor_rig'
        observation.range_raw_m = float(f['distance_m'])
        observation.range_calibrated_m = float(distance_cal_m)
        observation.azimuth_raw_rad = math.radians(float(f['azimuth_raw_deg']))
        observation.azimuth_body_rad = math.radians(float(azimuth_body_deg))
        observation.elevation_raw_rad = math.radians(
            float(f['elevation_raw_deg']))
        observation.tag_status = int(f['tag_status']) & 0xFFFFFFFF
        observation.sequence = int(f['seq']) & 0xFFFFFFFF
        observation.anchor_id = int(f['anchor_id']) & 0xFFFFFFFF
        observation.gate_code = int(gate_code) & 0xFF
        observation.link_status = int(self._last_link_status) & 0xFF
        observation.base_yaw_enu_rad = (
            float(frame_base_yaw) if frame_base_yaw is not None
            else float('nan'))
        observation.azimuth_enu_rad = (
            body_bearing_to_enu(
                math.radians(azimuth_body_deg), frame_base_yaw)
            if frame_base_yaw is not None else float('nan'))
        self.pub_aoa_observation.publish(observation)

    def _predicted_base_abs_from_odometry(
            self, base_yaw_enu_rad: float) -> Optional[np.ndarray]:
        if (self._solution_base_abs is None
                or self._solution_base_odom_pose is None
                or self._latest_base_odom_pose is None):
            return None
        delta_odom = (
            self._latest_base_odom_pose[:2]
            - self._solution_base_odom_pose[:2])
        yaw_offset = wrap_angle_rad(
            base_yaw_enu_rad - self._latest_base_odom_pose[2])
        delta_enu = rotate_body_xy_to_enu(
            np.array([delta_odom[0], delta_odom[1], 0.0]), yaw_offset)
        prediction = self._solution_base_abs.copy()
        prediction[:2] += delta_enu[:2]
        return prediction

    def _pick_disambiguated_theta(self,
                                  theta_enu_rad: float,
                                  r_slant_m: float,
                                  dz_tag: float,
                                  tag_offset_enu: np.ndarray,
                                  base_yaw_enu_rad: float,
                                  uav_abs: np.ndarray) -> float:
        """Choose between theta and theta+pi.

        Once initialized, temporal ENU continuity is the strongest safe cue for
        a moving ground base.  The fixed coarse spatial hint is used only to
        seed the branch before an accepted history sample exists.
        """
        theta_a = wrap_angle_rad(theta_enu_rad)
        theta_b = wrap_angle_rad(theta_enu_rad + math.pi)
        candidates = (theta_a, theta_b)

        predicted_base_abs = self._predicted_base_abs_from_odometry(
            base_yaw_enu_rad)
        if predicted_base_abs is not None:
            rho_sq = max(r_slant_m * r_slant_m - dz_tag * dz_tag, 0.0)
            rho_tag = math.sqrt(rho_sq)
            candidate_positions = []
            for theta in candidates:
                candidate_abs, _ = solve_ground_base_position(
                    uav_rtk_abs_enu=uav_abs,
                    tag_offset_enu=tag_offset_enu,
                    horizontal_range_m=rho_tag,
                    vertical_tag_minus_aoa_m=dz_tag,
                    bearing_enu_rad=theta,
                    base_to_aoa_body_m=self.base_to_aoa_body,
                    base_yaw_enu_rad=base_yaw_enu_rad,
                )
                candidate_positions.append(candidate_abs)
            distances = [
                float(np.linalg.norm(candidate[:2] - predicted_base_abs[:2]))
                for candidate in candidate_positions]
            choice = 0 if distances[0] <= distances[1] else 1
            if choice != self._disambig_last_choice:
                self._disambig_switch_count += 1
            self._disambig_last_choice = choice
            self._disambig_consec_other = 0
            return candidates[choice]

        if self._last_theta_enu_rad is not None:
            distances = [abs(wrap_angle_rad(
                theta - self._last_theta_enu_rad)) for theta in candidates]
            choice = 0 if distances[0] <= distances[1] else 1
            if choice != self._disambig_last_choice:
                self._disambig_switch_count += 1
            self._disambig_last_choice = choice
            self._disambig_consec_other = 0
            return candidates[choice]

        if (self._hint_dir_enu is None
                or self.world_origin is None
                or uav_abs is None):
            return theta_a

        rho_sq = r_slant_m * r_slant_m - dz_tag * dz_tag
        if rho_sq < 0.0:
            rho_sq = 0.0
        rho_tag = math.sqrt(rho_sq)

        scores: list[float] = []
        for theta in candidates:
            base_abs, _ = solve_ground_base_position(
                uav_rtk_abs_enu=uav_abs,
                tag_offset_enu=tag_offset_enu,
                horizontal_range_m=rho_tag,
                vertical_tag_minus_aoa_m=dz_tag,
                bearing_enu_rad=theta,
                base_to_aoa_body_m=self.base_to_aoa_body,
                base_yaw_enu_rad=base_yaw_enu_rad,
            )
            score = float(np.dot(
                (base_abs - self.world_origin)[:2],
                self._hint_dir_enu))
            scores.append(score)

        choice = 0 if scores[0] >= scores[1] else 1
        if choice != self._disambig_last_choice:
            self._disambig_consec_other += 1
            score_old = scores[self._disambig_last_choice]
            score_new = scores[choice]
            if (self._disambig_consec_other >= AOA_DISAMBIG_HYSTERESIS_FRAMES
                    and (score_new - score_old) >= AOA_DISAMBIG_MARGIN_M):
                self._disambig_last_choice = choice
                self._disambig_consec_other = 0
                self._disambig_switch_count += 1
            else:
                choice = self._disambig_last_choice
        else:
            self._disambig_consec_other = 0

        return candidates[choice]

    def _on_position_frame(self, f: dict):
        """Convert a 0x2001 frame into a ground base pose and publish.

        Geometry (base on ground, tag on UAV):
            theta_enu = theta_body + base_yaw_enu
            dz_tag    = altitude_uav + Lz − base_to_aoa_z
            rho_tag   = sqrt(max(r_slant² − dz_tag², 0))
            p_tag_rel = [rho_tag·cos(theta_enu),
                         rho_tag·sin(theta_enu), dz_tag]

        where R_enu←flu is the full 3-axis UAV attitude rotation
        (yaw ψ + pitch θ + roll φ), not just yaw.  See _tag_offset_enu
        for how it is reconstructed from the hybrid quaternion.

        In other words: AOA gives us a slant observation to the TAG, we
        project it onto the horizontal plane, and only then subtract the
        rotated Tag→RTK lever arm so the published vector still refers to
        the UAV/RTK reference point.

        Frame flow:
          1. UAV pose / altitude freshness
          2. range calibration + physics gate
          3. body bearing -> ENU bearing with fresh base heading
          4. range-rate gate
          5. geometric elevation gate
          6. front/back disambiguation (theta vs theta+pi)
          7. ENU angle-rate gate
          8. tentative EMA step + singularity gate
          9. commit EMA, compute p_uav_rel, publish

        The EMA is never updated from a rejected frame.  If the gate
        rejects AOA_ANGLE_CONSEC_REJECT_RESET frames in a row the EMA
        is re-seeded from the current raw measurement.

        CALIBRATION WARNING — the spec does NOT specify the AOA 0° axis
        direction or the CW/CCW sign convention.  Defaults are
        azimuth_sign=+1 and mount_yaw_offset=0°.  These MUST be
        validated empirically in the body frame before any downstream
        consumer trusts /ground_station/* coordinates.
        """
        now = self.get_clock().now().nanoseconds / 1e9
        frame_time = float(f.get('_recv_time', time.monotonic()))

        # Compute these up-front so /aoa/measurement always reflects
        # what the gate saw, even on reject.
        r_raw = float(f['distance_m'])
        r_cal = (r_raw - self.range_bias) / self.range_scale
        azimuth_raw_deg = float(f['azimuth_raw_deg'])
        azimuth_body_deg = (
            self.azimuth_sign * azimuth_raw_deg + self.mount_yaw_offset_deg
        )
        azimuth_body_deg = ((azimuth_body_deg + 180.0) % 360.0) - 180.0
        self._last_azimuth_raw_deg = azimuth_raw_deg

        # ── Pose must exist and be fresh ───────────────────────────
        if self._last_pose_time is not None:
            pose_age = now - self._last_pose_time
            if pose_age > self._pose_timeout:
                if not self._pose_stale_logged:
                    self.get_logger().warn(
                        f"[AOA] UAV pose stale ({pose_age:.1f}s > "
                        f"{self._pose_timeout}s), updates paused")
                    self._pose_stale_logged = True
                self._publish_aoa_measurement(
                    f, r_cal, azimuth_body_deg, GATE_POSE_STALE)
                return

        uav_state = self._uav_state_history.sample(
            frame_time, max_extrapolation_s=self._pose_timeout)
        if uav_state is None:
            self._publish_aoa_measurement(
                f, r_cal, azimuth_body_deg, GATE_NO_STATE)
            return
        uav_abs = uav_state.position
        uav_quaternion = uav_state.quaternion_xyzw
        frame_altitude = (
            self.latest_altitude
            if self._dedicated_alt_active and self.latest_altitude is not None
            else uav_state.pose_altitude_m
        )

        # A body-frame AOA bearing cannot be used for global localization
        # until the ground sensor-rig heading is known and fresh.
        if not self._base_heading_is_fresh(now):
            if not self._base_heading_stale_logged:
                source_topic = (self.base_odometry_topic
                                if self.base_heading_mode == 'odometry_relative'
                                else self.base_imu_topic)
                self.get_logger().warn(
                    "[AOA] Ground heading unavailable/stale; output paused "
                    f"(mode={self.base_heading_mode}, topic={source_topic}, "
                    f"timeout={self.base_heading_timeout:.2f}s)")
                self._base_heading_stale_logged = True
            self._publish_aoa_measurement(
                f, r_cal, azimuth_body_deg, GATE_BASE_HEADING_STALE)
            return

        frame_base_yaw = self._base_yaw_for_frame(f)
        if frame_base_yaw is None:
            self._publish_aoa_measurement(
                f, r_cal, azimuth_body_deg, GATE_BASE_HEADING_STALE)
            return
        base_yaw_enu_rad = float(frame_base_yaw)

        # ── 1. Physics gate on range ───────────────────────────────
        if not (AOA_MIN_RANGE <= r_cal <= AOA_MAX_RANGE):
            self._publish_aoa_measurement(
                f, r_cal, azimuth_body_deg, GATE_RANGE_PHYSICS)
            return

        # ── 2. Attitude from current quaternion ────────────────────
        # UAV attitude is NOT applied to the AOA bearing itself.  We
        # still need the FULL 3-axis attitude to rotate the optional
        # Tag→RTK lever arm into ENU (yaw rotates the horizontal
        # component, pitch/roll rotate the vertical component into
        # horizontal when the UAV banks).  Yaw is also exposed on
        # debug[8] for diagnostics.
        rot = Rotation.from_quat(uav_quaternion)
        yaw, _pitch, _roll = rot.as_euler('ZYX')
        theta_body_rad = math.radians(azimuth_body_deg)
        theta_enu_rad = body_bearing_to_enu(
            theta_body_rad, base_yaw_enu_rad)

        # ── 3. Range-rate gate ────────────────────────────────────
        if (self._last_range_cal_m is not None
                and self._last_range_time is not None):
            dt = frame_time - self._last_range_time
            if 0.0 < dt <= AOA_ANGLE_JUMP_DT_MAX:
                dr = abs(r_cal - self._last_range_cal_m)
                if dr / dt > AOA_RANGE_JUMP_MAX_MPS:
                    self._angle_reject_count += 1
                    self._consec_angle_reject += 1
                    self._publish_aoa_measurement(
                        f, r_cal, azimuth_body_deg, GATE_RANGE_RATE)
                    self._maybe_force_reset(
                        r_cal, theta_enu_rad, yaw, frame_time,
                        reason="range-rate")
                    return

        # ── 4. Geometric elevation gate ────────────────────────────
        # The horizontal projection rho = sqrt(r² − dz²) collapses to
        # zero as dz approaches r, so the AOA bearing becomes useless
        # for distinguishing "in front" vs "behind" at low geometric
        # elevation.  We compute the elevation implied by raw geometry
        # (NOT by the EMA, so a stuck filter cannot mask the issue)
        # and reject the frame outright if it falls below the
        # configured threshold.  Set AOA_MIN_GEOM_ELEVATION_DEG = 0.0
        # to disable.
        #
        # tag_offset_enu and altitude are reused in step 6 below;
        # both depend only on quantities that are already known at
        # this point (rot from the current frame's quat, and
        # latest_altitude validated above).
        tag_offset_enu = self._tag_offset_enu(rot)
        altitude = max(frame_altitude, 0.1)   # guard ≤ 0
        dz_tent = (
            altitude + tag_offset_enu[2] - self.base_to_aoa_body[2])
        rho_sq_t = r_cal * r_cal - dz_tent * dz_tent
        if AOA_MIN_GEOM_ELEVATION_DEG > 0.0 and rho_sq_t > 0.0:
            elev_geom_deg = math.degrees(
                math.atan2(abs(dz_tent), math.sqrt(rho_sq_t)))
            if elev_geom_deg < AOA_MIN_GEOM_ELEVATION_DEG:
                self._angle_reject_count += 1
                self._publish_aoa_measurement(
                    f, r_cal, azimuth_body_deg, GATE_LOW_ELEVATION)
                return

        # ── 5. Front/back disambiguation ──────────────────────────
        # AOA modules with only one azimuth channel cannot distinguish
        # θ from θ+π.  We resolve this by picking the branch whose
        # implied base position projects most positively onto a
        # user-supplied coarse prior direction (AOA_BASE_HINT_DIR_ENU,
        # measured from the world_origin / first-fix point).  A
        # hysteresis mechanism prevents single-frame noise from
        # toggling the choice.
        theta_enu_rad = self._pick_disambiguated_theta(
            theta_enu_rad, r_cal, dz_tent, tag_offset_enu,
            base_yaw_enu_rad, uav_abs)
        theta_body_rad = wrap_angle_rad(
            theta_enu_rad - base_yaw_enu_rad)
        azimuth_body_deg = math.degrees(theta_body_rad)

        # ── 6. ENU angle-rate gate ─────────────────────────────────
        # Branch selection must happen first; otherwise a valid mirrored raw
        # sample is rejected as a 180-degree jump before it can be corrected.
        if (self._last_theta_enu_rad is not None
                and self._last_theta_time is not None):
            dt = frame_time - self._last_theta_time
            if 0.0 < dt <= AOA_ANGLE_JUMP_DT_MAX:
                d_theta = wrap_angle_rad(
                    theta_enu_rad - self._last_theta_enu_rad)
                residual_deg = abs(math.degrees(d_theta))
                max_allowed_deg = (
                    AOA_ANGLE_JUMP_MAX_RATE_DPS * dt
                    + AOA_ANGLE_JUMP_SLACK_DEG
                )
                if residual_deg > max_allowed_deg:
                    self._angle_reject_count += 1
                    self._consec_angle_reject += 1
                    self._publish_aoa_measurement(
                        f, r_cal, azimuth_body_deg, GATE_ANGLE_RATE)
                    self._maybe_force_reset(
                        r_cal, theta_enu_rad, yaw, frame_time,
                        reason="angle-rate")
                    return

        # ── 7. Tentative EMA + singularity gate ────────────────────
        ux_meas = math.cos(theta_enu_rad)
        uy_meas = math.sin(theta_enu_rad)

        if (self._angle_ux_ema is None
                or self._angle_uy_ema is None
                or self._range_ema is None):
            tentative_ux = ux_meas
            tentative_uy = uy_meas
            tentative_r  = r_cal
        else:
            a_r = self.range_alpha
            a_a = self.angle_alpha
            tentative_r  = a_r * r_cal + (1.0 - a_r) * self._range_ema
            tentative_ux = a_a * ux_meas + (1.0 - a_a) * self._angle_ux_ema
            tentative_uy = a_a * uy_meas + (1.0 - a_a) * self._angle_uy_ema

        mag = math.hypot(tentative_ux, tentative_uy)
        if mag < AOA_ANGLE_EMA_MIN_MAG:
            # The EMA just got fed a near-opposite angle; the
            # reconstructed bearing would be numerically meaningless.
            # Drop this frame without corrupting state.
            self._angle_reject_count += 1
            self._consec_angle_reject += 1
            self._publish_aoa_measurement(
                f, r_cal, azimuth_body_deg, GATE_EMA_SINGULARITY)
            self._maybe_force_reset(
                r_cal, theta_enu_rad, yaw, frame_time,
                reason="EMA-singular")
            return

        # ── 8. Commit EMA and publish ──────────────────────────────
        self._range_ema    = tentative_r
        self._angle_ux_ema = tentative_ux
        self._angle_uy_ema = tentative_uy

        theta_smooth_rad = math.atan2(
            self._angle_uy_ema, self._angle_ux_ema)
        r_smooth = self._range_ema

        # Convert the smoothed base→Tag slant observation into a
        # horizontal projection, then subtract the rotated UAV-side
        # Tag→RTK lever arm so the published vector continues to refer
        # to the UAV/RTK reference point.  The lever arm is rotated
        # using the FULL 3-axis UAV attitude (see _tag_offset_enu).
        tag_offset_enu = self._tag_offset_enu(rot)

        altitude = max(frame_altitude, 0.1)   # guard against bogus ≤0
        dz_tag = (
            altitude + tag_offset_enu[2] - self.base_to_aoa_body[2])
        rho_sq = r_smooth * r_smooth - dz_tag * dz_tag
        if rho_sq < 0.0:
            self._projection_clamp_count += 1
            if now - self._projection_clamp_last_warn > 2.0:
                self.get_logger().warn(
                    "[AOA] Horizontal projection clamped: "
                    f"r^2 - dz_tag^2 = {rho_sq:.4f} < 0 "
                    f"(r_slant={r_smooth:.3f} m, dz_tag={dz_tag:+.3f} m). "
                    "Rejecting measurement. Check altitude source, "
                    "base_to_aoa_body_m, and AOA_TAG_OFFSET_BODY_M.")
                self._projection_clamp_last_warn = now
            self._publish_aoa_measurement(
                f, r_cal, azimuth_body_deg, GATE_RANGE_PHYSICS)
            return
        rho_tag = math.sqrt(rho_sq)
        rho_opt_sq = r_cal * r_cal - dz_tag * dz_tag
        rho_opt = math.sqrt(rho_opt_sq) if rho_opt_sq >= 0.0 else None

        base_abs, p_uav_rel = solve_ground_base_position(
            uav_rtk_abs_enu=uav_abs,
            tag_offset_enu=tag_offset_enu,
            horizontal_range_m=rho_tag,
            vertical_tag_minus_aoa_m=dz_tag,
            bearing_enu_rad=theta_smooth_rad,
            base_to_aoa_body_m=self.base_to_aoa_body,
            base_yaw_enu_rad=base_yaw_enu_rad,
        )

        self.latest_yaw = yaw   # kept as diagnostic only

        # Track freshness & publish
        self._last_position_time = now
        if self._aoa_meas_stale_logged:
            self.get_logger().info("[AOA] Position frames recovered")
            self._aoa_meas_stale_logged = False

        # Gate-state history (only updated on accepted frames so that
        # a long streak of bad samples cannot poison the reference
        # point used by the rate gates).  _last_yaw_at_theta is no
        # longer consulted by the angle-rate gate but we continue to
        # update it so log / debug tooling sees UAV yaw at the time
        # of the last accepted AOA frame.
        self._last_theta_enu_rad  = theta_enu_rad
        self._last_theta_time     = frame_time
        self._last_yaw_at_theta   = yaw
        self._last_range_cal_m    = r_cal
        self._last_range_time     = frame_time
        self._consec_angle_reject = 0
        self._last_link_status    = LINK_STATUS_POSITION_OK

        self._publish_aoa_measurement(
            f, r_cal, azimuth_body_deg, GATE_OK)

        self.update_count += 1
        self._update_sliding_window_optimizer(
            rho_m=rho_opt,
            bearing_rad=theta_enu_rad,
            tag_offset_enu=tag_offset_enu,
            uav_abs=uav_abs,
        )
        self._solution_base_abs = base_abs.copy()
        if self._latest_base_odom_pose is not None:
            self._solution_base_odom_pose = self._latest_base_odom_pose.copy()
        self._publish(
            p_uav_rel=p_uav_rel,
            base_abs=base_abs,
            base_yaw_enu_rad=base_yaw_enu_rad,
            uav_yaw_diag=yaw,
            horizontal_range_m=rho_tag,
            stamp=Time(nanoseconds=int(f.get(
                '_recv_ros_ns', self.get_clock().now().nanoseconds))).to_msg(),
        )

    # ------------------------------------------------------------------
    def _maybe_force_reset(self,
                           r_cal: float,
                           theta_enu_rad: float,
                           yaw: float,
                           now: float,
                           reason: str) -> None:
        """After AOA_ANGLE_CONSEC_REJECT_RESET consecutive rejects we
        assume the filter state is genuinely stale and re-seed it
        from the current raw measurement.  This prevents permanent
        lockout when the target really did make a large jump (e.g.
        brief occlusion followed by reappearance on the other side).
        """
        if not self.allow_gate_force_reseed:
            return
        if self._consec_angle_reject < AOA_ANGLE_CONSEC_REJECT_RESET:
            return
        self.get_logger().warn(
            f"[AOA] {self._consec_angle_reject} consecutive rejects "
            f"(last={reason}); re-seeding EMA from current frame")
        self._range_ema    = r_cal
        self._angle_ux_ema = math.cos(theta_enu_rad)
        self._angle_uy_ema = math.sin(theta_enu_rad)
        self._last_theta_enu_rad = theta_enu_rad
        self._last_theta_time     = now
        self._last_yaw_at_theta   = yaw
        self._last_range_cal_m    = r_cal
        self._last_range_time     = now
        self._consec_angle_reject = 0
        # Deliberately do NOT set _last_position_time: the force-reset
        # only reclaims filter state, it does not promise the upper
        # layer that a valid position just appeared.

    def _update_sliding_window_optimizer(self,
                                         rho_m: Optional[float],
                                         bearing_rad: float,
                                         tag_offset_enu: np.ndarray,
                                         uav_abs: np.ndarray) -> None:
        """Feed one accepted raw AOA observation to the SciPy backend."""
        if not self.aoa_opt_enable:
            return
        if rho_m is None or not math.isfinite(rho_m):
            return
        if self.world_origin is None and self.aoa_opt_require_world_origin:
            return

        origin = (self.world_origin if self.world_origin is not None
                  else np.zeros(3, dtype=float))
        tag_abs = uav_abs + tag_offset_enu
        # The legacy optimizer models tag = base + rho*bearing.  Shift
        # the tag by the known fixed base_link->AOA lever arm so its state
        # remains the base_link position rather than the antenna position.
        base_to_aoa_enu = rotate_body_xy_to_enu(
            self.base_to_aoa_body, float(self._base_yaw_enu_rad))
        tag_local = tag_abs - base_to_aoa_enu - origin
        tag_xy_local = tag_local[:2]

        self.aoa_optimizer.add_frame(
            tag_xy_local=tag_xy_local,
            rho_m=float(rho_m),
            bearing_rad=float(bearing_rad),
        )

        init_xy = np.array([
            tag_xy_local[0] - rho_m * math.cos(bearing_rad),
            tag_xy_local[1] - rho_m * math.sin(bearing_rad),
        ], dtype=float)
        result = self.aoa_optimizer.optimize(init_xy=init_xy)
        if result is None:
            return

        self._publish_optimized_pose(
            xy_local=result["xy"],
            azimuth_bias_rad=result["azimuth_bias_rad"],
            cost=result["cost"],
            n_frames=result["n_frames"],
        )

    @staticmethod
    def _optimized_covariance() -> list[float]:
        cov = [0.0] * 36
        cov[0] = 0.25     # x variance
        cov[7] = 0.25     # y variance
        cov[14] = 1.0     # z variance
        cov[35] = 999.0   # yaw unknown
        return cov

    def _publish_optimized_pose(self,
                                xy_local: np.ndarray,
                                azimuth_bias_rad: float,
                                cost: float,
                                n_frames: int) -> None:
        """Publish optimized base position on comparison-only topics."""
        stamp = self.get_clock().now().to_msg()
        z_local = 0.0

        local_msg = PoseWithCovarianceStamped()
        local_msg.header.stamp = stamp
        local_msg.header.frame_id = 'local_origin'
        local_msg.pose.pose.position.x = float(xy_local[0])
        local_msg.pose.pose.position.y = float(xy_local[1])
        local_msg.pose.pose.position.z = z_local
        local_msg.pose.pose.orientation.w = 1.0
        local_msg.pose.covariance = self._optimized_covariance()
        self.pub_abs_opt_local.publish(local_msg)

        if self.world_origin is not None:
            xy_abs = xy_local + self.world_origin[:2]
            utm_msg = PoseWithCovarianceStamped()
            utm_msg.header.stamp = stamp
            utm_msg.header.frame_id = 'utm'
            utm_msg.pose.pose.position.x = float(xy_abs[0])
            utm_msg.pose.pose.position.y = float(xy_abs[1])
            utm_msg.pose.pose.position.z = float(self.world_origin[2] + z_local)
            utm_msg.pose.pose.orientation.w = 1.0
            utm_msg.pose.covariance = self._optimized_covariance()
            self.pub_abs_opt_utm.publish(utm_msg)

        if self.update_count % 50 == 0:
            self.get_logger().info(
                f"[AOA-OPT] local=({xy_local[0]:+.2f}E, "
                f"{xy_local[1]:+.2f}N, {z_local:+.2f}U) m | "
                f"az_bias={math.degrees(azimuth_bias_rad):+.2f}° | "
                f"cost={cost:.3f} | N={n_frames}")

    def _check_watchdogs(self):
        now = self.get_clock().now().nanoseconds / 1e9

        # --- Position frame watchdog ---
        pos_fresh = False
        if self._last_position_time is not None:
            age = now - self._last_position_time
            pos_fresh = age <= AOA_MEAS_TIMEOUT
            if not pos_fresh and not self._aoa_meas_stale_logged:
                self.get_logger().warn(
                    f"[AOA] Position frames stale ({age:.1f}s > "
                    f"{AOA_MEAS_TIMEOUT}s), upper-layer output paused")
                self._aoa_meas_stale_logged = True

        # --- Heartbeat watchdog ---
        hb_fresh = False
        if self._last_heartbeat_time is not None:
            hb_age = now - self._last_heartbeat_time
            hb_fresh = hb_age <= AOA_HEARTBEAT_TIMEOUT
            if not hb_fresh and not self._aoa_hb_stale_logged:
                self.get_logger().warn(
                    f"[AOA] Heartbeat stale ({hb_age:.1f}s > "
                    f"{AOA_HEARTBEAT_TIMEOUT}s), base-station link may be down")
                self._aoa_hb_stale_logged = True

        # --- Link status for debug[10] ---
        if pos_fresh:
            self._last_link_status = LINK_STATUS_POSITION_OK
        elif hb_fresh:
            self._last_link_status = LINK_STATUS_HEARTBEAT
        else:
            self._last_link_status = LINK_STATUS_STALE

        # --- Throttled XOR failure log ---
        delta = self._parser.xor_fail_count - self._xor_fail_last_report
        if delta >= 50:
            self.get_logger().warn(
                f"[AOA] XOR failures: +{delta} "
                f"(total={self._parser.xor_fail_count}, "
                f"bad_len={self._parser.bad_length_count}, "
                f"resync={self._parser.resync_count})")
            self._xor_fail_last_report = self._parser.xor_fail_count

        # --- Throttled outlier-gate log ---
        gate_delta = self._angle_reject_count - self._angle_reject_last_report
        if gate_delta >= 20:
            self.get_logger().warn(
                f"[AOA] outlier gate rejected +{gate_delta} frames "
                f"(total={self._angle_reject_count}, "
                f"streak={self._consec_angle_reject})")
            self._angle_reject_last_report = self._angle_reject_count

        # --- Throttled queue drop log ---
        q_delta = self._queue_drop_count - self._queue_drop_last_report
        if q_delta >= 10:
            self.get_logger().warn(
                f"[AOA] serial→timer queue dropped +{q_delta} frames "
                f"(total={self._queue_drop_count}); "
                f"consider raising timer rate or maxlen")
            self._queue_drop_last_report = self._queue_drop_count

    # ==============================================================
    #  Publishing
    # ==============================================================
    #
    #  /ground_station/relative_pose        = p_uav_rel (UAV relative
    #                                           to base, ENU or body).
    #  /ground_station/absolute_pose_utm    = solved base_link in UTM.
    #  /ground_station/absolute_pose_local  = base_abs − world_origin.
    #  /ground_station/debug[0..2]          = p_uav_rel in ENU.
    # ==============================================================
    def _single_measurement_covariance(
            self, horizontal_range_m: float) -> tuple[list[float], float]:
        angle_sigma = math.hypot(
            self.position_sigma_azimuth_rad,
            self.base_heading_sigma_rad)
        sigma_xy = math.hypot(
            self.position_sigma_range_m,
            max(0.0, horizontal_range_m) * angle_sigma)
        covariance = [0.0] * 36
        covariance[0] = sigma_xy * sigma_xy
        covariance[7] = sigma_xy * sigma_xy
        covariance[14] = max(float(self.alt_std), 0.01) ** 2
        covariance[21] = 999.0
        covariance[28] = 999.0
        covariance[35] = max(self.base_heading_sigma_rad, 1e-3) ** 2
        return covariance, sigma_xy

    @staticmethod
    def _set_pose_values(pose, position: np.ndarray, yaw_rad: float) -> None:
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose.orientation.w = math.cos(yaw_rad / 2.0)

    def _publish(self,
                 p_uav_rel: np.ndarray,
                 base_abs: np.ndarray,
                 base_yaw_enu_rad: float,
                 uav_yaw_diag: float,
                 horizontal_range_m: float,
                 stamp):
        covariance, sigma_xy = self._single_measurement_covariance(
            horizontal_range_m)

        # ── Relative Pose (UAV relative to base) ───────────────────
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        if self.output_frame == 'enu':
            pose_msg.header.frame_id = 'enu_relative'
            relative_output = p_uav_rel
        else:
            pose_msg.header.frame_id = 'base_link_leveled'
            relative_output = rotate_body_xy_to_enu(
                p_uav_rel, -base_yaw_enu_rad)
        pose_msg.pose.position.x = float(relative_output[0])
        pose_msg.pose.position.y = float(relative_output[1])
        pose_msg.pose.position.z = float(relative_output[2])
        pose_msg.pose.orientation.w = 1.0
        self.pub_relative.publish(pose_msg)

        # ── Debug ──────────────────────────────────────────────────
        # Slot order is unchanged from the UWB-era contract so that
        # localization_debug_viewer.py / sensor_monitor.py still work,
        # but the SEMANTICS of [0..2] have flipped:
        #   old:  x_fwd/y_left/z_up of "GS relative to UAV"
        #   new:  x_east/y_north/z_up of "UAV relative to base"
        # debug[4] is the ENU azimuth (0°=East, +90°=North).
        dist_2d = float(np.hypot(p_uav_rel[0], p_uav_rel[1]))
        bearing = float(np.degrees(np.arctan2(p_uav_rel[1], p_uav_rel[0])))

        debug = Float32MultiArray()
        debug.data = [
            float(p_uav_rel[0]),                # [0] x_east  (UAV − base)
            float(p_uav_rel[1]),                # [1] y_north (UAV − base)
            float(p_uav_rel[2]),                # [2] z_up    (UAV − base)
            dist_2d,                            # [3] horizontal projected distance
            bearing,                            # [4] ENU azimuth of UAV (deg)
            float(sigma_xy),                    # [5] sigma_x
            float(sigma_xy),                    # [6] sigma_y
            float(max(self.alt_std, 0.01)),      # [7] sigma_z
            float(uav_yaw_diag),                # [8] UAV yaw (rad, ENU) — diagnostic only
            float(self._last_azimuth_raw_deg),  # [9] AOA raw azimuth (deg)
            float(self._last_link_status),      # [10] link status (0/1/2)
            float(base_yaw_enu_rad),            # [11] ground base yaw (rad, ENU)
        ]
        self.pub_debug.publish(debug)

        if self.update_count % 50 == 0:
            if self._hint_dir_enu is None:
                disambig_state = "OFF"
            elif self.world_origin is None:
                disambig_state = "WAIT_ORIGIN"
            else:
                disambig_state = "B" if self._disambig_last_choice else "A"
            self.get_logger().info(
                f"[AOA] base→UAV rel "
                f"({p_uav_rel[0]:+.2f}E, {p_uav_rel[1]:+.2f}N, "
                f"{p_uav_rel[2]:+.2f}U) m | "
                f"Dist {dist_2d:.2f} m | "
                f"Bear {bearing:+.1f}° ENU | "
                f"base_yaw {math.degrees(base_yaw_enu_rad):+.1f}° | "
                f"raw_az {self._last_azimuth_raw_deg:+.1f}° | "
                f"link={self._last_link_status} | "
                f"disambig={disambig_state} "
                f"(consec_other={self._disambig_consec_other})")

        # ── Absolute Pose (SOLVED base position) ────────────────────
        abs_utm = PoseStamped()
        abs_utm.header.stamp = stamp
        abs_utm.header.frame_id = 'utm'
        self._set_pose_values(abs_utm.pose, base_abs, base_yaw_enu_rad)
        self.pub_abs_utm.publish(abs_utm)

        abs_utm_cov = PoseWithCovarianceStamped()
        abs_utm_cov.header = abs_utm.header
        self._set_pose_values(
            abs_utm_cov.pose.pose, base_abs, base_yaw_enu_rad)
        abs_utm_cov.pose.covariance = covariance
        self.pub_abs_utm_cov.publish(abs_utm_cov)

        if self.world_origin is not None:
            base_local = base_abs - self.world_origin

            abs_local = PoseStamped()
            abs_local.header.stamp = stamp
            abs_local.header.frame_id = 'local_origin'
            self._set_pose_values(
                abs_local.pose, base_local, base_yaw_enu_rad)
            self.pub_abs_local.publish(abs_local)

            abs_local_cov = PoseWithCovarianceStamped()
            abs_local_cov.header = abs_local.header
            self._set_pose_values(
                abs_local_cov.pose.pose, base_local, base_yaw_enu_rad)
            abs_local_cov.pose.covariance = covariance
            self.pub_abs_local_cov.publish(abs_local_cov)

    # ==============================================================
    #  Shutdown
    # ==============================================================
    def stop(self):
        self._serial_running = False
        try:
            if self._serial_thread.is_alive():
                self._serial_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self._serial_conn is not None:
                self._serial_conn.close()
        except Exception:
            pass


# ==================================================================
#  Entry point
# ==================================================================
def main(args=None):
    # AOA_USE_ELEVATION is intentionally read here so a log line shows
    # whether elevation would contribute (it does not in this test
    # stage, but the flag is exposed in aoa_config for future use).
    rclpy.init(args=args)
    node = AoaLocalizationNode()
    node.get_logger().info(
        f"[AOA] AOA_USE_ELEVATION={AOA_USE_ELEVATION} "
        f"(elevation parsed but not used in fusion)")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Each of these steps is wrapped independently so a failure in
        # one (e.g. the launch script already called rclpy.shutdown())
        # cannot prevent the others from running.  This keeps the
        # shutdown path quiet instead of producing "already shutdown"
        # red tracebacks when the process is reaped by a launcher.
        try:
            node.stop()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
