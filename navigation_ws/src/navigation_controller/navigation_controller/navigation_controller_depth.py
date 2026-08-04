#!/usr/bin/env python3
"""
深度增强版导航控制节点 - 备用节点
在原有基于拟合线的角度/距离估计的基础上，优先使用相机深度图进行位置估计，
提高距离估计准确性；当深度不可用时回退到原先基于图像位置的估计逻辑。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import math
import time
from enum import Enum

from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo

from trunk_interfaces.msg import TrunkLine, ChassisControl, NavigationStatus
from trunk_interfaces.srv import SetNavigationMode


class NavigationState(Enum):
    STOPPED = 0
    SEARCHING = 1
    TRACKING = 2
    ARRIVED = 3
    ERROR = 4


class NavigationControllerDepth(Node):
    """深度增强版导航控制节点（备用）"""

    def __init__(self):
        super().__init__('navigation_controller_depth')

        # 参数声明（继承原有参数，并新增深度相关参数）
        self.declare_parameters(
            namespace='',
            parameters=[
                # 运动控制参数
                ('max_linear_velocity', 0.5),
                ('max_angular_velocity', 1.0),
                ('min_linear_velocity', 0.1),
                ('target_distance', 1.0),
                ('min_distance', 0.3),
                ('arrival_threshold', 0.5),

                # 控制参数
                ('distance_scale_factor', 0.1),
                ('angle_proportional_gain', 1.5),
                ('max_steering_angle', 0.785),

                # 安全参数
                ('timeout_threshold', 2.0),
                ('emergency_stop_enabled', True),

                # ROS话题参数
                ('line_topic', '/trunk_detection/trunk_line'),
                ('control_topic', '/chassis_control/cmd'),
                ('status_topic', '/navigation/status'),
                ('twist_topic', '/cmd_vel'),

                # 深度估计相关参数
                ('use_depth_estimation', True),
                ('depth_topic', '/camera/depth/image_raw'),
                ('camera_info_topic', '/camera/depth/camera_info'),
                ('depth_window_size', 5),
                ('depth_min', 0.2),
                ('depth_max', 10.0),
                # 对 16UC1(mm) → 米使用 0.001；如为 32FC1(米) 则设为 1.0
                ('depth_scale_m', 0.001),
            ]
        )

        # 读取参数
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.min_linear_vel = self.get_parameter('min_linear_velocity').value
        self.target_distance = self.get_parameter('target_distance').value
        self.min_distance = self.get_parameter('min_distance').value
        self.arrival_threshold = self.get_parameter('arrival_threshold').value
        self.distance_scale = self.get_parameter('distance_scale_factor').value
        self.angle_gain = self.get_parameter('angle_proportional_gain').value
        self.max_steering = self.get_parameter('max_steering_angle').value
        self.timeout_threshold = self.get_parameter('timeout_threshold').value
        self.emergency_stop = self.get_parameter('emergency_stop_enabled').value

        self.line_topic = self.get_parameter('line_topic').value
        self.control_topic = self.get_parameter('control_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.twist_topic = self.get_parameter('twist_topic').value

        # 深度相关
        self.use_depth = bool(self.get_parameter('use_depth_estimation').value)
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.depth_window_size = int(self.get_parameter('depth_window_size').value)
        self.depth_min = float(self.get_parameter('depth_min').value)
        self.depth_max = float(self.get_parameter('depth_max').value)
        self.depth_scale_m = float(self.get_parameter('depth_scale_m').value)

        # 状态变量
        self.navigation_state = NavigationState.STOPPED
        self.last_valid_detection_time = time.time()
        self.last_line_data = None
        self.control_mode = 0  # 0: 停止, 1: 跟踪
        self.start_time = time.time()
        self.control_commands_sent = 0

        # 深度缓存与内参
        self._last_depth = None  # np.ndarray (float32 meters)
        self._depth_shape = None
        self._fx = self._fy = self._cx = self._cy = None

        # QoS：订阅 BEST_EFFORT，发布到 /cmd_vel 使用 RELIABLE
        sub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub_qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # 订阅与发布
        self.line_subscription = self.create_subscription(
            TrunkLine, self.line_topic, self.line_callback, sub_qos
        )

        self.control_publisher = self.create_publisher(
            ChassisControl, self.control_topic, sub_qos
        )

        self.status_publisher = self.create_publisher(
            NavigationStatus, self.status_topic, sub_qos
        )

        self.twist_publisher = self.create_publisher(
            Twist, self.twist_topic, pub_qos_reliable
        )

        # 深度与内参订阅
        if self.use_depth:
            self.depth_sub = self.create_subscription(
                Image, self.depth_topic, self.depth_image_callback, sub_qos
            )
            self.cam_info_sub = self.create_subscription(
                CameraInfo, self.camera_info_topic, self.camera_info_callback, sub_qos
            )

        # 服务与定时器
        self.mode_service = self.create_service(
            SetNavigationMode, '/navigation/set_mode', self.set_navigation_mode_callback
        )
        self.status_timer = self.create_timer(0.1, self.status_timer_callback)
        self.timeout_timer = self.create_timer(0.2, self.timeout_check_callback)

        self.get_logger().info('深度增强版导航控制节点已启动')

    def camera_info_callback(self, msg: CameraInfo):
        try:
            self._fx = float(msg.k[0])
            self._fy = float(msg.k[4])
            self._cx = float(msg.k[2])
            self._cy = float(msg.k[5])
        except Exception as e:
            self.get_logger().warn(f'解析 CameraInfo 失败: {e}')

    def depth_image_callback(self, msg: Image):
        try:
            if msg.encoding in ('16UC1', 'mono16'):
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                depth_m = arr.astype(np.float32) * self.depth_scale_m
            elif msg.encoding in ('32FC1',):
                depth_m = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            else:
                # 尝试按 16UC1 处理
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                depth_m = arr.astype(np.float32) * self.depth_scale_m

            # 过滤 NaN/Inf
            depth_m = np.where(np.isfinite(depth_m), depth_m, 0.0)
            self._last_depth = depth_m
            self._depth_shape = depth_m.shape
        except Exception as e:
            self.get_logger().warn(f'处理深度图失败: {e}')

    def line_callback(self, msg: TrunkLine):
        try:
            self.last_line_data = msg

            if msg.line_valid and self.control_mode == 1:
                self.last_valid_detection_time = time.time()
                control_cmd = self.calculate_control_command(msg)
                self.publish_control_command(control_cmd)
                self.navigation_state = NavigationState.TRACKING
            elif not msg.line_valid and self.control_mode == 1:
                self.navigation_state = NavigationState.SEARCHING
                self.publish_stop_command()
        except Exception as e:
            self.get_logger().error(f'处理直线数据错误: {e}')
            self.navigation_state = NavigationState.ERROR

    def _estimate_pose_from_depth(self, u_px: float, v_px: float):
        """
        使用深度图窗口中值 + 相机内参反投影，估计相机坐标系下(X,Y,Z)。
        返回: (valid, X, Y, Z, distance, yaw_angle)
        yaw_angle 约等于水平转角 = atan2(X, Z)
        """
        if self._last_depth is None or self._fx is None:
            return (False, 0.0, 0.0, 0.0, 0.0, 0.0)

        h, w = self._depth_shape
        u = int(round(u_px))
        v = int(round(v_px))
        if u < 0 or v < 0 or u >= w or v >= h:
            return (False, 0.0, 0.0, 0.0, 0.0, 0.0)

        ws = max(1, self.depth_window_size // 2)
        u0, u1 = max(0, u - ws), min(w, u + ws + 1)
        v0, v1 = max(0, v - ws), min(h, v + ws + 1)

        roi = self._last_depth[v0:v1, u0:u1]
        roi = roi[(roi > self.depth_min) & (roi < self.depth_max)]
        if roi.size == 0:
            return (False, 0.0, 0.0, 0.0, 0.0, 0.0)

        Z = float(np.median(roi))
        X = (u_px - self._cx) / self._fx * Z
        Y = (v_px - self._cy) / self._fy * Z
        dist = float(math.hypot(X, Z))
        yaw = float(math.atan2(X, Z))
        return (True, X, Y, Z, dist, yaw)

    def calculate_control_command(self, line_msg: TrunkLine):
        try:
            # 从拟合线得到像素中心与转向角
            steering_angle = line_msg.steering_angle_rad
            trunk_center_x = line_msg.trunk_center_x
            trunk_center_y = line_msg.trunk_center_y

            distance_est = None
            angle_from_depth = None

            if self.use_depth:
                valid, X, Y, Z, dist, yaw = self._estimate_pose_from_depth(
                    trunk_center_x, trunk_center_y
                )
                if valid:
                    distance_est = dist
                    angle_from_depth = yaw

            # 角度优先使用深度；否则回退拟合线角度
            if angle_from_depth is not None:
                steering_angle = angle_from_depth

            # 限幅与角速度
            steering_angle = max(-self.max_steering, min(self.max_steering, steering_angle))
            angular_velocity = self.angle_gain * steering_angle
            angular_velocity = max(-self.max_angular_vel, min(self.max_angular_vel, angular_velocity))

            # 距离优先使用深度；否则回退旧法
            if distance_est is None:
                estimated_distance = self.estimate_distance_from_image_position(
                    trunk_center_x, trunk_center_y
                )
            else:
                estimated_distance = distance_est

            linear_velocity = self.calculate_linear_velocity(estimated_distance)

            if estimated_distance < self.arrival_threshold:
                linear_velocity = 0.0
                angular_velocity = self.calculate_final_alignment_angle(line_msg)
                self.navigation_state = NavigationState.ARRIVED

            return {
                'linear_velocity': linear_velocity,
                'angular_velocity': angular_velocity,
                'estimated_distance': estimated_distance,
                'steering_angle': steering_angle
            }

        except Exception as e:
            self.get_logger().error(f'控制指令计算错误: {e}')
            return self.get_stop_command()

    def calculate_linear_velocity(self, distance_to_target: float):
        if distance_to_target <= self.min_distance:
            return 0.0
        normalized_distance = min(distance_to_target / self.target_distance, 1.0)
        linear_velocity = self.max_linear_vel * normalized_distance
        if linear_velocity > 0:
            linear_velocity = max(self.min_linear_vel, linear_velocity)
        return linear_velocity

    def estimate_distance_from_image_position(self, trunk_center_x: float, trunk_center_y: float):
        # 回退：基于图像位置的简化估计
        image_height = 480
        normalized_y = trunk_center_y / image_height
        estimated_distance = self.target_distance * (1.0 - normalized_y)
        return max(0.1, min(10.0, estimated_distance))

    def calculate_final_alignment_angle(self, line_msg: TrunkLine):
        try:
            steering_angle = line_msg.steering_angle_rad
            final_angle = math.pi / 2 - steering_angle
            final_angle = max(-self.max_steering, min(self.max_steering, final_angle))
            return self.angle_gain * final_angle
        except Exception:
            return 0.0

    def get_stop_command(self):
        return {
            'linear_velocity': 0.0,
            'angular_velocity': 0.0,
            'estimated_distance': 0.0,
            'steering_angle': 0.0
        }

    def publish_control_command(self, control_cmd):
        chassis_msg = ChassisControl()
        chassis_msg.header = Header()
        chassis_msg.header.stamp = self.get_clock().now().to_msg()
        chassis_msg.header.frame_id = 'base_link'
        chassis_msg.linear_velocity = control_cmd['linear_velocity']
        chassis_msg.angular_velocity = control_cmd['angular_velocity']
        left_speed, right_speed = self.calculate_track_speeds(
            control_cmd['linear_velocity'], control_cmd['angular_velocity']
        )
        chassis_msg.left_track_speed = left_speed
        chassis_msg.right_track_speed = right_speed
        chassis_msg.control_mode = self.control_mode
        chassis_msg.emergency_stop = False
        chassis_msg.target_distance = control_cmd['estimated_distance']
        chassis_msg.min_distance = self.min_distance
        self.control_publisher.publish(chassis_msg)

        twist_msg = Twist()
        twist_msg.linear.x = control_cmd['linear_velocity']
        twist_msg.angular.z = control_cmd['angular_velocity']
        self.twist_publisher.publish(twist_msg)
        self.control_commands_sent += 1

    def calculate_track_speeds(self, linear_vel: float, angular_vel: float):
        track_width = 0.5
        left_speed = linear_vel - (angular_vel * track_width / 2.0)
        right_speed = linear_vel + (angular_vel * track_width / 2.0)
        max_abs_speed = max(abs(left_speed), abs(right_speed))
        if max_abs_speed > self.max_linear_vel:
            scale = self.max_linear_vel / max_abs_speed
            left_speed *= scale
            right_speed *= scale
        left_speed_normalized = left_speed / self.max_linear_vel
        right_speed_normalized = right_speed / self.max_linear_vel
        return left_speed_normalized, right_speed_normalized

    def publish_stop_command(self):
        stop_cmd = self.get_stop_command()
        self.publish_control_command(stop_cmd)

    def set_navigation_mode_callback(self, request, response):
        try:
            old_mode = self.control_mode
            self.control_mode = request.mode
            if request.mode == 0:
                self.navigation_state = NavigationState.STOPPED
                self.publish_stop_command()
            elif request.mode == 1:
                self.navigation_state = NavigationState.SEARCHING
                self.last_valid_detection_time = time.time()
            response.success = True
            response.message = f'导航模式从 {old_mode} 切换到 {request.mode}'
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f'设置导航模式失败: {e}'
            self.get_logger().error(response.message)
        return response

    def status_timer_callback(self):
        status_msg = NavigationStatus()
        status_msg.header = Header()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.header.frame_id = 'base_link'
        status_msg.navigation_state = self.navigation_state.value
        status_msg.state_description = self.navigation_state.name
        status_msg.target_detected = (
            self.last_line_data is not None and self.last_line_data.line_valid
        )
        if self.last_line_data:
            status_msg.distance_to_target = self.estimate_distance_from_image_position(
                self.last_line_data.trunk_center_x, self.last_line_data.trunk_center_y
            )
            status_msg.angle_to_target = self.last_line_data.steering_angle_rad
        current_time = time.time()
        runtime = current_time - self.start_time
        status_msg.total_runtime.sec = int(runtime)
        status_msg.total_runtime.nanosec = int((runtime % 1) * 1e9)
        if runtime > 0:
            status_msg.control_frequency = self.control_commands_sent / runtime
        status_msg.has_error = (self.navigation_state == NavigationState.ERROR)
        if status_msg.has_error:
            status_msg.error_message = 'Navigation error occurred'
        self.status_publisher.publish(status_msg)

    def timeout_check_callback(self):
        current_time = time.time()
        time_since_detection = current_time - self.last_valid_detection_time
        if (self.control_mode == 1 and time_since_detection > self.timeout_threshold
                and self.navigation_state != NavigationState.STOPPED):
            self.get_logger().warn(
                f'检测超时 ({time_since_detection:.2f}s)，切换到搜索模式'
            )
            self.navigation_state = NavigationState.SEARCHING
            if self.emergency_stop:
                self.publish_stop_command()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = NavigationControllerDepth()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点运行错误: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



