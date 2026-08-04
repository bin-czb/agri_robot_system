#!/usr/bin/env python3
"""
导航控制节点 - Stop-Turn-Go (智能保速版)
1. 强制提速到 1.0 m/s。
2. 丢失目标保持 3.0s (配置为 coasting_time)。
3. Coasting 期间保持原速度，不再强制降速。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import time
from enum import Enum

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from trunk_interfaces.msg import TrunkLine, ChassisControl
from trunk_interfaces.srv import SetNavigationMode

class NavigationState(Enum):
    STOPPED = 0
    SEARCHING = 1
    ALIGNING = 2
    MOVING = 3
    COASTING = 4

class NavigationController(Node):

    def __init__(self):
        super().__init__('navigation_controller')
        
        # 参数
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 1.2)
        self.declare_parameter('lookahead_ratio', 0.5)
        self.declare_parameter('align_threshold_rad', 0.1) 
        self.declare_parameter('move_threshold_rad', 0.05)
        self.declare_parameter('imu_yaw_rate_threshold', 0.15)
        
        self.max_v = self.get_parameter('max_linear_velocity').value
        self.max_w = self.get_parameter('max_angular_velocity').value
        self.ratio = self.get_parameter('lookahead_ratio').value
        self.align_thresh = self.get_parameter('align_threshold_rad').value
        self.move_thresh = self.get_parameter('move_threshold_rad').value
        self.imu_yaw_thresh = self.get_parameter('imu_yaw_rate_threshold').value
        
        self.mode = 0
        self.state = NavigationState.STOPPED
        self.fx = 640.0 / (2.0 * math.tan(math.radians(30)))
        self.last_valid = 0
        
        # 记录最后一次正常行驶的速度
        self.last_moving_v = 0.0

        # 陀螺仪 yaw 速率（由 imu_cb 持续更新）
        self.imu_yaw_rate = 0.0
        
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(TrunkLine, '/trunk_detection/trunk_line', self.line_cb, qos)
        self.create_subscription(Imu, '/camera/gyro', self.imu_cb, qos)
        self.pub_ctrl = self.create_publisher(ChassisControl, '/chassis_control/cmd', qos)
        self.pub_twist = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_service(SetNavigationMode, '/navigation/set_mode', self.mode_cb)
        
        self.get_logger().info(
            f"Smart Coasting Controller. MaxV={self.max_v}, "
            f"ImuYawThresh={self.imu_yaw_thresh} rad/s"
        )

    def mode_cb(self, req, res):
        self.mode = req.mode
        if self.mode == 0: self.stop()
        res.success = True
        res.message = f"Mode: {self.mode}"
        return res

    def stop(self):
        self.send(0, 0)
        self.state = NavigationState.STOPPED

    def imu_cb(self, msg):
        self.imu_yaw_rate = msg.angular_velocity.z

    def line_cb(self, msg):
        if self.mode != 1: return
        
        # 1. 丢失处理 (Coasting)
        if not msg.line_valid:
            if time.time() - self.last_valid < 3.0: # 保持 3 秒
                if abs(self.imu_yaw_rate) > self.imu_yaw_thresh:
                    self.get_logger().warn(
                        f"Coasting 偏航过大 yaw={self.imu_yaw_rate:.3f} rad/s，停车"
                    )
                    self.stop()
                else:
                    # 使用记录的速度 (如果是 0，至少给个 0.2 慢慢走)
                    coast_v = self.last_moving_v if self.last_moving_v > 0.1 else 0.2
                    self.send(coast_v, 0.0)
                    self.state = NavigationState.COASTING
            else:
                self.stop()
            return
            
        self.last_valid = time.time()
        
        # 2. 计算误差
        h = msg.trunk_center_y
        k = -math.tan(msg.steering_angle_rad)
        b = msg.trunk_center_x - k * h
        
        y_tgt = h * self.ratio
        x_tgt = k * y_tgt + b
        
        dx = x_tgt - 320.0
        heading_err = math.atan2(dx, self.fx) 
        
        # 3. 状态切换
        if self.state == NavigationState.COASTING:
            if abs(heading_err) > self.move_thresh: self.state = NavigationState.ALIGNING
            else: self.state = NavigationState.MOVING
            
        elif self.state == NavigationState.MOVING:
            if abs(heading_err) > self.align_thresh: self.state = NavigationState.ALIGNING
        elif self.state == NavigationState.ALIGNING:
            if abs(heading_err) < self.move_thresh: self.state = NavigationState.MOVING
        else: # Init
            if abs(heading_err) > self.move_thresh: self.state = NavigationState.ALIGNING
            else: self.state = NavigationState.MOVING
            
        # 4. 执行
        target_v = 0.0
        target_w = 0.0
        
        if self.state == NavigationState.ALIGNING:
            w = -1.5 * heading_err
            target_w = max(-self.max_w, min(self.max_w, w))
            if abs(target_w) < 0.1: target_w = 0.1 * (1 if target_w > 0 else -1)
            target_v = 0.0
            
        elif self.state == NavigationState.MOVING:
            w_correction = -0.5 * heading_err
            target_w = max(-0.3, min(0.3, w_correction))
            target_v = self.max_v
            
        # 记录这次的速度，供 Coasting 使用
        if target_v > 0:
            self.last_moving_v = target_v
            
        self.send(target_v, target_w)

    def send(self, v, w):
        msg = ChassisControl()
        msg.linear_velocity = float(v)
        msg.angular_velocity = float(w)
        msg.control_mode = self.mode
        
        msg.left_track_speed = max(-1.0, min(1.0, v - w))
        msg.right_track_speed = max(-1.0, min(1.0, v + w))
        
        self.pub_ctrl.publish(msg)
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.pub_twist.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
