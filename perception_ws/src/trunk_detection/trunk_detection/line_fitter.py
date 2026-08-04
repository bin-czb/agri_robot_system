#!/usr/bin/env python3
"""
LineFitter - 绝对左侧优先版
只要左侧有线，就强制选左侧，除非左侧完全没线。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from trunk_interfaces.msg import TrunkDetection, TrunkLine

class LineFitter(Node):

    def __init__(self):
        super().__init__('line_fitter')
        
        self.declare_parameter('detection_topic', '/trunk_detection/detections')
        self.declare_parameter('trunk_line_topic', '/trunk_detection/trunk_line')
        self.declare_parameter('viz_topic', '/trunk_detection/line_viz')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('center_offset_pixels', 180)
        self.declare_parameter('k_nearest', 5)
        
        self.det_topic = self.get_parameter('detection_topic').value
        self.line_topic = self.get_parameter('trunk_line_topic').value
        self.viz_topic = self.get_parameter('viz_topic').value
        self.img_topic = self.get_parameter('image_topic').value
        self.offset = self.get_parameter('center_offset_pixels').value
        self.k_nearest = self.get_parameter('k_nearest').value
        
        self.cv_bridge = CvBridge()
        self.latest_image = None
        self.latest_vel = None
        self.last_side = None
        
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(TrunkDetection, self.det_topic, self.det_cb, qos)
        self.create_subscription(Image, self.img_topic, self.img_cb, qos)
        self.create_subscription(Twist, '/cmd_vel', self.vel_cb, qos)
        
        self.pub_line = self.create_publisher(TrunkLine, self.line_topic, qos)
        self.pub_viz = self.create_publisher(Image, self.viz_topic, qos)
        
        self.get_logger().info("LineFitter Priority: LEFT.")

    def vel_cb(self, msg):
        self.latest_vel = (msg.linear.x, msg.angular.z)

    def img_cb(self, msg):
        try: self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except: pass

    def get_nearest_points(self, pts, h, k):
        if len(pts) == 0: return []
        sorted_pts = sorted(pts, key=lambda p: p[1], reverse=True)
        return np.array(sorted_pts[:k])

    def fit(self, pts):
        if len(pts) < 2: return None
        try:
            x = pts[:, 0]
            y = pts[:, 1]
            k, b = np.polyfit(y, x, 1)
            # 放宽一切限制，只要能算出来就行
            return (k, b)
        except: return None

    def det_cb(self, msg):
        # 1. Viz
        if self.latest_image is not None: viz = self.latest_image.copy()
        else: viz = np.zeros((480, 640, 3), dtype=np.uint8)
        h, w = viz.shape[:2]
        cx = w / 2.0
        
        # 2. Collect & Split
        raw_L, raw_R = [], []
        for p in msg.detection_points:
            cv2.circle(viz, (int(p.x), int(p.y)), 3, (0, 255, 255), -1) 
            if p.x < cx: raw_L.append([p.x, p.y])
            else: raw_R.append([p.x, p.y])
            
        # 3. Filter Nearest
        pts_L = self.get_nearest_points(raw_L, h, self.k_nearest)
        pts_R = self.get_nearest_points(raw_R, h, self.k_nearest)
        
        # Viz Fit Points
        for p in pts_L: cv2.circle(viz, (int(p[0]), int(p[1])), 6, (0, 0, 255), 1)
        for p in pts_R: cv2.circle(viz, (int(p[0]), int(p[1])), 6, (255, 0, 0), 1)
        
        # 4. Fit
        line_L = self.fit(pts_L)
        line_R = self.fit(pts_R)
        
        # 5. Logic: 绝对左侧优先
        chosen_side = None
        
        if line_L:
            chosen_side = 'LEFT'
        elif line_R:
            chosen_side = 'RIGHT'
        
        # Fallback (合体拟合)
        if chosen_side is None:
            all_pts = raw_L + raw_R
            nearest_all = self.get_nearest_points(all_pts, h, self.k_nearest * 2)
            line_all = self.fit(nearest_all)
            if line_all:
                k, b = line_all
                x_btm = k * h + b
                if x_btm < cx: chosen_side = 'LEFT'; line_L = line_all
                else: chosen_side = 'RIGHT'; line_R = line_all
        
        self.last_side = chosen_side
        
        # 6. Output
        valid = False
        tgt_k, tgt_b = 0, 0
        
        if chosen_side == 'LEFT' and line_L:
            k, b = line_L
            tgt_k, tgt_b = k, b + self.offset
            valid = True
            cv2.line(viz, (int(b), 0), (int(k*h+b), h), (0,0,255), 2)
            
        elif chosen_side == 'RIGHT' and line_R:
            k, b = line_R
            tgt_k, tgt_b = k, b - self.offset
            valid = True
            cv2.line(viz, (int(b), 0), (int(k*h+b), h), (255,0,0), 2)
            
        # Debug Text
        dbg = f"L:{len(pts_L)}pts " + ("OK" if line_L else "NO") + \
              f" | R:{len(pts_R)}pts " + ("OK" if line_R else "NO")
        cv2.putText(viz, dbg, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        out = TrunkLine()
        out.header = msg.header
        out.line_valid = valid
        
        if valid:
            tgt_x = tgt_k * h + tgt_b
            out.trunk_center_x = float(tgt_x)
            out.trunk_center_y = float(h)
            out.steering_angle_rad = float(-math.atan(tgt_k))
            
            # Hide Green Line, Show Status
            cv2.putText(viz, f"LOCK: {chosen_side}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
        else:
            cv2.putText(viz, f"NO LINE", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

        # Show Speed
        if self.latest_vel:
            v, w_vel = self.latest_vel
            vel_str = f"V: {v:.2f} m/s  W: {w_vel:.2f} rad/s"
            cv2.putText(viz, vel_str, (10, h-20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        self.pub_line.publish(out)
        self.pub_viz.publish(self.cv_bridge.cv2_to_imgmsg(viz, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = LineFitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
