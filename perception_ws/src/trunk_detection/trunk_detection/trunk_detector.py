#!/usr/bin/env python3
"""
树干检测节点 - 集成现有YOLO算法
基于原始 tree_follow.py 和 trunk_point.py 代码
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import cv2
import numpy as np
from ultralytics import YOLO
from scipy.optimize import curve_fit
import os
import time
import math
from ament_index_python.packages import get_package_share_directory

# ROS消息类型
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Polygon, Point32, Twist
from cv_bridge import CvBridge

# 自定义消息类型
from trunk_interfaces.msg import TrunkDetection, TrunkLine


class TrunkDetector(Node):
    """树干检测节点"""

    def __init__(self):
        super().__init__('trunk_detector')
        
        # 参数声明
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('max_detections', 10)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detection_topic', '/trunk_detection/detections')
        self.declare_parameter('detection_image_topic', '/trunk_detection/detection_image')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('trunk_line_topic', '/trunk_detection/trunk_line')
        
        # 获取参数
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.max_detections = self.get_parameter('max_detections').value
        self.image_topic = self.get_parameter('image_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.detection_image_topic = self.get_parameter('detection_image_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.trunk_line_topic = self.get_parameter('trunk_line_topic').value
        
        # 初始化
        self.cv_bridge = CvBridge()
        self.model = None
        self.last_detection_time = time.time()
        self.last_cmd_vel = None
        self.last_trunk_line = None
        self.last_trunk_line_time = 0.0
        
        # 加载YOLO模型
        self._load_model()
        
        # QoS设置
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        # 订阅和发布
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile
        )
        
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            qos_profile
        )

        self.trunk_line_subscription = self.create_subscription(
            TrunkLine,
            self.trunk_line_topic,
            self.trunk_line_callback,
            qos_profile
        )
        
        self.detection_publisher = self.create_publisher(
            TrunkDetection,
            self.detection_topic,
            qos_profile
        )

        # 发布带标注的检测图像，便于可视化
        self.annotated_image_publisher = self.create_publisher(
            Image,
            self.detection_image_topic,
            qos_profile
        )
        
        # 状态统计
        self.detection_count = 0
        self.fps_counter = 0
        self.last_fps_time = time.time()
        
        self.get_logger().info(f"树干检测节点已启动，模型: {self.model_path}")
        self.get_logger().info(f"订阅话题: {self.image_topic}, {self.cmd_vel_topic}, {self.trunk_line_topic}")
        self.get_logger().info(f"发布话题: {self.detection_topic}")

    def _load_model(self):
        """加载YOLO模型"""
        try:
            package_model = os.path.join(
                get_package_share_directory('trunk_detection'),
                'models',
                os.path.basename(self.model_path),
            )
            model_paths_to_try = [
                self.model_path,
                os.path.join(os.getcwd(), self.model_path),
                package_model,
            ]
            
            for path in model_paths_to_try:
                if os.path.exists(path):
                    self.get_logger().info(f"尝试加载模型: {path}")
                    self.model = YOLO(path)
                    self.get_logger().info(f"成功加载YOLO模型: {path}")
                    return
            
            # 如果找不到模型文件，使用默认的YOLOv8模型
            self.get_logger().warn(f"找不到模型文件 {self.model_path}，使用默认YOLOv8n模型")
            self.model = YOLO('yolov8n.pt')
            
        except Exception as e:
            self.get_logger().error(f"加载模型失败: {e}")
            raise

    def cmd_vel_callback(self, msg):
        """记录最新的控制指令"""
        self.last_cmd_vel = msg

    def trunk_line_callback(self, msg):
        """记录最新的拟合线"""
        self.last_trunk_line = msg
        self.last_trunk_line_time = time.time()

    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 转换ROS图像消息到OpenCV格式
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 进行目标检测
            detections = self.detect_trunks(cv_image)
            
            # 发布检测结果
            detection_msg = self.create_detection_message(detections, msg.header, cv_image)
            self.detection_publisher.publish(detection_msg)

            # 生成并发布带标注的图像
            annotated = self.draw_annotations(cv_image, detections)
            try:
                annotated_msg = self.cv_bridge.cv2_to_imgmsg(annotated, "bgr8")
                annotated_msg.header = msg.header
                self.annotated_image_publisher.publish(annotated_msg)
            except Exception as e:
                self.get_logger().warn(f"发布标注图像失败: {e}")
            
            # 更新统计信息
            self.update_statistics()
            
        except Exception as e:
            self.get_logger().error(f"图像处理错误: {e}")

    def detect_trunks(self, image):
        """
        使用YOLO模型检测树干
        基于原始代码的get_trunk_info函数
        """
        try:
            # 使用模型进行预测
            results = self.model(image, conf=self.confidence_threshold)
            
            # 提取检测框
            if len(results) > 0 and results[0].boxes is not None:
                boxes = results[0].boxes.xyxy
                confidences = results[0].boxes.conf
                
                detections = []
                for i, (box, conf) in enumerate(zip(boxes.cpu(), confidences.cpu())):
                    if i >= self.max_detections:
                        break
                        
                    x_min, y_min, x_max, y_max = box.numpy()
                    confidence = float(conf.numpy())
                    
                    # 计算底部中点 (基于原始算法)
                    base_point_x = (x_min + x_max) / 2
                    base_point_y = y_max
                    
                    detection = {
                        'bbox': [float(x_min), float(y_min), float(x_max), float(y_max)],
                        'confidence': confidence,
                        'center_point': [float(base_point_x), float(base_point_y)]
                    }
                    detections.append(detection)
                
                return detections
            
        except Exception as e:
            self.get_logger().error(f"YOLO检测错误: {e}")
        
        return []

    def draw_annotations(self, image, detections):
        """在图像上绘制检测框、拟合线、速度数值"""
        annotated = image.copy()
        h, w = annotated.shape[:2]
        
        try:
            # 1. 绘制检测框
            for det in detections:
                x_min, y_min, x_max, y_max = det['bbox']
                conf = det['confidence']
                p1 = (int(x_min), int(y_min))
                p2 = (int(x_max), int(y_max))
                cv2.rectangle(annotated, p1, p2, (0, 255, 0), 2)
                label = f"trunk {conf:.2f}"
                cv2.putText(annotated, label, (int(x_min), max(0, int(y_min) - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # 2. 绘制拟合线 (蓝色)
            # 只有当 trunk_line 消息很新 (< 0.5s) 时才绘制
            if (self.last_trunk_line is not None and 
                self.last_trunk_line.line_valid and 
                time.time() - self.last_trunk_line_time < 0.5):
                
                line = self.last_trunk_line
                # Line equation: ax + by + c = 0
                a, b, c = line.coeff_a, line.coeff_b, line.coeff_c
                
                # Draw from top (y=0) to bottom (y=h)
                p1_y, p2_y = 0, h
                
                # x = (-by - c) / a  (if a != 0, typically for vertical-ish lines we solve for x)
                # But line_fitter solves y = ax + b form mostly, then converts to general.
                # Actually line_fitter usually sends vertical lines.
                # Let's use general form: ax + by + c = 0 => x = (-by - c) / a
                
                # Careful with division by zero
                if abs(a) > 1e-6:
                    p1_x = int((-b * p1_y - c) / a)
                    p2_x = int((-b * p2_y - c) / a)
                    cv2.line(annotated, (p1_x, p1_y), (p2_x, p2_y), (255, 0, 0), 2)
                elif abs(b) > 1e-6:
                    # Horizontal-ish line: y = (-ax - c) / b
                    p1_x, p2_x = 0, w
                    p1_y = int((-a * p1_x - c) / b)
                    p2_y = int((-a * p2_x - c) / b)
                    cv2.line(annotated, (p1_x, p1_y), (p2_x, p2_y), (255, 0, 0), 2)

            # 3. 绘制速度数值 (左下角)
            if self.last_cmd_vel is not None:
                v = self.last_cmd_vel.linear.x
                w_ang = self.last_cmd_vel.angular.z
                
                # 颜色：停止时白，动时黄
                color = (255, 255, 255)
                if abs(v) > 0.01 or abs(w_ang) > 0.01:
                    color = (0, 255, 255)
                
                info = f"V={v:.2f} m/s  W={w_ang:.2f} rad/s"
                cv2.putText(annotated, info, (10, h - 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
        except Exception as e:
            self.get_logger().warn(f"绘制标注失败: {e}")
        return annotated

    def create_detection_message(self, detections, header, image):
        """创建检测结果消息"""
        msg = TrunkDetection()
        msg.header = header
        msg.header.frame_id = "camera_color_optical_frame"  # 根据实际相机frame_id调整
        
        # 图像信息
        msg.image_width = image.shape[1]
        msg.image_height = image.shape[0]
        
        # 检测结果
        msg.trunk_count = len(detections)
        msg.detection_valid = len(detections) > 0
        
        if detections:
            for detection in detections:
                # 检测点(底部中点)
                point = Point()
                point.x = detection['center_point'][0]
                point.y = detection['center_point'][1]
                point.z = 0.0
                msg.detection_points.append(point)
                
                # 置信度
                msg.confidences.append(detection['confidence'])
                
                # 边界框
                bbox = detection['bbox']
                polygon = Polygon()
                # 添加四个顶点
                for x, y in [(bbox[0], bbox[1]), (bbox[2], bbox[1]), 
                            (bbox[2], bbox[3]), (bbox[0], bbox[3])]:
                    point32 = Point32()
                    point32.x = float(x)
                    point32.y = float(y)
                    point32.z = 0.0
                    polygon.points.append(point32)
                msg.bounding_boxes.append(polygon)
        
        return msg

    def update_statistics(self):
        """更新性能统计"""
        self.detection_count += 1
        self.fps_counter += 1
        
        current_time = time.time()
        if current_time - self.last_fps_time >= 1.0:  # 每秒计算一次FPS
            fps = self.fps_counter / (current_time - self.last_fps_time)
            self.get_logger().info(f"检测FPS: {fps:.2f}, 总检测次数: {self.detection_count}")
            self.fps_counter = 0
            self.last_fps_time = current_time


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        trunk_detector = TrunkDetector()
        rclpy.spin(trunk_detector)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点运行错误: {e}")
    finally:
        if 'trunk_detector' in locals():
            trunk_detector.destroy_node()
        # Ctrl+C 时 rclpy 可能已经触发 shutdown；这里二次调用会抛 RCLError
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
