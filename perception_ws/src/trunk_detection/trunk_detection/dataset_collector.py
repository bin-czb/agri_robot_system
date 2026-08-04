#!/usr/bin/env python3
"""
树干数据集采集节点 - 用于采集 YOLOv8 训练数据
========================================================

功能说明：
  订阅 Orbbec 相机彩色图像话题，按设定时间间隔自动保存图片，
  同时生成符合 YOLOv8 标准的目录结构和 dataset.yaml 配置文件。

一键启动（在 trunk_tracking_ws 目录下）：
  source install/setup.bash
  ros2 launch trunk_detection dataset_collection.launch.py

单独运行（需已启动相机节点）：
  ros2 run trunk_detection dataset_collector

可选参数：
  ros2 run trunk_detection dataset_collector \
    --ros-args \
    -p save_interval:=1.0 \
    -p output_dir:=/home/czb/trunk_dataset \
    -p image_topic:=/camera/color/image_raw \
    -p show_preview:=true

采集完成后的训练流程：
  1. 安装标注工具：pip install labelimg
  2. 启动标注：labelimg ~/trunk_dataset/images/train ~/trunk_dataset/labels/train
  3. 将约 20% 图片移入 val/ 目录（同步移动对应 labels）
  4. 训练：yolo train model=yolov8n.pt data=~/trunk_dataset/dataset.yaml epochs=100
========================================================
"""

import os
import time
import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image


# ── 预览窗口 HUD 绘制参数 ──────────────────────────────────────────
_FONT = cv2.FONT_HERSHEY_SIMPLEX
_GREEN = (0, 220, 0)
_RED = (0, 0, 220)
_YELLOW = (0, 220, 220)
_WHITE = (255, 255, 255)
_OVERLAY_ALPHA = 0.45          # 半透明背景透明度


class DatasetCollector(Node):
    """
    ROS2 节点：从相机话题采集图像并保存为 YOLOv8 训练数据集。

    键盘快捷键（预览窗口获焦时生效）：
      s — 立即保存当前帧（不受间隔限制）
      p — 暂停 / 继续自动采集
      q — 退出节点
    """

    def __init__(self):
        super().__init__('dataset_collector')

        # ── 参数声明 ──────────────────────────────────────────────
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('save_interval', 1.0)          # 秒/张
        self.declare_parameter('output_dir', '~/trunk_dataset')
        self.declare_parameter('show_preview', True)

        self._image_topic   = self.get_parameter('image_topic').value
        self._interval      = float(self.get_parameter('save_interval').value)
        self._output_dir    = os.path.expanduser(
                                self.get_parameter('output_dir').value)
        self._show_preview  = bool(self.get_parameter('show_preview').value)

        # ── 状态变量 ──────────────────────────────────────────────
        self._bridge          = CvBridge()
        self._saved_count     = 0
        self._last_save_time  = 0.0   # 上次自动保存的时间戳（秒）
        self._paused          = False
        self._latest_frame    = None  # 最新收到的 BGR 图像（numpy）

        # ── 初始化目录 & dataset.yaml ─────────────────────────────
        self._train_img_dir  = os.path.join(self._output_dir, 'images', 'train')
        self._val_img_dir    = os.path.join(self._output_dir, 'images', 'val')
        self._train_lbl_dir  = os.path.join(self._output_dir, 'labels', 'train')
        self._val_lbl_dir    = os.path.join(self._output_dir, 'labels', 'val')
        self._setup_directories()

        # ── QoS & 订阅 ────────────────────────────────────────────
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self._image_topic,
                                 self._image_callback, qos)

        # ── 定时器：处理键盘事件（每 30 ms 检测一次） ──────────────
        if self._show_preview:
            self.create_timer(0.03, self._check_keyboard)

        self.get_logger().info(
            f'\n'
            f'  数据集采集节点已启动\n'
            f'  ─────────────────────────────\n'
            f'  订阅话题  : {self._image_topic}\n'
            f'  保存间隔  : {self._interval} 秒/张\n'
            f'  保存目录  : {self._output_dir}\n'
            f'  实时预览  : {"开启" if self._show_preview else "关闭"}\n'
            f'  ─────────────────────────────\n'
            f'  键盘控制（预览窗口需获焦）\n'
            f'    s — 立即保存当前帧\n'
            f'    p — 暂停/继续\n'
            f'    q — 退出\n'
        )

    # ─────────────────────────────────────────────────────────────
    # 初始化
    # ─────────────────────────────────────────────────────────────

    def _setup_directories(self) -> None:
        """创建 YOLOv8 标准目录结构，并生成 dataset.yaml。"""
        for d in [self._train_img_dir, self._val_img_dir,
                  self._train_lbl_dir,  self._val_lbl_dir]:
            os.makedirs(d, exist_ok=True)

        yaml_path = os.path.join(self._output_dir, 'dataset.yaml')
        if not os.path.exists(yaml_path):
            content = (
                f'# YOLOv8 数据集配置文件（自动生成）\n'
                f'# 用法：yolo train model=yolov8n.pt data={yaml_path} epochs=100\n'
                f'\n'
                f'path: {self._output_dir}\n'
                f'train: images/train\n'
                f'val:   images/val\n'
                f'\n'
                f'# 类别定义（根据实际标注修改）\n'
                f'nc: 1\n'
                f'names:\n'
                f'  0: trunk\n'
            )
            with open(yaml_path, 'w', encoding='utf-8') as f:
                f.write(content)
            self.get_logger().info(f'已生成 dataset.yaml: {yaml_path}')

        self.get_logger().info(
            f'数据集目录就绪: {self._output_dir}\n'
            f'  images/train — 采集图片保存位置\n'
            f'  labels/train — 标注文件放置位置（标注后填入）'
        )

    # ─────────────────────────────────────────────────────────────
    # 回调
    # ─────────────────────────────────────────────────────────────

    def _image_callback(self, msg: Image) -> None:
        """接收图像，更新最新帧，并按时间间隔自动保存。"""
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        self._latest_frame = frame

        # 自动定时保存
        if not self._paused:
            now = time.time()
            if now - self._last_save_time >= self._interval:
                self._save_frame(frame)
                self._last_save_time = now

        # 更新预览窗口
        if self._show_preview:
            self._show_frame(frame)

    def _check_keyboard(self) -> None:
        """定时器回调：处理 OpenCV 窗口键盘事件。"""
        if not self._show_preview:
            return
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self._shutdown()
        elif key == ord('s'):
            if self._latest_frame is not None:
                self._save_frame(self._latest_frame, forced=True)
        elif key == ord('p'):
            self._paused = not self._paused
            state = '暂停' if self._paused else '继续'
            self.get_logger().info(f'采集已{state}')

    # ─────────────────────────────────────────────────────────────
    # 保存 & 显示
    # ─────────────────────────────────────────────────────────────

    def _save_frame(self, frame: np.ndarray, forced: bool = False) -> None:
        """将帧保存为 JPEG，写入 images/train/ 目录。"""
        now_str  = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'trunk_{now_str}_{self._saved_count:05d}.jpg'
        save_path = os.path.join(self._train_img_dir, filename)

        success = cv2.imwrite(save_path, frame,
                              [cv2.IMWRITE_JPEG_QUALITY, 95])
        if success:
            self._saved_count += 1
            tag = '[手动]' if forced else '[自动]'
            self.get_logger().info(
                f'{tag} 保存第 {self._saved_count} 张: {filename}'
            )
        else:
            self.get_logger().error(f'图片保存失败: {save_path}')

    def _show_frame(self, frame: np.ndarray) -> None:
        """在预览窗口上叠加状态信息后显示。"""
        display = frame.copy()
        h, w    = display.shape[:2]

        # 半透明状态栏背景
        overlay = display.copy()
        cv2.rectangle(overlay, (0, 0), (w, 90), (30, 30, 30), -1)
        cv2.addWeighted(overlay, _OVERLAY_ALPHA,
                        display, 1 - _OVERLAY_ALPHA, 0, display)

        # 状态文字
        status_color = _RED if self._paused else _GREEN
        status_text  = 'PAUSED' if self._paused else 'COLLECTING'

        elapsed = time.time() - self._last_save_time
        next_in = max(0.0, self._interval - elapsed)

        cv2.putText(display, f'Status : {status_text}',
                    (10, 25), _FONT, 0.65, status_color, 2)
        cv2.putText(display, f'Saved  : {self._saved_count}  |  '
                             f'Next in: {next_in:.1f}s  |  '
                             f'Interval: {self._interval:.1f}s',
                    (10, 55), _FONT, 0.55, _WHITE, 1)
        cv2.putText(display, '[s] Save now   [p] Pause/Resume   [q] Quit',
                    (10, 80), _FONT, 0.45, _YELLOW, 1)

        cv2.imshow('Dataset Collector — trunk', display)

    # ─────────────────────────────────────────────────────────────
    # 退出
    # ─────────────────────────────────────────────────────────────

    def _shutdown(self) -> None:
        """打印汇总信息并关闭节点。"""
        self.get_logger().info(
            f'\n'
            f'  采集结束汇总\n'
            f'  ─────────────────────────────\n'
            f'  共保存图片 : {self._saved_count} 张\n'
            f'  保存位置   : {self._train_img_dir}\n'
            f'  dataset.yaml: {os.path.join(self._output_dir, "dataset.yaml")}\n'
            f'  ─────────────────────────────\n'
            f'  后续步骤：\n'
            f'    1. labelimg {self._train_img_dir} {self._train_lbl_dir}\n'
            f'    2. 将 ~20% 图片移至 images/val/ 并同步 labels/val/\n'
            f'    3. yolo train model=yolov8n.pt '
            f'data={os.path.join(self._output_dir, "dataset.yaml")} '
            f'epochs=100\n'
        )
        if self._show_preview:
            cv2.destroyAllWindows()
        rclpy.shutdown()


# ─────────────────────────────────────────────────────────────────
# 入口
# ─────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = DatasetCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._shutdown()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        if node._show_preview:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
