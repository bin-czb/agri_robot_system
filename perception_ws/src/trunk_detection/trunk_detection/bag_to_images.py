import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
import cv2
import numpy as np
from cv_bridge import CvBridge
import os
import json


class BagToImages(Node):
    def __init__(self) -> None:
        super().__init__('bag_to_images')
        self.bridge = CvBridge()
        self.dataset_root = os.path.expanduser('~/trunk_dataset')
        self.create_dirs()

        # Parameters for sampling control
        self.declare_parameter('sampling_mode', 'frame')  # 'frame' or 'time'
        self.declare_parameter('sampling_every_n', 5)      # save 1 per N synchronized pairs
        self.declare_parameter('sampling_interval_s', 0.5) # save 1 pair per T seconds

        # Internal state for sampling
        self.synced_pair_counter = 0
        self.last_saved_time_sec = 0.0

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Synchronized subscribers for color and depth
        self.color_sub = Subscriber(self, Image, '/camera/color/image_raw', qos_profile=sensor_qos)
        self.depth_sub = Subscriber(self, Image, '/camera/depth/image_raw', qos_profile=sensor_qos)
        self.ts = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=20, slop=0.05)
        self.ts.registerCallback(self.synced_image_callback)

        # Camera info subscriptions
        self.color_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.color_info_callback, 10)
        self.depth_info_sub = self.create_subscription(
            CameraInfo, '/camera/depth/camera_info', self.depth_info_callback, 10)

        self.color_info_saved = False
        self.depth_info_saved = False

    def create_dirs(self) -> None:
        """Create directories for saving dataset outputs."""
        self.color_dir = os.path.join(self.dataset_root, 'color')
        self.depth_dir = os.path.join(self.dataset_root, 'depth')
        self.info_dir = os.path.join(self.dataset_root, 'camera_info')
        for dir_path in [self.color_dir, self.depth_dir, self.info_dir]:
            os.makedirs(dir_path, exist_ok=True)
        self.get_logger().info(f"数据集保存至: {self.dataset_root}")

    def _should_save_now(self, now_sec: float) -> bool:
        mode = self.get_parameter('sampling_mode').value
        if mode not in ('frame', 'time'):
            mode = 'frame'
        if mode == 'frame':
            n = int(self.get_parameter('sampling_every_n').value)
            if n <= 0:
                n = 1
            self.synced_pair_counter += 1
            if self.synced_pair_counter >= n:
                self.synced_pair_counter = 0
                return True
            return False
        else:
            interval_s = float(self.get_parameter('sampling_interval_s').value)
            if interval_s <= 0:
                interval_s = 0.5
            if (now_sec - self.last_saved_time_sec) >= interval_s:
                self.last_saved_time_sec = now_sec
                return True
            return False

    def synced_image_callback(self, color_msg: Image, depth_msg: Image) -> None:
        """Save synchronized color-depth pair according to sampling settings.
        Assumes the camera driver has depth_registration enabled for pixel alignment.
        """
        try:
            # Use color timestamp as the pair id
            now_sec = float(color_msg.header.stamp.sec) + float(color_msg.header.stamp.nanosec) / 1e9
            if not self._should_save_now(now_sec):
                return

            color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

            timestamp = f"{color_msg.header.stamp.sec}_{color_msg.header.stamp.nanosec // 1000000}"
            color_path = os.path.join(self.color_dir, f"color_{timestamp}.jpg")
            depth_path = os.path.join(self.depth_dir, f"depth_{timestamp}.png")

            cv2.imwrite(color_path, color_img)
            cv2.imwrite(depth_path, depth_img)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"同步图像处理失败: {str(exc)}")

    def color_info_callback(self, msg: CameraInfo) -> None:
        """Save color camera intrinsics once."""
        if self.color_info_saved:
            return
        info = {
            'width': msg.width,
            'height': msg.height,
            'fx': msg.k[0],
            'fy': msg.k[4],
            'cx': msg.k[2],
            'cy': msg.k[5],
            'distortion': list(msg.d),
        }
        save_path = os.path.join(self.info_dir, 'color_camera_info.json')
        with open(save_path, 'w', encoding='utf-8') as file_handle:
            json.dump(info, file_handle, indent=2, ensure_ascii=False)
        self.color_info_saved = True
        self.get_logger().info(f"保存彩色相机内参: {save_path}")

    def depth_info_callback(self, msg: CameraInfo) -> None:
        """Save depth camera intrinsics once."""
        if self.depth_info_saved:
            return
        info = {
            'width': msg.width,
            'height': msg.height,
            'fx': msg.k[0],
            'fy': msg.k[4],
            'cx': msg.k[2],
            'cy': msg.k[5],
            'distortion': list(msg.d),
        }
        save_path = os.path.join(self.info_dir, 'depth_camera_info.json')
        with open(save_path, 'w', encoding='utf-8') as file_handle:
            json.dump(info, file_handle, indent=2, ensure_ascii=False)
        self.depth_info_saved = True
        self.get_logger().info(f"保存深度相机内参: {save_path}")


def main() -> None:
    rclpy.init()
    node = BagToImages()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('停止数据提取')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



