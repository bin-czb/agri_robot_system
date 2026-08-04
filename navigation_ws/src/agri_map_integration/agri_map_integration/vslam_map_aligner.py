"""Register the VSLAM map under the georeferenced map from an RViz pose."""

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformException,
    TransformListener,
)

from .geometry import (
    compute_map_to_vslam,
    quaternion_to_yaw,
    yaw_to_quaternion,
)


class VslamMapAligner(Node):
    """Convert a desired map-frame base pose into map->vslam_map."""

    def __init__(self) -> None:
        super().__init__('vslam_map_aligner')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('vslam_frame', 'vslam_map')
        self.declare_parameter('base_frame', 'bb_robot/base_link')
        self.declare_parameter('initialpose_topic', '/initialpose')

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.vslam_frame = str(self.get_parameter('vslam_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        initialpose_topic = str(
            self.get_parameter('initialpose_topic').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.pending_pose = None

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(
            String, '/map_alignment/status', status_qos)
        self.create_subscription(
            PoseWithCovarianceStamped,
            initialpose_topic,
            self._initialpose_callback,
            10,
        )
        self.timer = self.create_timer(0.25, self._try_alignment)
        self._publish_status('WAITING_FOR_2D_POSE_ESTIMATE')
        self.get_logger().info(
            f'Waiting on {initialpose_topic}; RViz pose will register '
            f'{self.map_frame} -> {self.vslam_frame}')

    def _initialpose_callback(
            self, message: PoseWithCovarianceStamped) -> None:
        if message.header.frame_id not in ('', self.map_frame):
            self.get_logger().error(
                f'Initial pose must use {self.map_frame}, got '
                f'{message.header.frame_id}')
            self._publish_status('REJECTED_WRONG_FRAME')
            return
        self.pending_pose = message
        self._publish_status('POSE_RECEIVED_WAITING_FOR_VSLAM_TF')
        self._try_alignment()

    def _try_alignment(self) -> None:
        if self.pending_pose is None:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.vslam_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.15),
            )
        except TransformException as error:
            self.get_logger().warning(
                f'Waiting for {self.vslam_frame} -> {self.base_frame}: '
                f'{error}',
                throttle_duration_sec=3.0,
            )
            return

        pose = self.pending_pose.pose.pose
        map_to_base = (
            pose.position.x,
            pose.position.y,
            quaternion_to_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )
        rotation = transform.transform.rotation
        translation = transform.transform.translation
        vslam_to_base = (
            translation.x,
            translation.y,
            quaternion_to_yaw(
                rotation.x, rotation.y, rotation.z, rotation.w),
        )
        map_to_vslam = compute_map_to_vslam(
            map_to_base, vslam_to_base)
        self._broadcast(map_to_vslam)
        self.pending_pose = None
        self._publish_status(
            f'ALIGNED x={map_to_vslam[0]:.3f} '
            f'y={map_to_vslam[1]:.3f} '
            f'yaw={map_to_vslam[2]:.4f}')
        self.get_logger().info(
            f'Registered {self.map_frame} -> {self.vslam_frame}: '
            f'x={map_to_vslam[0]:.3f}, y={map_to_vslam[1]:.3f}, '
            f'yaw={map_to_vslam[2]:.4f} rad')

    def _broadcast(self, pose) -> None:
        x, y, yaw = pose
        quaternion = yaw_to_quaternion(yaw)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.vslam_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self.broadcaster.sendTransform(transform)

    def _publish_status(self, value: str) -> None:
        self.status_pub.publish(String(data=value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VslamMapAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
