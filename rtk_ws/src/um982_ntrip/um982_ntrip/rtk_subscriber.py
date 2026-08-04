"""Example consumer for the unified RTK fix topic."""

import rclpy
from rclpy.node import Node
from rtk_interfaces.msg import RtkFix


class RtkSubscriber(Node):
    """Subscribe to complete RTK fixes published by um982_ntrip."""

    def __init__(self) -> None:
        super().__init__('rtk_subscriber')
        self.subscription = self.create_subscription(
            RtkFix,
            'rtk/fix_info',
            self._on_fix,
            10,
        )

    def _on_fix(self, message: RtkFix) -> None:
        if not message.rtk_fixed:
            self.get_logger().warning(
                f'RTK unavailable: {message.status}, '
                f'satellites={message.satellites}')
            return

        self.get_logger().info(
            f'RTK fixed: latitude={message.latitude:.9f}, '
            f'longitude={message.longitude:.9f}, '
            f'altitude={message.altitude_ellipsoid:.3f} m')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RtkSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
