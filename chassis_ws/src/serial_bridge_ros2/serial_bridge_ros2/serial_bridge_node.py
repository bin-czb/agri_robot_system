#!/usr/bin/env python3
"""
ROS 2 串口通信节点.

将 Modbus RTU 通信封装为标准 ROS 2 话题接口。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from agri_chassis_serial.simple_serial_bridge import SimpleSerialBridge

# ROS消息类型
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Twist


class SerialBridgeROS2(Node):
    """ROS 2 Modbus RTU 串口通信节点."""

    def __init__(self):
        super().__init__('serial_bridge_ros2')

        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('serial_port', '/dev/ttyUSB0'),
                ('baudrate', 115200),
                ('slave_id', 1),
                ('max_linear_speed', 1.0),
                ('max_angular_speed', 2.0),
                ('battery_check_frequency', 1.0),
                # 当相机安装在底盘尾部且希望“倒着走”跟随时，可将该参数设为 true
                ('invert_forward', False),
                ('cmd_vel_topic', '/cmd_vel'),
                ('battery_voltage_topic', '/battery_voltage'),
                ('modbus_connected_topic', '/modbus_connected'),
                ('chassis_status_topic', '/chassis_status'),
            ]
        )

        # 获取参数
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.slave_id = self.get_parameter('slave_id').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.battery_freq = self.get_parameter('battery_check_frequency').value
        self.invert_forward = bool(self.get_parameter('invert_forward').value)

        # 话题名称
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.battery_topic = self.get_parameter('battery_voltage_topic').value
        self.connection_topic = self.get_parameter('modbus_connected_topic').value
        self.status_topic = self.get_parameter('chassis_status_topic').value

        # 初始化串口桥接器
        self.serial_bridge = SimpleSerialBridge(
            port=self.serial_port,
            baudrate=self.baudrate,
            slave_id=self.slave_id,
            max_linear_speed=self.max_linear_speed,
            max_angular_speed=self.max_angular_speed
        )

        if not self.serial_bridge.connected:
            self.get_logger().error(f"无法连接到串口设备 {self.serial_port}")
            return

        self.get_logger().info("串口通讯节点已启动")
        self.get_logger().info(
            f"串口设备: {self.serial_port}@{self.baudrate}")
        if self.invert_forward:
            self.get_logger().warn(
                "已启用 invert_forward: 线速度为正时，底盘将实际后退运动")

        # QoS设置：使用RELIABLE以兼容默认发布者（例如 ros2 topic pub 默认可靠）
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # 订阅者 - 运动控制指令
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            qos_profile
        )

        # 发布者
        self.battery_pub = self.create_publisher(
            Float32,
            self.battery_topic,
            qos_profile
        )

        self.connection_pub = self.create_publisher(
            Bool,
            self.connection_topic,
            qos_profile
        )

        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            qos_profile
        )

        # 定时器 - 定期读取电池电压和状态
        self.timer = self.create_timer(
            1.0/self.battery_freq,
            self.battery_and_status_callback
        )

        self.get_logger().info("ROS 2 话题接口:")
        self.get_logger().info(f"  订阅: {self.cmd_vel_topic}")
        self.get_logger().info(f"  发布: {self.battery_topic}")
        self.get_logger().info(f"  发布: {self.connection_topic}")
        self.get_logger().info(f"  发布: {self.status_topic}")

    def cmd_vel_callback(self, msg):
        """处理运动控制指令."""
        try:
            if not self.serial_bridge.connected:
                self.get_logger().warn("设备未连接，忽略运动指令")
                return

            # 线速度根据 invert_forward 进行可选反向（用于相机安装在尾部时“倒车跟随”）
            linear = msg.linear.x
            if self.invert_forward:
                linear = -linear

            # 发送运动指令
            success = self.serial_bridge.send_motion_command(
                linear_speed=linear,
                angular_speed=msg.angular.z
            )

            if not success:
                self.get_logger().error("发送运动指令失败")

        except Exception as e:
            self.get_logger().error(f"处理运动指令出错: {e}")

    def battery_and_status_callback(self):
        """读取并发布电池电压和设备状态."""
        try:
            # 读取电池电压
            voltage = self.serial_bridge.read_battery_voltage()

            # 发布电池电压
            if voltage is not None:
                voltage_msg = Float32()
                voltage_msg.data = voltage
                self.battery_pub.publish(voltage_msg)

            # 获取并发布设备状态
            status = self.serial_bridge.get_status()

            # 发布连接状态
            connection_msg = Bool()
            connection_msg.data = status['connected']
            self.connection_pub.publish(connection_msg)

            # 发布详细状态
            status_msg = String()
            status_text = (
                f"Port: {status['port']}, "
                f"Baudrate: {status['baudrate']}, "
                f"Connected: {status['connected']}, "
                f"Battery: {status['battery_voltage']}V"
            )
            status_msg.data = status_text
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"读取状态出错: {e}")

            # 发布断开状态
            connection_msg = Bool()
            connection_msg.data = False
            self.connection_pub.publish(connection_msg)

    def emergency_stop_service(self, request, response):
        """执行紧急停止服务."""
        try:
            self.serial_bridge.emergency_stop()
            self.get_logger().warn("执行紧急停止")
            response.success = True
            response.message = "紧急停止执行成功"
        except Exception as e:
            self.get_logger().error(f"紧急停止失败: {e}")
            response.success = False
            response.message = f"紧急停止失败: {e}"

        return response

    def set_control_mode_service(self, request, response):
        """执行控制模式设置服务."""
        try:
            success = self.serial_bridge.set_control_mode(
                motor_power=request.motor_power,
                reverse_mode=request.reverse_mode,
                straight_translation=request.straight_translation,
                in_place_rotation=request.in_place_rotation,
                steering_proportion_mode=request.steering_proportion_mode
            )

            response.success = success
            response.message = "控制模式设置成功" if success else "控制模式设置失败"

        except Exception as e:
            self.get_logger().error(f"设置控制模式失败: {e}")
            response.success = False
            response.message = f"设置控制模式失败: {e}"

        return response

    def destroy_node(self):
        """节点销毁时安全关闭串口."""
        try:
            if hasattr(self, 'serial_bridge'):
                self.serial_bridge.disconnect()
                self.get_logger().info("串口连接已安全关闭")
        except Exception as e:
            self.get_logger().error(f"关闭串口时出错: {e}")
        finally:
            super().destroy_node()


def main(args=None):
    """运行 ROS 2 节点."""
    rclpy.init(args=args)

    try:
        node = SerialBridgeROS2()

        if not node.serial_bridge.connected:
            node.get_logger().error("串口连接失败，退出节点")
            return

        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n用户中断，安全关闭中...")
    except Exception as e:
        print(f"节点运行出错: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print("节点已安全关闭")


if __name__ == '__main__':
    main()
