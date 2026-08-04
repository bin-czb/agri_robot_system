#!/usr/bin/env python3
"""
升级版底盘控制节点 - 使用高级ModBus-RTU串口通信
整合了升级版rs485模块，提供更稳定和功能丰富的串口通讯
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time
import threading

# 串口与CRC依赖（安全导入）
try:
    import serial
    from serial import SerialException
except Exception:  # pragma: no cover
    serial = None
    class SerialException(Exception):
        pass

try:
    import crcmod.predefined
    CRC_AVAILABLE = True
except Exception:  # pragma: no cover
    CRC_AVAILABLE = False

from agri_chassis_serial.simple_serial_bridge import SimpleSerialBridge

# ROS消息类型
from std_msgs.msg import Header, String, Float32, Bool
from geometry_msgs.msg import Twist

# 自定义消息类型
from trunk_interfaces.msg import ChassisControl


class ChassisController(Node):
    """底盘控制节点"""

    def __init__(self):
        super().__init__('chassis_controller')
        
        # 参数声明
        self.declare_parameters(
            namespace='',
            parameters=[
                # 串口参数
                ('serial_port', '/dev/ttyUSB0'),
                ('baud_rate', 9600),
                ('timeout', 1.0),
                ('device_id', 1),
                
                # 控制参数
                ('max_speed_value', 255),
                ('control_frequency', 10.0),  # Hz
                ('enable_crc', True),
                ('enable_feedback', False),
                # 兼容测试编码：使用与 serial_test 相同的固定字节（100/200/0）
                ('use_fixed_bytes', True),
                ('fixed_forward_byte', 100),
                ('fixed_reverse_byte', 200),
                # 新协议（按资料）寄存器映射
                ('use_speed_angle_registers', True),
                ('control_mode_register', 0x0003),
                ('turn_register', 0x0004),
                ('speed_register', 0x0005),
                # 速度/角速度映射到寄存器 -1000..1000 的上限
                ('max_linear_velocity', 0.5),
                ('max_angular_velocity', 1.0),
                
                # ROS话题参数
                ('control_topic', '/chassis_control/cmd'),
                ('twist_topic', '/cmd_vel'),
                ('status_topic', '/chassis_control/status'),
                
                # 安全参数
                ('watchdog_timeout', 2.0),  # 看门狗超时时间
                ('emergency_stop_enabled', True),
            ]
        )
        
        # 获取参数
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.device_id = self.get_parameter('device_id').value
        self.max_speed_value = self.get_parameter('max_speed_value').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.enable_crc = bool(self.get_parameter('enable_crc').value) and CRC_AVAILABLE
        self.enable_feedback = self.get_parameter('enable_feedback').value
        self.use_fixed_bytes = bool(self.get_parameter('use_fixed_bytes').value)
        self.fixed_forward_byte = int(self.get_parameter('fixed_forward_byte').value)
        self.fixed_reverse_byte = int(self.get_parameter('fixed_reverse_byte').value)
        self.use_speed_angle_registers = bool(self.get_parameter('use_speed_angle_registers').value)
        self.control_mode_reg = int(self.get_parameter('control_mode_register').value)
        self.turn_reg = int(self.get_parameter('turn_register').value)
        # 兼容字段名：沿用 motor_target_register 的旧参数，但优先 speed_register
        self.speed_reg = int(self.get_parameter('speed_register').value)
        self.max_linear_vel_param = float(self.get_parameter('max_linear_velocity').value)
        self.max_angular_vel_param = float(self.get_parameter('max_angular_velocity').value)
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value
        self.emergency_stop_enabled = self.get_parameter('emergency_stop_enabled').value
        
        # 话题名称
        self.control_topic = self.get_parameter('control_topic').value
        self.twist_topic = self.get_parameter('twist_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        
        # 状态变量
        self.serial_connection = None
        self.last_command_time = time.time()
        self.current_control_mode = 0
        self.last_linear_vel = 0.0
        self.last_angular_vel = 0.0
        self.connection_status = False
        self.commands_sent = 0
        self.communication_errors = 0
        
        # 线程锁
        self.serial_lock = threading.Lock()
        
        # 初始化串口连接
        self._initialize_serial()
        
        # QoS设置
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        # 订阅者
        self.control_subscription = self.create_subscription(
            ChassisControl,
            self.control_topic,
            self.control_callback,
            qos_profile
        )
        
        self.twist_subscription = self.create_subscription(
            Twist,
            self.twist_topic,
            self.twist_callback,
            qos_profile
        )
        
        # 发布者
        self.status_publisher = self.create_publisher(
            String,
            self.status_topic,
            qos_profile
        )
        
        # 定时器 - 看门狗和状态发布
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_callback)
        self.status_timer = self.create_timer(1.0, self.status_callback)
        
        # 启动时设置RS485控制模式
        if self.connection_status:
            self._set_rs485_control_mode()
        
        self.get_logger().info(f"底盘控制节点已启动")
        self.get_logger().info(f"串口: {self.serial_port}@{self.baud_rate}")
        self.get_logger().info(f"设备ID: {self.device_id}")
        self.get_logger().info(f"CRC使能: {self.enable_crc}")
        
        if not CRC_AVAILABLE and self.enable_crc:
            self.get_logger().warn("crcmod模块未安装，CRC功能禁用。请运行: pip install crcmod")

    def _initialize_serial(self):
        """初始化串口连接"""
        try:
            if serial is None:
                raise SerialException("pyserial 未安装，请执行: pip install pyserial")
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout
            )
            self.connection_status = True
            self.get_logger().info(f"串口连接成功: {self.serial_port}")
            
        except SerialException as e:
            self.connection_status = False
            self.get_logger().error(f"串口连接失败: {e}")
            self.serial_connection = None

    def _set_rs485_control_mode(self):
        """设置RS485控制模式 (基于原始代码)"""
        try:
            command = self._create_write_command(
                self.device_id, 
                0x06,  # 写单个寄存器
                self.control_mode_reg,
                0x01   # RS485控制模式
            )
            self._send_command(command)
            self.get_logger().info("已设置RS485控制模式")
            
        except Exception as e:
            self.get_logger().error(f"设置控制模式失败: {e}")

    def control_callback(self, msg):
        """底盘控制指令回调"""
        try:
            if not self.connection_status:
                self.get_logger().warn("串口未连接，忽略控制指令")
                return
                
            # 检查紧急停止
            if msg.emergency_stop:
                self._emergency_stop()
                return
                
            # 更新控制模式
            if msg.control_mode != self.current_control_mode:
                self.current_control_mode = msg.control_mode
                if msg.control_mode == 0:  # 停止模式
                    self._stop_vehicle()
                    return
            
            # 执行运动控制
            if msg.control_mode == 1:  # 跟踪模式
                self._execute_tracking_control(msg)
            elif msg.control_mode == 2:  # 手动模式
                self._execute_manual_control(msg)
                
            self.last_command_time = time.time()
            
        except Exception as e:
            self.get_logger().error(f"控制指令处理错误: {e}")

    def twist_callback(self, msg):
        """Twist消息回调 (兼容性支持)"""
        try:
            if not self.connection_status:
                return
                
            # 转换Twist到底盘控制
            chassis_cmd = ChassisControl()
            chassis_cmd.linear_velocity = msg.linear.x
            chassis_cmd.angular_velocity = msg.angular.z
            chassis_cmd.control_mode = 1  # 自动模式
            chassis_cmd.emergency_stop = False
            
            self.control_callback(chassis_cmd)
            
        except Exception as e:
            self.get_logger().error(f"Twist指令处理错误: {e}")

    def _execute_tracking_control(self, msg):
        """执行跟踪控制 (基于原始算法逻辑)"""
        linear_vel = msg.linear_velocity
        angular_vel = msg.angular_velocity
        
        # 始终由线速度/角速度计算履带速度，避免外部字段为0导致无动作
        left_speed, right_speed = self._calculate_track_speeds(linear_vel, angular_vel)
        
        # 发送电机控制指令
        self._send_motor_commands(left_speed, right_speed)
        
        # 保存最后的指令
        self.last_linear_vel = linear_vel
        self.last_angular_vel = angular_vel

    def _execute_manual_control(self, msg):
        """执行手动控制"""
        # 手动控制逻辑 (可根据需要扩展)
        self._execute_tracking_control(msg)

    def _calculate_track_speeds(self, linear_vel, angular_vel):
        """计算履带速度"""
        # 差速驱动模型
        track_width = 1.12  # 履带间距，根据实际底盘调整
        
        left_vel = linear_vel - (angular_vel * track_width / 2.0)
        right_vel = linear_vel + (angular_vel * track_width / 2.0)
        
        # 归一化到[-1, 1]范围
        max_vel = 0.5  # 最大速度，根据实际调整
        left_speed = max(-1.0, min(1.0, left_vel / max_vel))
        right_speed = max(-1.0, min(1.0, right_vel / max_vel))
        
        return left_speed, right_speed

    def _send_motor_commands(self, left_speed, right_speed):
        """
        发送电机控制指令
        基于原始代码的send_control_commands函数
        """
        try:
            # 兼容模式：使用与 serial_test 相同的固定字节编码
            if self.use_speed_angle_registers:
                # 根据线/角速度映射到 -1000..1000 的寄存器值
                def clip(v, lo, hi):
                    return max(lo, min(hi, v))
                # 估算线/角速度（由上游 linear/ angular 设置）
                # 这里复用已保存的最后指令（_execute_tracking_control 中设置）
                lin = float(self.last_linear_vel)
                ang = float(self.last_angular_vel)
                lin_reg = int(clip(round(1000.0 * (lin / (self.max_linear_vel_param if self.max_linear_vel_param > 1e-6 else 0.5))), -1000, 1000))
                ang_reg = int(clip(round(1000.0 * (ang / (self.max_angular_vel_param if self.max_angular_vel_param > 1e-6 else 1.0))), -1000, 1000))

                def to_u16_signed(x):
                    return x & 0xFFFF
                # 先写转向角，再写速度
                cmd_turn = self._create_write_command(self.device_id, 0x06, self.turn_reg, to_u16_signed(ang_reg))
                cmd_speed = self._create_write_command(self.device_id, 0x06, self.speed_reg, to_u16_signed(lin_reg))
                with self.serial_lock:
                    self._send_command(cmd_turn)
                    self._send_command(cmd_speed)

            elif self.use_fixed_bytes:
                def to_byte(s):
                    if s > 1e-6:
                        return self.fixed_forward_byte
                    if s < -1e-6:
                        return self.fixed_reverse_byte
                    return 0
                left_b = to_byte(left_speed)
                right_b = to_byte(right_speed)
                combined_value = (left_b << 8) | (right_b & 0xFF)
                command = self._create_write_command(
                    self.device_id,
                    0x06,
                    self.speed_reg,  # 复用 speed_reg 位置写入组合值（兼容旧控制器）
                    combined_value
                )
                with self.serial_lock:
                    self._send_command(command)
            else:
                # 标准比例编码（可能与设备不匹配，仅作备选）
                left_value = int(max(-1.0, min(1.0, left_speed)) * self.max_speed_value)
                right_value = int(max(-1.0, min(1.0, right_speed)) * self.max_speed_value)
                combined_value = (left_value & 0xFF) | ((right_value & 0xFF) << 8)

                command = self._create_write_command(
                    self.device_id,
                    0x06,
                    self.speed_reg,
                    combined_value
                )
                with self.serial_lock:
                    self._send_command(command)
            
            self.commands_sent += 1
            
        except Exception as e:
            self.get_logger().error(f"发送电机指令失败: {e}")
            self.communication_errors += 1

    def _emergency_stop(self):
        """紧急停止"""
        self.get_logger().warn("执行紧急停止")
        self._stop_vehicle()

    def _stop_vehicle(self):
        """停止车辆"""
        try:
            # 发送停止指令
            command = self._create_write_command(
                self.device_id,
                0x06,
                self.motor_target_reg,
                0x0000  # 停止
            )
            
            with self.serial_lock:
                self._send_command(command)
                
            self.last_linear_vel = 0.0
            self.last_angular_vel = 0.0
            
        except Exception as e:
            self.get_logger().error(f"停止指令发送失败: {e}")

    def _create_write_command(self, device_id, function_code, register_addr, value):
        """
        创建写寄存器命令
        基于原始代码的send_command函数
        """
        data = [
            device_id,
            function_code,
            (register_addr >> 8) & 0xFF,    # 高字节
            register_addr & 0xFF,           # 低字节
            (value >> 8) & 0xFF,            # 值高字节
            value & 0xFF                    # 值低字节
        ]
        
        if self.enable_crc:
            crc = self._calculate_crc(data)
            data.extend(crc)
        
        return bytes(data)

    def _calculate_crc(self, data):
        """
        计算CRC校验
        基于原始代码的calculate_crc函数
        """
        if not CRC_AVAILABLE:
            return [0, 0]
            
        crc16 = crcmod.predefined.Crc('modbus')
        crc16.update(bytes(data))
        crc_value = crc16.crcValue
        
        # 返回小端序的CRC字节
        return [crc_value & 0xFF, (crc_value >> 8) & 0xFF]

    def _send_command(self, command):
        """发送串口命令"""
        if self.serial_connection and self.serial_connection.is_open:
            try:
                self.serial_connection.write(command)
                self.serial_connection.flush()
                
                # 如果启用反馈，读取响应
                if self.enable_feedback:
                    response = self.serial_connection.read(8)  # 读取响应
                    if len(response) > 0:
                        self.get_logger().debug(f"收到响应: {list(response)}")
                
            except Exception as e:
                self.get_logger().error(f"串口通信错误: {e}")
                self.communication_errors += 1
                # 尝试重新连接
                self._reconnect_serial()

    def _reconnect_serial(self):
        """重新连接串口"""
        try:
            if self.serial_connection:
                self.serial_connection.close()
            
            time.sleep(1)  # 等待一秒再重连
            self._initialize_serial()
            
        except Exception as e:
            self.get_logger().error(f"串口重连失败: {e}")

    def watchdog_callback(self):
        """看门狗定时器 - 检查超时"""
        current_time = time.time()
        time_since_command = current_time - self.last_command_time
        
        if (self.emergency_stop_enabled and 
            time_since_command > self.watchdog_timeout and
            (self.last_linear_vel != 0.0 or self.last_angular_vel != 0.0)):
            
            self.get_logger().warn(f"控制超时 ({time_since_command:.2f}s)，执行安全停止")
            self._stop_vehicle()

    def status_callback(self):
        """状态发布定时器"""
        status_msg = String()
        
        status_info = {
            'connection': 'connected' if self.connection_status else 'disconnected',
            'serial_port': self.serial_port,
            'device_id': self.device_id,
            'control_mode': self.current_control_mode,
            'commands_sent': self.commands_sent,
            'comm_errors': self.communication_errors,
            'last_linear_vel': round(self.last_linear_vel, 3),
            'last_angular_vel': round(self.last_angular_vel, 3)
        }
        
        status_msg.data = str(status_info)
        self.status_publisher.publish(status_msg)

    def destroy_node(self):
        """节点销毁时清理资源"""
        try:
            if self.serial_connection and self.serial_connection.is_open:
                # 发送停止指令
                self._stop_vehicle()
                time.sleep(0.1)
                self.serial_connection.close()
                self.get_logger().info("串口连接已关闭")
        except:
            pass
        
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        chassis_controller = ChassisController()
        rclpy.spin(chassis_controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点运行错误: {e}")
    finally:
        if 'chassis_controller' in locals():
            chassis_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
