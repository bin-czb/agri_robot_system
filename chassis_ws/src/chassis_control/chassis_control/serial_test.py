#!/usr/bin/env python3
"""
串口测试节点 - 用于调试和测试RS485通信
"""

import rclpy
from rclpy.node import Node
import serial
import time

# 导入CRC计算
try:
    import crcmod.predefined
    CRC_AVAILABLE = True
except ImportError:
    CRC_AVAILABLE = False


class SerialTest(Node):
    """串口测试节点"""

    def __init__(self):
        super().__init__('serial_test')
        
        # 参数
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('device_id', 1)
        self.declare_parameter('test_mode', 'basic')  # basic, motor, read
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.device_id = self.get_parameter('device_id').value
        self.test_mode = self.get_parameter('test_mode').value
        
        # 初始化串口
        self.ser = None
        self._init_serial()
        
        # 创建测试定时器
        self.test_timer = self.create_timer(2.0, self.run_test)
        self.test_count = 0
        
        self.get_logger().info(f"串口测试节点启动: {self.serial_port}@{self.baud_rate}")
        self.get_logger().info(f"测试模式: {self.test_mode}")

    def _init_serial(self):
        """初始化串口"""
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info("串口连接成功")
        except Exception as e:
            self.get_logger().error(f"串口连接失败: {e}")
            self.ser = None

    def calculate_crc(self, data):
        """计算CRC"""
        if not CRC_AVAILABLE:
            return [0, 0]
        
        crc16 = crcmod.predefined.Crc('modbus')
        crc16.update(bytes(data))
        crc_value = crc16.crcValue
        return [crc_value & 0xFF, (crc_value >> 8) & 0xFF]

    def send_command(self, device_id, function_code, register_addr, value):
        """发送Modbus命令"""
        data = [
            device_id,
            function_code,
            (register_addr >> 8) & 0xFF,
            register_addr & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF
        ]
        
        crc = self.calculate_crc(data)
        command = bytes(data + crc)
        
        self.get_logger().info(f"发送指令: {list(command)}")
        
        if self.ser:
            try:
                self.ser.write(command)
                self.ser.flush()
                
                # 读取响应
                response = self.ser.read(8)
                if len(response) > 0:
                    self.get_logger().info(f"收到响应: {list(response)}")
                else:
                    self.get_logger().warn("无响应")
                    
            except Exception as e:
                self.get_logger().error(f"发送命令失败: {e}")

    def read_registers(self, device_id, start_addr, count):
        """读取寄存器"""
        data = [device_id, 0x03, (start_addr >> 8) & 0xFF, start_addr & 0xFF, 0x00, count]
        crc = self.calculate_crc(data)
        command = bytes(data + crc)
        
        self.get_logger().info(f"读取寄存器指令: {list(command)}")
        
        if self.ser:
            try:
                self.ser.write(command)
                response = self.ser.read(32)  # 读取更多字节
                if len(response) > 0:
                    self.get_logger().info(f"读取响应: {list(response)}")
                else:
                    self.get_logger().warn("读取无响应")
            except Exception as e:
                self.get_logger().error(f"读取失败: {e}")

    def run_test(self):
        """运行测试"""
        if not self.ser:
            self.get_logger().warn("串口未连接，跳过测试")
            return

        self.test_count += 1
        self.get_logger().info(f"=== 测试循环 {self.test_count} ===")

        # HY7610 寄存器地址
        CONTROL_MODE_REG = 0x0003  # 控制模式（bit0=1 电机通电）
        TURN_REG         = 0x0004  # 转向角度 -1000~1000
        SPEED_REG        = 0x0005  # 行驶速度 -1000~1000

        if self.test_mode == 'basic':
            # 基础测试：设置控制模式（电机通电）
            self.get_logger().info("测试：设置控制模式（电机通电）")
            self.send_command(self.device_id, 0x06, CONTROL_MODE_REG, 0x01)

        elif self.test_mode == 'motor':
            # 电机测试：先确保电机通电，再循环发送运动指令
            # 负数按补码转 u16
            def u16(v):
                return v & 0xFFFF

            if self.test_count == 1:
                # 首次先通电
                self.get_logger().info("测试：电机通电")
                self.send_command(self.device_id, 0x06, CONTROL_MODE_REG, 0x01)
            elif self.test_count % 4 == 2:
                self.get_logger().info("测试：前进 speed=500")
                self.send_command(self.device_id, 0x06, TURN_REG,  u16(0))
                self.send_command(self.device_id, 0x06, SPEED_REG, u16(500))
            elif self.test_count % 4 == 3:
                self.get_logger().info("测试：左转 angle=-500")
                self.send_command(self.device_id, 0x06, TURN_REG,  u16(-500))
                self.send_command(self.device_id, 0x06, SPEED_REG, u16(0))
            elif self.test_count % 4 == 0:
                self.get_logger().info("测试：右转 angle=500")
                self.send_command(self.device_id, 0x06, TURN_REG,  u16(500))
                self.send_command(self.device_id, 0x06, SPEED_REG, u16(0))
            else:
                self.get_logger().info("测试：停止")
                self.send_command(self.device_id, 0x06, TURN_REG,  u16(0))
                self.send_command(self.device_id, 0x06, SPEED_REG, u16(0))

        elif self.test_mode == 'read':
            # 读取测试：从控制模式寄存器开始，读4个寄存器
            self.get_logger().info("测试：读取寄存器 0x0003~0x0006")
            self.read_registers(self.device_id, CONTROL_MODE_REG, 4)

        time.sleep(0.1)  # 短暂延时


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        serial_test = SerialTest()
        rclpy.spin(serial_test)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点运行错误: {e}")
    finally:
        if 'serial_test' in locals():
            serial_test.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

