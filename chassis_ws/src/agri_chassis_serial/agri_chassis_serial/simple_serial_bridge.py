#!/usr/bin/env python3
"""
简化版串口通讯模块 - 不依赖ROS2
提供基本的ModBus-RTU通讯功能，可以直接在Python脚本中使用

使用示例:
    from simple_serial_bridge import SimpleSerialBridge
    
    # 创建串口桥接器
    bridge = SimpleSerialBridge('/dev/ttyUSB0', 115200)
    
    # 发送运动指令
    bridge.send_motion_command(linear_speed=0.5, angular_speed=0.2)
    
    # 读取电池电压
    voltage = bridge.read_battery_voltage()
    print(f"电池电压: {voltage}V")
    
    # 设置控制模式
    bridge.set_control_mode(motor_power=True, reverse_mode=False)
    
    # 紧急停止
    bridge.emergency_stop()
"""

from pymodbus.client import ModbusSerialClient
import time
import logging

class SimpleSerialBridge:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, slave_id=1, 
                 max_linear_speed=1.0, max_angular_speed=2.0):
        """
        初始化串口桥接器
        
        参数:
            port: 串口设备路径
            baudrate: 波特率
            slave_id: ModBus从机地址
            max_linear_speed: 最大线速度 (m/s)
            max_angular_speed: 最大角速度 (rad/s)
        """
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        
        # 配置日志
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # 初始化ModBus-RTU客户端（pymodbus v3+ 使用 framer 参数）
        self.client = ModbusSerialClient(
            self.port,
            framer='rtu',
            baudrate=self.baudrate,
            timeout=0.1,
            parity='N',  # 无校验
            stopbits=1,  # 1位停止位
            bytesize=8   # 8位数据位
        )
        
        # 连接到串口设备
        self.connected = self.connect()
        
    def connect(self):
        """连接到ModBus设备"""
        ok = self.client.connect()
        self.connected = bool(ok)
        if self.connected:
            self.logger.info(f"成功连接到ModBus设备 {self.port}，波特率 {self.baudrate}")
            # 连接标志已就绪后再初始化控制模式
            self.initialize_control_mode()
            return True
        else:
            self.logger.error(f"无法连接到ModBus设备 {self.port}")
            return False
            
    def disconnect(self):
        """断开连接"""
        try:
            if hasattr(self, 'client'):
                # 紧急停止后断开连接
                self.emergency_stop()
                self.client.close()
                self.logger.info("ModBus连接已断开")
        except Exception as e:
            self.logger.error(f"断开连接时出错: {e}")
    
    def initialize_control_mode(self):
        """初始化控制模式寄存器（地址3），设置电机通电和正常模式"""
        # 仅在客户端可用时尝试写入
        if not hasattr(self, 'client'):
            return False
            
        # bit0=1 (电机通电), 其他位保持0（正常模式）
        control_mode = 0x01
        
        result = self.client.write_register(
            address=3,
            value=control_mode,
            device_id=self.slave_id
        )
        
        if result.isError():
            self.logger.error(f"初始化控制模式失败: {result}")
            return False
        else:
            self.logger.info("控制模式初始化成功: 电机通电，正常模式")
            return True
    
    def send_motion_command(self, linear_speed=0.0, angular_speed=0.0):
        """
        发送运动指令
        
        参数:
            linear_speed: 线速度 (m/s)，正值前进，负值后退
            angular_speed: 角速度 (rad/s)，正值左转，负值右转
        
        返回:
            bool: 发送是否成功
        """
        if not self.connected:
            self.logger.error("设备未连接")
            return False
            
        try:
            # 转换为-1000~1000范围的整数
            steering_angle = int((angular_speed / self.max_angular_speed) * 1000)
            steering_angle = max(min(steering_angle, 1000), -1000)
            
            speed = int((linear_speed / self.max_linear_speed) * 1000)
            speed = max(min(speed, 1000), -1000)
            
            # Modbus单寄存器为16位无符号，负数需按两补码编码
            def to_u16(value: int) -> int:
                return value & 0xFFFF
            
            steer_u16 = to_u16(steering_angle)
            speed_u16 = to_u16(speed)
            
            # 发送指令到寄存器
            result1 = self.client.write_register(4, steer_u16, device_id=self.slave_id)
            result2 = self.client.write_register(5, speed_u16, device_id=self.slave_id)
            
            if result1.isError() or result2.isError():
                self.logger.error(f"发送运动指令失败 - 转向: {result1}, 速度: {result2}")
                return False
            else:
                self.logger.debug(f"发送运动指令成功 - 转向角度: {steering_angle}, 行驶速度: {speed}")
                return True
                
        except Exception as e:
            self.logger.error(f"发送运动指令出错: {e}")
            return False
    
    def read_battery_voltage(self):
        """
        读取电池电压
        
        返回:
            float: 电池电压值，失败时返回None
        """
        if not self.connected:
            self.logger.error("设备未连接")
            return None
            
        try:
            result = self.client.read_holding_registers(
                address=6,
                count=1,
                device_id=self.slave_id
            )
            
            if result.isError():
                self.logger.warning(f"读取电池电压失败: {result}")
                return None
            
            # 转换为实际电压值（1位小数）
            voltage_raw = result.registers[0]
            voltage = voltage_raw / 10.0
            
            return voltage
            
        except Exception as e:
            self.logger.error(f"读取电池电压出错: {e}")
            return None
    
    def set_control_mode(self, motor_power=True, reverse_mode=False, 
                         straight_translation=False, in_place_rotation=False,
                         steering_proportion_mode=False):
        """
        设置控制模式寄存器（地址3）
        
        参数:
            motor_power: 电机电源状态
            reverse_mode: 反向模式
            straight_translation: 直平移模式
            in_place_rotation: 原地掉头模式
            steering_proportion_mode: 转向角比例模式
        
        返回:
            bool: 设置是否成功
        """
        if not self.connected:
            self.logger.error("设备未连接")
            return False
            
        try:
            # 构建控制模式值
            control_mode = 0x00
            if motor_power:
                control_mode |= 0x01
            if reverse_mode:
                control_mode |= 0x02
            if straight_translation:
                control_mode |= 0x10
            if in_place_rotation:
                control_mode |= 0x20
            if steering_proportion_mode:
                control_mode |= 0x40
            
            result = self.client.write_register(3, control_mode, device_id=self.slave_id)
            
            if result.isError():
                self.logger.error(f"设置控制模式失败: {result}")
                return False
            else:
                self.logger.info(f"控制模式已更新 - 电机电源: {motor_power}")
                return True
                
        except Exception as e:
            self.logger.error(f"设置控制模式出错: {e}")
            return False
    
    def emergency_stop(self):
        """紧急停止：关闭电机电源并停止所有运动"""
        if not self.connected:
            return
            
        try:
            # 停止运动
            self.client.write_register(4, 0, device_id=self.slave_id)  # 转向角度清零
            self.client.write_register(5, 0, device_id=self.slave_id)  # 行驶速度清零
            
            # 断开电机电源
            self.set_control_mode(motor_power=False)
            
            self.logger.warning("执行紧急停止：电机断电，运动停止")
            
        except Exception as e:
            self.logger.error(f"紧急停止执行出错: {e}")
    
    def move_forward(self, speed=0.5):
        """前进"""
        return self.send_motion_command(linear_speed=speed, angular_speed=0.0)
    
    def move_backward(self, speed=0.5):
        """后退"""
        return self.send_motion_command(linear_speed=-speed, angular_speed=0.0)
    
    def turn_left(self, angular_speed=1.0):
        """左转"""
        return self.send_motion_command(linear_speed=0.0, angular_speed=angular_speed)
    
    def turn_right(self, angular_speed=1.0):
        """右转"""
        return self.send_motion_command(linear_speed=0.0, angular_speed=-angular_speed)
    
    def stop(self):
        """停止"""
        return self.send_motion_command(linear_speed=0.0, angular_speed=0.0)
    
    def get_status(self):
        """获取设备状态信息"""
        voltage = self.read_battery_voltage()
        return {
            'connected': self.connected,
            'battery_voltage': voltage,
            'port': self.port,
            'baudrate': self.baudrate,
            'slave_id': self.slave_id
        }


# 使用示例和测试函数
def main():
    """示例使用"""
    print("=" * 50)
    print("简化版串口通讯模块测试")
    print("=" * 50)
    
    # 创建串口桥接器
    bridge = SimpleSerialBridge('/dev/ttyUSB0', 115200)
    
    if not bridge.connected:
        print("❌ 无法连接到设备，请检查串口设置")
        return
    
    try:
        # 显示状态
        status = bridge.get_status()
        print(f"✅ 设备已连接")
        print(f"📍 端口: {status['port']}")
        print(f"⚡ 波特率: {status['baudrate']}")
        print(f"🔋 电池电压: {status['battery_voltage']}V")
        
        print("\n🚀 开始运动测试...")
        
        # 前进测试
        print("  ⬆️  前进...")
        bridge.move_forward(0.3)
        time.sleep(2)
        
        # 左转测试
        print("  ⬅️  左转...")
        bridge.turn_left(0.5)
        time.sleep(2)
        
        # 右转测试
        print("  ➡️  右转...")
        bridge.turn_right(0.5)
        time.sleep(2)
        
        # 后退测试
        print("  ⬇️  后退...")
        bridge.move_backward(0.3)
        time.sleep(2)
        
        # 停止
        print("  🛑 停止")
        bridge.stop()
        
        print("\n✅ 测试完成")
        
    except KeyboardInterrupt:
        print("\n⏸️  用户中断")
        
    finally:
        # 确保安全关闭
        bridge.disconnect()
        print("🔚 设备已安全断开")


if __name__ == '__main__':
    main()
