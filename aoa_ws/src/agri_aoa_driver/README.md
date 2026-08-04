# agri_aoa_driver

AOA 串口和无人机 MAVLink 位置源的标准 ROS 2 Python 包。统一入口：

```bash
ros2 launch agri_aoa_driver aoa_driver.launch.py
```

参数位于 `config/aoa_driver.yaml`。AOA 原始协议、几何解算和滑动窗口代码均作为包内模块安装，不再依赖源码目录或 `sys.path`。
