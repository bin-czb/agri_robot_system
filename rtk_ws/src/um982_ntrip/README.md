# UM982 NTRIP ROS 2 驱动

本包面向 Ubuntu 22.04 和 ROS 2 Humble，负责读取 UM982 输出的 GGA、登录
NTRIP 服务、把 RTCM 差分数据写回接收机，并发布标准 ROS 2 定位话题。

## 编译

必须使用 Ubuntu 系统 Python 3.10，不要使用 Conda Python：

```bash
cd /home/czb/agri_robot_system/rtk_ws
source /opt/ros/humble/setup.bash

/usr/bin/python3 -m colcon build --symlink-install \
  --packages-select rtk_interfaces um982_ntrip \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3

source install/setup.bash
```

串口用户需要属于 `dialout` 组：

```bash
sudo usermod -aG dialout "$USER"
```

修改用户组后需要注销并重新登录。

## 安全保存和自动加载 NTRIP 凭据

推荐使用农业导航工作空间提供的安全配置脚本。首次执行：

```bash
cd /home/czb/agri_robot_system/rtk_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run agri_rtk_localization configure_ntrip_credentials
```

以后直接自动加载凭据并启动驱动：

```bash
ros2 run agri_rtk_localization start_um982_ntrip
```

凭据保存在 `~/.config/agri_rtk/ntrip.env`，权限为 `600`，不会写入本仓库。

## 配置

串口、NTRIP 地址、端口和挂载点位于：

```text
config/um982_ntrip.yaml
```

关键连接参数：

```yaml
receiver_mode_command: MODE ROVER SURVEY
receiver_pvt_command: CONFIG PVTALG MULTI
receiver_rtk_timeout_sec: 60
send_gga_interval_sec: 1.0
reconnect_delay_sec: 5.0
rtcm_stall_timeout_sec: 10.0
rtcm_status_interval_sec: 5.0
```

UM982 默认 rover 动态模型偏向 UAV。当前系统是地面低动态精准农业平台，因此
驱动每次启动都会发送 `MODE ROVER SURVEY`。该命令只设置本次运行状态，不自动
执行 `SAVECONFIG`。后续底盘高速或强振动实验应重新验证动态模型，不能只为追求
`FIX` 状态盲目切换参数。

`CONFIG RTK TIMEOUT 60` 用于让接收机在差分中断后及时退出陈旧 RTK 状态。
它不会帮助模糊度固定，也不会把浮点解伪装成固定解。

UM982 官方命令手册说明 `PVTALG` 默认值为 `SINGLE`。驱动启动时设置
`CONFIG PVTALG MULTI`，启用双频 PVT 解算。它和 rover 动态模型一样不会自动
写入闪存；停止使用本驱动后，接收机永久配置不会被静默改写。

`rtcm_stall_timeout_sec` 用于处理 TCP 仍显示连接、但服务端已经停止发送 RTCM
的半开连接。连续 10 秒没有 RTCM 时，驱动会主动关闭旧连接并自动重连。

## 输出话题

- `/rtk/fix_info`：完整 RTK 状态，推荐作为诊断接口。
- `/fix`：标准 `sensor_msgs/NavSatFix`。
- `/rtk/status`：`NO_FIX`、`SINGLE`、`DGNSS`、`RTK_FLOAT` 或
  `RTK_FIXED`。
- `/rtk/fix_quality`：GGA 原始质量，`4` 为固定解，`5` 为浮点解。
- `/rtk/satellites`：卫星数量。
- `/rtk/hdop`：水平精度因子。
- `/rtk/correction_age`：当前解使用的差分龄期，单位为秒。
- `/rtk/rtcm_bytes`：CRC 校验通过并写入 UM982 的累计 RTCM3 字节数。
- `/rtk/rtcm_status`：有效帧数、坏 CRC 数和 RTCM 消息类型统计。
- `/rtk/receiver_response`：`MODE`、`VERSIONA` 等接收机查询响应。
- `/rtk/gga`：UM982 原始 GGA 语句。
- `/rtk/height_msl`：海拔高度。
- `/rtk/geoid_separation`：大地水准面分离值。

## 运行检查

```bash
ros2 topic echo /rtk/fix_info
ros2 topic echo /rtk/rtcm_bytes
ros2 topic echo /rtk/rtcm_status
ros2 topic echo /rtk/receiver_response
ros2 topic echo /rtk/correction_age
```

健康的差分链路应满足：

```text
/rtk/rtcm_bytes 持续增加
/rtk/correction_age 通常保持在数秒以内
/rtk/fix_quality 最终为 4
/rtk/status 最终为 RTK_FIXED
```

如果 `rtcm_bytes` 停止且 `correction_age` 持续增加，说明 RTCM 已停流。新版
驱动会在 10 秒后打印 `RTCM stream stalled`，随后自动重连。

`/rtk/rtcm_status` 中至少应持续出现基准站坐标类消息 `1005` 或 `1006`，以及
一个或多个星座的观测消息，例如 GPS `107x`、GLONASS `108x`、Galileo
`109x`、北斗 `112x`。只有字节增长而没有有效 RTCM3 帧，不能视为差分链健康。

## 长期 RTK_FLOAT 的判断顺序

同一位置、同一账号的一体式 RTK 能固定，而本机长期浮点时，按以下顺序检查：

1. `/rtk/receiver_response` 是否确认接收机为 rover，动态模型是否为 SURVEY。
2. `/rtk/rtcm_status` 是否有连续有效帧及必要的基准站/观测消息。
3. `/rtk/correction_age` 是否通常低于数秒，`invalid_crc` 是否快速增长。
4. 主天线是否接在 UM982 的 ANT1，馈线、供电和天线增益是否正常。
5. 接收机固件、授权和启用频点是否与能固定的一体机一致。

如果前 3 项正常而仍无法固定，问题已经缩小到接收机固件/授权、频点配置或
射频硬件，不应继续通过修改 ROS 状态字段掩盖问题。驱动不会自动修改
`SIGNALGROUP` 或写入 `SAVECONFIG`，这两项必须在确认设备版本和官方配置后再做。

官方参考：

- [Unicore N4 高精度产品命令手册](https://en.unicore.com/uploads/file/unicore-reference-commands-manual-for-n4-high-precision-products-v2-en-r1.2.pdf)
- [UM982 用户手册](https://en.unicore.com/uploads/file/um982-user-manual-en-r1-1.pdf)

不要把 `NO_FIX`、`SINGLE`、`DGNSS` 或 `RTK_FLOAT` 当作厘米级导航输入。
农业导航适配包默认只接受 `RTK_FIXED`。
