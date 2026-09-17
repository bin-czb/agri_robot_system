# RTK/AOA 二维位置观测选择器

`agri_localization_selector` 对 RTK 与 AOA 的**地图系二维位置观测**进行质量门控和互斥选择，发布 `/global_pose/selected`、`/global_pose/selection_status` 和 `/global_pose/usable`。输出不是连续导航位姿，不提供可信高度、航向、TF 或底盘速度命令。

## 结构与启动

输入位置：`/global_pose/rtk`、`/global_pose/aoa_raw_map`。对应质量证据：`/rtk/fix`、`/aoa/observation`，AOA 还需要 `/uav/localization_health`。位置与对应质量证据按完全相同的消息时间戳配对；候选来源必须连续合格，并经过时间、协方差、运动可达性及交接一致性检查。RTK 优先，失效后允许 AOA 接管，RTK 恢复后带滞回切回。

从仓库根目录构建时，先构建并加载 `chassis_ws`（提供 `trunk_interfaces`）和 `rtk_ws`（提供 `rtk_interfaces`），再构建此包：

```bash
source /opt/ros/humble/setup.bash
source chassis_ws/install/setup.bash
source rtk_ws/install/setup.bash
cd navigation_ws
colcon build --symlink-install --packages-select agri_localization_selector
source install/setup.bash
ros2 launch agri_localization_selector switching_localization.launch.py
```

`switching_localization.launch.py` 只启动 RTK 地图适配、AOA 地图适配和选择器；RTK 接收机、AOA 驱动、无人机定位等上游节点仍需分别启动。此入口会关闭 RTK 适配器对 `/global_pose/selected` 的直接发布，避免多个最终位置发布者。不要同时使用会直接发布同一最终话题的旧启动方式。

## 默认安全门槛

配置见 [`config/selector.yaml`](config/selector.yaml)。默认 `alignment_verified: false`，`alignment_id` 为空，且 RTK 与 AOA 声明的物理参考点分别为 `gps_link` 与 `sensor_rig`。因此**默认不会放行自动切换结果**。必须先实际完成 ENU/UTM 地图对齐及传感器安装杆臂/参考点校准，再填写校准 ID 和一致的参考点；仅修改字符串标签不构成校准。

默认要求测量龄期不超过 0.6 秒、初次资格持续至少 1 秒且不少于 5 帧；AOA 工作时 RTK 需连续恢复 3 秒，并满足最短驻留 2 秒和交接一致性检查。阈值是待外场标定的工程初值，不代表已有实机性能验证。状态只有 `RTK_ACTIVE`、`AOA_ACTIVE` 和 `GLOBAL_UNAVAILABLE`；下游必须同时检查 `/global_pose/usable` 与测量时间戳，不能在失效后继续使用最后一条位置。

重置服务为 `/localization_selector/reset`，会清空缓存证据和交接锚点，随后等待新的连续合格观测。切换逻辑的自动化测试位于 `test/`，不替代真实 RTK/AOA 联调。
