# 系统数据

- `calibration/`：已接受的相机、IMU、深度和刚性组件标定记录。
- `maps/`：独立地图交付物；当前运行地图仍随 `agri_map_integration` 包安装。
- `models/`：模型归档；当前树干模型随 `trunk_detection` 包安装。
- `rosbags/`：实验录制数据，不提交到普通 Git。

运行生成的数据不得放入任意工作空间的 `src/`。
