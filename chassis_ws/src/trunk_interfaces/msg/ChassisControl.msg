# 底盘控制指令消息
std_msgs/Header header

# 基础运动控制
float64 linear_velocity     # 线速度 (m/s)
float64 angular_velocity    # 角速度 (rad/s)

# 履带底盘特定控制
float64 left_track_speed    # 左履带速度 (-1.0 到 1.0)
float64 right_track_speed   # 右履带速度 (-1.0 到 1.0)

# 控制模式
int32 control_mode          # 0: 停止, 1: 跟踪模式, 2: 手动模式
bool emergency_stop         # 紧急停止标志

# 目标距离信息 (用于速度调节)
float64 target_distance     # 目标距离 (m)
float64 min_distance        # 最小安全距离 (m)

