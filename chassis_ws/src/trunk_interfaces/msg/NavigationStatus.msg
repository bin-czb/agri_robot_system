# 导航状态消息
std_msgs/Header header

# 导航状态
int32 navigation_state      # 0: 停止, 1: 搜索目标, 2: 跟踪中, 3: 到达目标, 4: 错误
string state_description    # 状态描述

# 跟踪信息
bool target_detected
float64 distance_to_target  # 估计距离到目标
float64 angle_to_target     # 目标相对角度

# 性能指标
float64 detection_fps       # 检测帧率
float64 control_frequency   # 控制频率
builtin_interfaces/Duration total_runtime  # 总运行时间

# 错误信息
bool has_error
string error_message

