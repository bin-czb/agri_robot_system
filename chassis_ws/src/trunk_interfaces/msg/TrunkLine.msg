# 树干拟合直线消息
std_msgs/Header header

# 直线方程 ax + by + c = 0
float64 coeff_a
float64 coeff_b
float64 coeff_c

# 树干中心坐标 (图像像素坐标)
float64 trunk_center_x
float64 trunk_center_y

# 拟合质量信息
float64 fitting_error
bool line_valid
int32 points_used_for_fitting

# 直线角度信息
float64 line_angle_rad      # 直线与水平方向夹角 (弧度)
float64 steering_angle_rad  # 建议转向角度 (弧度)
