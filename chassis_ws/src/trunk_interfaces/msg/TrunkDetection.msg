# 树干检测结果消息
std_msgs/Header header

# 检测框信息
geometry_msgs/Point[] detection_points  # 检测框底部中点坐标
float32[] confidences                   # 对应的置信度
geometry_msgs/Polygon[] bounding_boxes  # 检测框坐标 (x_min, y_min, x_max, y_max)

# 图像信息
int32 image_width
int32 image_height

# 检测状态
bool detection_valid
int32 trunk_count

