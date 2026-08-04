std_msgs/Header header

# Raw and calibrated AOA observation. Angles use radians.
float64 range_raw_m
float64 range_calibrated_m
float64 azimuth_raw_rad
float64 azimuth_body_rad
float64 elevation_raw_rad

uint32 tag_status
uint32 sequence
uint32 anchor_id
uint8 gate_code
uint8 link_status

# NaN until the corresponding heading is available.
float64 base_yaw_enu_rad
float64 azimuth_enu_rad
