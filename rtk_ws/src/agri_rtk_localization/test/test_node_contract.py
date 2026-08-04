import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import UInt8

from agri_rtk_localization.rtk_localizer import RtkLocalizer


def test_fixed_navsat_message_initializes_pose_and_path():
    rclpy.init()
    node = RtkLocalizer(parameter_overrides=[
        Parameter('auto_datum_samples', value=1),
        Parameter('require_rtk_fixed', value=True),
    ])
    try:
        node._quality_callback(UInt8(data=4))
        fix = NavSatFix()
        fix.header.stamp.sec = 1
        fix.header.frame_id = 'gps_link'
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix.status.service = 15
        fix.latitude = 30.0
        fix.longitude = 114.0
        fix.altitude = 50.0
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        node._fix_callback(fix)

        assert node.projection is not None
        assert node.accepted == 1
        assert node.rejected == 0
        assert node.filter.position is not None
        assert len(node.path) == 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
