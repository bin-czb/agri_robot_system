"""Pure quality checks for the existing stamped sensor contracts."""

import math


def rtk_reason(fix, max_correction_age=5.0, max_hdop=2.5):
    if not fix.position_valid or not fix.rtk_fixed or fix.fix_quality != 4:
        return 'rtk_not_fixed'
    if not math.isfinite(fix.correction_age) or not 0 <= fix.correction_age <= max_correction_age:
        return 'rtk_correction_age_invalid'
    if not math.isfinite(fix.hdop) or not 0 < fix.hdop <= max_hdop:
        return 'rtk_hdop_invalid'
    if fix.satellites < 5:
        return 'rtk_satellites_insufficient'
    return ''


def aoa_reason(obs):
    if obs.gate_code != 0 or obs.link_status != 2:
        return 'aoa_driver_rejected'
    if not all(math.isfinite(v) for v in (
            obs.range_calibrated_m, obs.azimuth_body_rad, obs.base_yaw_enu_rad)):
        return 'aoa_geometry_or_heading_invalid'
    if obs.range_calibrated_m <= 0:
        return 'aoa_range_invalid'
    # Elevation is reserved in ALX-AOA-FIT V1.2. Never assume it is valid.
    return ''


def uav_reason(health, now_s, max_age=0.5):
    try:
        values = [float(health[k]) for k in ('stamp_s', 'gps_age_s', 'attitude_age_s')]
        if not all(math.isfinite(v) for v in values):
            return 'uav_nonfinite_health'
        if not -0.05 <= now_s - values[0] <= max_age:
            return 'uav_position_stale'
        if not 0 <= values[1] <= max_age or not 0 <= values[2] <= max_age:
            return 'uav_gps_or_attitude_stale'
        if health['fix_type'] != 6:
            return 'uav_not_rtk_fixed'
    except (KeyError, TypeError, ValueError, OverflowError):
        return 'uav_health_invalid'
    return ''
