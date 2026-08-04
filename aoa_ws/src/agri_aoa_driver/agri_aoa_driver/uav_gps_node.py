#!/usr/bin/env python3
"""
UAV GPS/RTK + Full Attitude Node.

Reads from MAVLink:
  - GLOBAL_POSITION_INT → lat, lon, relative_alt (EKF-fused altitude)
  - ATTITUDE            → roll, pitch, yaw  (CRITICAL for UWB anchor transform)
  - GPS_RAW_INT         → pure GPS/RTK altitude + fix_type
                          (only when publish_rtk_altitude is True)

Publishes (always):
  /uav/utm_pose     (PoseStamped) — position (UTM, fused altitude) +
                    full orientation quaternion

Publishes (optional, gated by publish_rtk_altitude parameter):
  /uav/altitude_agl (Float64) — pure RTK altitude referenced to the
                    altitude of the takeoff / first lock moment.

  The single MAVLink serial port physically cannot be shared between
  two processes, so rather than spawning a separate rtk_altitude_node
  we publish the RTK-only altitude from this same node.  UWB and AOA
  consumers already have dedicated-altitude subscribers with graceful
  fallback, so adding this publisher is a drop-in improvement for
  whichever launch enables it.  Default is OFF so the UWB launch
  script keeps the previous behaviour without changes.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from pymavlink import mavutil
import utm
import time
import threading
import math
from collections import deque
from scipy.spatial.transform import Rotation as R


# Max acceptable ATTITUDE age (at the moment we publish pose) before
# we log a warning.  This is a diagnostic only — the quaternion is
# still written using whatever attitude we have cached.  The number
# is conservative: at typical ATTITUDE rates (10 Hz) a healthy link
# should produce <110 ms gaps; 150 ms flags a slow stream or a
# momentary dropout.
ATT_FRESH_WARN_SEC = 0.15


# MAVLink GPS_FIX_TYPE labels (for log readability).  Values match the
# MAVLink spec:
#   0 NO_GPS, 1 NO_FIX, 2 2D, 3 3D, 4 DGPS, 5 RTK_FLOAT, 6 RTK_FIXED,
#   7 STATIC, 8 PPP
GPS_FIX_TYPE_LABELS = {
    0: 'NO_GPS', 1: 'NO_FIX', 2: '2D', 3: '3D', 4: 'DGPS',
    5: 'RTK_FLOAT', 6: 'RTK_FIXED', 7: 'STATIC', 8: 'PPP',
}


def _fix_label(ft: int) -> str:
    return GPS_FIX_TYPE_LABELS.get(ft, f'UNK({ft})')


# How many consecutive "good-enough fix" samples we average to
# establish the home altitude, and how tight their vertical scatter
# must be before we commit.  Mirrors the origin-lock pattern used in
# the localization nodes: prevents a single bad reading from biasing
# the entire flight.
RTK_HOME_WARMUP_SAMPLES = 5
RTK_HOME_WARMUP_MAX_SCATTER_M = 0.20
# Throttle for "RTK altitude waiting for fix" warnings.
RTK_WAIT_WARN_PERIOD_SEC = 3.0


class UAVGPSNode(Node):
    def __init__(self):
        super().__init__('uav_gps_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 57600)
        # RTK altitude publishing — OFF by default to preserve the
        # exact legacy behaviour of launch_system.sh (UWB).  Enable from
        # launch_aoa.sh to let the AOA node feed on pure RTK altitude.
        self.declare_parameter('publish_rtk_altitude', False)
        # Minimum GPS_FIX_TYPE we trust for altitude:
        #   6 = RTK_FIXED  (~1–5 cm vertical, default)
        #   5 = RTK_FLOAT  (~20–50 cm)
        #   3 = 3D fix     (standalone GPS, ~m-level; NOT recommended)
        self.declare_parameter('rtk_min_fix_type', 6)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.publish_rtk_alt = self.get_parameter('publish_rtk_altitude').value
        self.rtk_min_fix_type = int(self.get_parameter('rtk_min_fix_type').value)

        self.publisher_ = self.create_publisher(PoseStamped, '/uav/utm_pose', 10)
        # The altitude publisher is always created (so ros2 topic tools
        # see it for discovery even when no RTK fix yet), but we only
        # actually publish to it while RTK is healthy AND the feature
        # is enabled by parameter.  When disabled this publisher stays
        # silent and the existing pose-altitude fallback in AOA / UWB
        # kicks in unchanged.
        self.alt_publisher_ = self.create_publisher(
            Float64, '/uav/altitude_agl', 10)

        self.mav_conn = None
        self.running = True

        # RTK altitude state (only consulted when publish_rtk_alt=True).
        # _rtk_home_alt_m is the MSL altitude locked at takeoff; all
        # subsequent Float64 messages carry (current_msl - home_msl),
        # matching the "relative to home" convention the rest of the
        # stack expects.  Once locked we never unlock it: if RTK drops
        # and re-acquires, we resume publishing with the same home so
        # the AOA / UWB altitude-origin machinery does not see a jump.
        self._rtk_home_alt_m: 'float|None' = None
        self._rtk_warmup_buf = deque(maxlen=RTK_HOME_WARMUP_SAMPLES)
        self._rtk_last_fix_type = -1
        self._rtk_wait_last_warn = 0.0
        self._rtk_publish_count = 0

        # Cached attitude from ATTITUDE message (NED frame: roll, pitch, yaw in rad)
        self.att_roll  = 0.0
        self.att_pitch = 0.0
        self.att_yaw   = 0.0
        self.att_valid  = False
        # Wall-clock time we last received an ATTITUDE MAVLink message.
        # Used to expose the GPS↔attitude asynchrony every time we
        # publish a pose — we cannot truly sync the two streams from
        # inside this reader, but we can at least log the skew so
        # downstream consumers (AOA, UWB) know when to distrust yaw.
        self.att_recv_time = 0.0
        # Throttle "attitude stale" warn log so we don't spam it on
        # every GLOBAL_POSITION_INT event during a real outage.
        self._att_stale_last_warn = 0.0

        self.thread = threading.Thread(target=self.mavlink_loop)
        self.thread.start()

        self.get_logger().info(
            f'UAV GPS Node started. Connecting to {self.port}:{self.baud}...')
        if self.publish_rtk_alt:
            self.get_logger().info(
                f'[RTK-ALT] publish_rtk_altitude=True, '
                f'min_fix_type={self.rtk_min_fix_type} '
                f'({_fix_label(self.rtk_min_fix_type)}). '
                f'/uav/altitude_agl will carry pure GPS/RTK altitude '
                f'(MSL-zeroed at first {RTK_HOME_WARMUP_SAMPLES}-sample '
                f'lock).  Pose fallback in consumers handles RTK loss.')
        else:
            self.get_logger().info(
                '[RTK-ALT] publish_rtk_altitude=False (default) — '
                '/uav/altitude_agl will stay silent; downstream nodes '
                'use fused pose altitude.')

    def mavlink_loop(self):
        while self.running and rclpy.ok():
            try:
                if self.mav_conn is None:
                    try:
                        self.mav_conn = mavutil.mavlink_connection(
                            self.port, baud=self.baud)
                        self.get_logger().info(f'MAVLink connected on {self.port}')
                        self.mav_conn.mav.request_data_stream_send(
                            self.mav_conn.target_system,
                            self.mav_conn.target_component,
                            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
                    except Exception as e:
                        self.get_logger().error(f'MAVLink connect fail: {e}')
                        time.sleep(2.0)
                        continue

                recv_types = ['GLOBAL_POSITION_INT', 'ATTITUDE']
                if self.publish_rtk_alt:
                    recv_types.append('GPS_RAW_INT')

                msg = self.mav_conn.recv_match(
                    type=recv_types,
                    blocking=True, timeout=1.0)

                if not msg:
                    continue

                if msg.get_type() == 'ATTITUDE':
                    self.att_roll  = msg.roll     # rad, NED body
                    self.att_pitch = msg.pitch    # rad, NED body
                    self.att_yaw   = msg.yaw      # rad, NED body (-π..π)
                    self.att_recv_time = time.monotonic()
                    if not self.att_valid:
                        self.att_valid = True
                        self.get_logger().info(
                            f'Attitude stream active: '
                            f'R={math.degrees(msg.roll):.1f}° '
                            f'P={math.degrees(msg.pitch):.1f}° '
                            f'Y={math.degrees(msg.yaw):.1f}°')

                elif msg.get_type() == 'GLOBAL_POSITION_INT':
                    self.publish_pose(msg)

                elif msg.get_type() == 'GPS_RAW_INT':
                    self.handle_gps_raw(msg)

            except Exception as e:
                self.get_logger().error(f'MAVLink loop error: {e}')
                self.mav_conn = None
                time.sleep(1.0)

    def publish_pose(self, msg):
        try:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt_rel = msg.relative_alt / 1000.0

            utm_e, utm_n, zone_num, zone_let = utm.from_latlon(lat, lon)

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'

            pose_msg.pose.position.x = utm_e
            pose_msg.pose.position.y = utm_n
            pose_msg.pose.position.z = alt_rel

            # --- Full 3-axis quaternion ---
            # MAVLink ATTITUDE gives NED-frame roll/pitch/yaw.
            # Convert NED yaw → ENU yaw:  enu_yaw = π/2 - ned_yaw
            # Roll and pitch stay the same sign convention for FLU body.
            # NOTE: this is still a known approximation — MAVLink uses
            # FRD body axes while ROS expects FLU, which implies a
            # pitch/roll sign flip.  That correction is intentionally
            # NOT applied here yet because it would require a regression
            # sweep against the existing UWB anchor transform.  The
            # freshness log below gives you the information you need
            # to decide when to revisit the issue.
            if self.att_valid:
                att_age = time.monotonic() - self.att_recv_time
                now_mono = time.monotonic()
                if (att_age > ATT_FRESH_WARN_SEC
                        and now_mono - self._att_stale_last_warn > 2.0):
                    self.get_logger().warn(
                        f'ATTITUDE is {att_age*1000:.0f} ms old at '
                        f'publish time (> {ATT_FRESH_WARN_SEC*1000:.0f} ms); '
                        f'yaw used in pose may lag during fast slewing')
                    self._att_stale_last_warn = now_mono

                enu_yaw = math.pi / 2.0 - self.att_yaw
                quat = R.from_euler('ZYX',
                    [enu_yaw, self.att_pitch, self.att_roll]).as_quat()
            else:
                heading_deg = msg.hdg / 100.0
                enu_yaw = math.pi / 2.0 - math.radians(heading_deg)
                quat = R.from_euler('z', enu_yaw).as_quat()

            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]

            self.publisher_.publish(pose_msg)

        except Exception as e:
            self.get_logger().warn(f'Pose publish error: {e}')

    def handle_gps_raw(self, msg):
        """Emit pure RTK altitude on /uav/altitude_agl.

        Source of truth: GPS_RAW_INT.alt (mm, MSL).  We subtract a
        one-time locked "home MSL" so the published Float64 matches the
        semantics downstream already expects ("altitude above takeoff
        point", i.e. rises to 10 m when the UAV is 10 m up).

        RTK lifecycle handling
        ----------------------
        * While fix_type < rtk_min_fix_type we publish NOTHING.  The AOA
          / UWB altitude watchdog (ALTITUDE_TIMEOUT in aoa_config) will
          expire within ~1 s and the pose-altitude fallback takes over.
        * The first rtk_min_fix_type samples with <20 cm vertical
          scatter establish home_alt_m.  This filters out the single
          spuriously-good sample that can appear during float→fixed
          transitions.
        * If RTK re-drops to float/none after home has been locked we
          simply stop publishing; home is NEVER unlocked, so a
          subsequent re-acquisition continues seamlessly in the same
          reference frame.

        Fix-type log lines are edge-triggered (only on transition) so
        that a steady RTK_FIXED link stays quiet.
        """
        try:
            fix_type = int(msg.fix_type)
            alt_msl_m = msg.alt / 1000.0       # MAVLink: alt is mm MSL

            # Edge-triggered fix-type transition log
            if fix_type != self._rtk_last_fix_type:
                self.get_logger().info(
                    f'[RTK-ALT] fix_type {_fix_label(self._rtk_last_fix_type)} '
                    f'→ {_fix_label(fix_type)} '
                    f'(alt_msl={alt_msl_m:+.2f} m)')
                self._rtk_last_fix_type = fix_type

            if fix_type < self.rtk_min_fix_type:
                # Below quality threshold — reset warmup buffer (so we
                # don't average pre-fix samples into home_alt), leave
                # any already-locked home intact, and DO NOT publish.
                if self._rtk_warmup_buf:
                    self._rtk_warmup_buf.clear()
                now = time.monotonic()
                if (self._rtk_home_alt_m is None
                        and now - self._rtk_wait_last_warn
                        > RTK_WAIT_WARN_PERIOD_SEC):
                    self.get_logger().warn(
                        f'[RTK-ALT] waiting for fix '
                        f'≥ {_fix_label(self.rtk_min_fix_type)} '
                        f'(have {_fix_label(fix_type)}); '
                        f'/uav/altitude_agl silent, consumers use pose '
                        f'altitude fallback')
                    self._rtk_wait_last_warn = now
                return

            # Quality OK — if home not yet locked, accumulate warmup.
            if self._rtk_home_alt_m is None:
                self._rtk_warmup_buf.append(alt_msl_m)
                if len(self._rtk_warmup_buf) >= RTK_HOME_WARMUP_SAMPLES:
                    samples = list(self._rtk_warmup_buf)
                    scatter = max(samples) - min(samples)
                    if scatter <= RTK_HOME_WARMUP_MAX_SCATTER_M:
                        self._rtk_home_alt_m = sum(samples) / len(samples)
                        self.get_logger().info(
                            f'[RTK-ALT] Home altitude locked: '
                            f'{self._rtk_home_alt_m:+.3f} m MSL '
                            f'({RTK_HOME_WARMUP_SAMPLES} samples, '
                            f'scatter={scatter*100:.1f} cm, '
                            f'fix={_fix_label(fix_type)}).  '
                            f'/uav/altitude_agl now live.')
                    else:
                        # Drop the oldest sample and keep trying until
                        # a tight window lands.  This prevents a gentle
                        # drift during GPS warmup from ever locking.
                        self._rtk_warmup_buf.popleft()
                return

            # Home locked, fix OK → publish.
            rel_alt = alt_msl_m - self._rtk_home_alt_m
            out = Float64()
            out.data = float(rel_alt)
            self.alt_publisher_.publish(out)
            self._rtk_publish_count += 1

        except Exception as e:
            self.get_logger().warn(f'GPS_RAW handler error: {e}')

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        if self.mav_conn:
            self.mav_conn.close()

def main(args=None):
    rclpy.init(args=args)
    node = UAVGPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Wrap each teardown step so launch-script shutdown ordering
        # cannot produce duplicate-shutdown tracebacks.
        try:
            node.stop()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()








