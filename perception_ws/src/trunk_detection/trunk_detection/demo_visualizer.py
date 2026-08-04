#!/usr/bin/env python3
"""Demo Visualizer for the trunk tracking navigation stack.

This node is a standalone visualization dashboard intended for live demos.
It only reads sensor and perception topics and does NOT depend on the
chassis (no /cmd_vel, no chassis control). The robot can therefore be
driven manually via remote control while this dashboard runs.

Subscriptions:
  sensor_msgs/Image          /camera/color/image_raw
  trunk_interfaces/TrunkDetection /trunk_detection/detections
  trunk_interfaces/TrunkLine      /trunk_detection/trunk_line
  sensor_msgs/Imu            /camera/gyro/sample   (Orbbec Gemini 330)
  sensor_msgs/Imu            /camera/accel/sample  (Orbbec Gemini 330)
  sensor_msgs/LaserScan      /scan                 (2D LiDAR)

The dashboard renders on a fixed schedule (default 15 FPS) so it never
blocks YOLO inference, and reports per-stream Hz / staleness so the
operator can verify the pipeline at a glance.
"""

import math
import time
import threading
from collections import deque

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, Imu, LaserScan
from trunk_interfaces.msg import TrunkDetection, TrunkLine


# BGR colors for a dark, instrument-panel look
COLOR_BG = (24, 24, 28)
COLOR_PANEL = (36, 36, 42)
COLOR_HEADER = (28, 70, 56)
COLOR_DIVIDER = (60, 60, 70)
COLOR_TEXT = (235, 235, 235)
COLOR_LABEL = (170, 170, 180)
COLOR_GOOD = (80, 220, 120)
COLOR_BAD = (60, 80, 240)
COLOR_WARN = (60, 200, 240)
COLOR_BOX = (80, 220, 120)
COLOR_POINT = (60, 200, 240)
COLOR_LINE = (240, 160, 60)
COLOR_TARGET = (60, 220, 240)
COLOR_CENTER = (200, 200, 200)
COLOR_LIDAR_BG = (18, 18, 22)
COLOR_LIDAR_RING = (55, 55, 65)
COLOR_LIDAR_AXIS = (70, 70, 80)
COLOR_LIDAR_POINT = (80, 200, 255)
COLOR_LIDAR_ROBOT = (120, 220, 140)


class RateTracker:
    """Sliding-window message rate estimator."""

    def __init__(self, window: float = 2.0):
        self.window = window
        self.ts: deque = deque()

    def tick(self) -> None:
        t = time.monotonic()
        self.ts.append(t)
        cutoff = t - self.window
        while self.ts and self.ts[0] < cutoff:
            self.ts.popleft()

    def hz(self) -> float:
        if len(self.ts) < 2:
            return 0.0
        span = self.ts[-1] - self.ts[0]
        if span <= 1e-6:
            return 0.0
        return (len(self.ts) - 1) / span

    def age(self) -> float:
        if not self.ts:
            return float('inf')
        return time.monotonic() - self.ts[-1]


class DemoVisualizer(Node):
    def __init__(self):
        super().__init__('demo_visualizer')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('detection_topic', '/trunk_detection/detections')
        self.declare_parameter('line_topic', '/trunk_detection/trunk_line')
        self.declare_parameter('gyro_topic', '/camera/gyro/sample')
        self.declare_parameter('accel_topic', '/camera/accel/sample')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('lidar_max_range_m', 5.0)
        self.declare_parameter('lidar_view_height', 280)
        self.declare_parameter('display_fps', 15.0)
        self.declare_parameter('display_image_width', 720)
        self.declare_parameter('window_name', 'Trunk Tracking - Demo Dashboard')

        self.image_topic = self.get_parameter('image_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.line_topic = self.get_parameter('line_topic').value
        self.gyro_topic = self.get_parameter('gyro_topic').value
        self.accel_topic = self.get_parameter('accel_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.lidar_max_range_m = float(self.get_parameter('lidar_max_range_m').value)
        self.lidar_view_height = int(self.get_parameter('lidar_view_height').value)
        self.display_fps = float(self.get_parameter('display_fps').value)
        self.display_image_width = int(self.get_parameter('display_image_width').value)
        self.window_name = self.get_parameter('window_name').value

        self.cv_bridge = CvBridge()

        self.lock = threading.Lock()
        self.latest_image = None
        self.latest_detection = None
        self.latest_line = None
        self.latest_gyro = None
        self.latest_accel = None
        self.latest_scan = None

        self.image_rate = RateTracker()
        self.det_rate = RateTracker()
        self.line_rate = RateTracker()
        self.gyro_rate = RateTracker()
        self.accel_rate = RateTracker()
        self.scan_rate = RateTracker()
        self.render_rate = RateTracker()

        # BEST_EFFORT matches the publishers in this stack (camera, YOLO, fitter)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.image_topic, self.image_cb, qos)
        self.create_subscription(TrunkDetection, self.detection_topic, self.det_cb, qos)
        self.create_subscription(TrunkLine, self.line_topic, self.line_cb, qos)
        self.create_subscription(Imu, self.gyro_topic, self.gyro_cb, qos)
        self.create_subscription(Imu, self.accel_topic, self.accel_cb, qos)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos)

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        period = 1.0 / max(self.display_fps, 1.0)
        self.create_timer(period, self.render_cb)

        self.t0 = time.monotonic()
        self.get_logger().info(
            f'Demo Visualizer ready. image={self.image_topic} '
            f'det={self.detection_topic} line={self.line_topic} '
            f'gyro={self.gyro_topic} accel={self.accel_topic} '
            f'scan={self.scan_topic} '
            f'display_fps={self.display_fps}'
        )

    # -------------------- callbacks --------------------

    def image_cb(self, msg: Image) -> None:
        try:
            img = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'image convert failed: {e}')
            return
        with self.lock:
            self.latest_image = img
        self.image_rate.tick()

    def det_cb(self, msg: TrunkDetection) -> None:
        with self.lock:
            self.latest_detection = msg
        self.det_rate.tick()

    def line_cb(self, msg: TrunkLine) -> None:
        with self.lock:
            self.latest_line = msg
        self.line_rate.tick()

    def gyro_cb(self, msg: Imu) -> None:
        with self.lock:
            self.latest_gyro = msg
        self.gyro_rate.tick()

    def accel_cb(self, msg: Imu) -> None:
        with self.lock:
            self.latest_accel = msg
        self.accel_rate.tick()

    def scan_cb(self, msg: LaserScan) -> None:
        with self.lock:
            self.latest_scan = msg
        self.scan_rate.tick()

    # -------------------- render --------------------

    def render_cb(self) -> None:
        with self.lock:
            image = None if self.latest_image is None else self.latest_image.copy()
            detection = self.latest_detection
            line = self.latest_line
            gyro = self.latest_gyro
            accel = self.latest_accel
            scan = self.latest_scan

        canvas = self._compose_canvas(image, detection, line, gyro, accel, scan)

        try:
            cv2.imshow(self.window_name, canvas)
            key = cv2.waitKey(1) & 0xFF
        except cv2.error as e:
            self.get_logger().error(f'cv2 display error: {e}')
            rclpy.shutdown()
            return

        self.render_rate.tick()
        if key in (ord('q'), 27):
            self.get_logger().info('quit requested by user')
            rclpy.shutdown()

    # -------------------- canvas --------------------

    def _compose_canvas(self, image, detection, line, gyro, accel, scan):
        target_w = self.display_image_width
        if image is None:
            img_w = target_w
            img_h = int(target_w * 9 / 16)
            disp = np.full((img_h, img_w, 3), 48, dtype=np.uint8)
            cv2.putText(disp, 'Waiting for camera image...', (30, img_h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, COLOR_LABEL, 2, cv2.LINE_AA)
            orig_w, orig_h = img_w, img_h
            scale = 1.0
        else:
            orig_h, orig_w = image.shape[:2]
            scale = target_w / float(orig_w)
            img_w = target_w
            img_h = int(round(orig_h * scale))
            disp = cv2.resize(image, (img_w, img_h), interpolation=cv2.INTER_LINEAR)

        self._draw_image_overlay(disp, detection, line, orig_w, orig_h, scale)

        header_h = 56
        footer_h = 32
        panel_w = 360
        lidar_h = self.lidar_view_height
        gap_h = 8
        left_h = img_h + gap_h + lidar_h
        min_panel_h = 720
        content_h = max(left_h, min_panel_h)
        canvas_w = img_w + panel_w
        canvas_h = header_h + content_h + footer_h

        canvas = np.full((canvas_h, canvas_w, 3), COLOR_BG, dtype=np.uint8)

        # Header bar
        cv2.rectangle(canvas, (0, 0), (canvas_w, header_h), COLOR_HEADER, -1)
        cv2.putText(canvas, 'TRUNK TRACKING NAVIGATION  -  DEMO DASHBOARD',
                    (20, 36), cv2.FONT_HERSHEY_DUPLEX, 0.85, COLOR_TEXT, 1, cv2.LINE_AA)

        clock = time.strftime('%H:%M:%S')
        uptime = int(time.monotonic() - self.t0)
        right_text = f'{clock}  |  UPTIME {uptime:05d}s  |  RENDER {self.render_rate.hz():4.1f} Hz'
        (tw, _), _ = cv2.getTextSize(right_text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.putText(canvas, right_text, (canvas_w - tw - 20, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, COLOR_TEXT, 1, cv2.LINE_AA)

        # Image area
        canvas[header_h:header_h + img_h, 0:img_w] = disp
        cv2.line(canvas, (img_w, header_h), (img_w, header_h + content_h),
                 COLOR_DIVIDER, 1)

        # LIDAR view (directly below the camera image)
        lidar_y0 = header_h + img_h + gap_h
        lidar_y1 = lidar_y0 + lidar_h
        if lidar_y1 <= canvas_h - footer_h:
            lidar_view = canvas[lidar_y0:lidar_y1, 0:img_w]
            lidar_view[:] = COLOR_LIDAR_BG
            self._draw_lidar(lidar_view, scan)
            cv2.rectangle(canvas, (0, lidar_y0 - 1),
                          (img_w, lidar_y1), COLOR_DIVIDER, 1)

        # Side panel
        panel_y0 = header_h
        panel_y1 = header_h + content_h
        panel_x0 = img_w + 1
        panel = canvas[panel_y0:panel_y1, panel_x0:canvas_w]
        panel[:] = COLOR_PANEL
        self._draw_side_panel(panel, detection, line, gyro, accel)

        # Footer
        cv2.rectangle(canvas, (0, canvas_h - footer_h), (canvas_w, canvas_h),
                      COLOR_PANEL, -1)
        cv2.putText(canvas,
                    'Press Q or ESC to quit  |  Sensor-only demo: chassis not required',
                    (20, canvas_h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_LABEL, 1, cv2.LINE_AA)

        return canvas

    # -------------------- image overlay --------------------

    def _draw_image_overlay(self, disp, detection, line, orig_w, orig_h, scale):
        h, w = disp.shape[:2]
        center_x = w // 2
        cv2.line(disp, (center_x, 0), (center_x, h), COLOR_CENTER, 1, cv2.LINE_AA)

        if detection is not None:
            for box, conf in zip(detection.bounding_boxes, detection.confidences):
                if len(box.points) < 4:
                    continue
                xs = [p.x * scale for p in box.points]
                ys = [p.y * scale for p in box.points]
                p1 = (int(min(xs)), int(min(ys)))
                p2 = (int(max(xs)), int(max(ys)))
                cv2.rectangle(disp, p1, p2, COLOR_BOX, 2)
                label = f'trunk {conf:.2f}'
                (tw_, th_), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                tx, ty = p1[0], max(0, p1[1] - 6)
                cv2.rectangle(disp, (tx, ty - th_ - 4), (tx + tw_ + 6, ty + 2),
                              COLOR_BOX, -1)
                cv2.putText(disp, label, (tx + 3, ty - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (10, 30, 10), 1, cv2.LINE_AA)
            for pt in detection.detection_points:
                px = int(pt.x * scale)
                py = int(pt.y * scale)
                cv2.circle(disp, (px, py), 6, COLOR_POINT, -1, cv2.LINE_AA)
                cv2.circle(disp, (px, py), 7, (30, 30, 30), 1, cv2.LINE_AA)

        if line is not None and line.line_valid:
            pts = self._line_endpoints(line.coeff_a, line.coeff_b, line.coeff_c,
                                       orig_w, orig_h)
            if pts is not None:
                p1, p2 = pts
                cv2.line(disp,
                         (int(p1[0] * scale), int(p1[1] * scale)),
                         (int(p2[0] * scale), int(p2[1] * scale)),
                         COLOR_LINE, 2, cv2.LINE_AA)
            tx = int(line.trunk_center_x * scale)
            ty = int(line.trunk_center_y * scale)
            tx = max(0, min(w - 1, tx))
            ty = max(0, min(h - 1, ty))
            cv2.drawMarker(disp, (tx, ty), COLOR_TARGET,
                           cv2.MARKER_CROSS, 20, 2, cv2.LINE_AA)
            cv2.line(disp, (center_x, ty), (tx, ty), COLOR_TARGET, 2, cv2.LINE_AA)
            err_px = tx - center_x
            err_label = f'offset {err_px:+d}px'
            anchor_x = min(tx, center_x) + 8
            cv2.putText(disp, err_label, (anchor_x, max(20, ty - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_TARGET, 1, cv2.LINE_AA)

        status_ok = line is not None and line.line_valid
        status = 'TRACKING' if status_ok else 'NO LINE'
        color = COLOR_GOOD if status_ok else COLOR_BAD
        cv2.rectangle(disp, (10, 10), (170, 40), (0, 0, 0), -1)
        cv2.rectangle(disp, (10, 10), (170, 40), color, 1)
        cv2.putText(disp, status, (22, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

    @staticmethod
    def _line_endpoints(a, b, c, w, h):
        """Return two endpoints where ax+by+c=0 meets the image rectangle."""
        pts = []
        if abs(b) > 1e-6:
            for x in (0.0, float(w)):
                y = (-a * x - c) / b
                if -1.0 <= y <= h + 1.0:
                    pts.append((x, y))
        if abs(a) > 1e-6:
            for y in (0.0, float(h)):
                x = (-b * y - c) / a
                if -1.0 <= x <= w + 1.0:
                    pts.append((x, y))
        if len(pts) < 2:
            return None
        pts = sorted(pts, key=lambda p: (p[1], p[0]))
        return pts[0], pts[-1]

    # -------------------- lidar view --------------------

    def _draw_lidar(self, view, scan):
        """Draw a top-down 2D lidar view.

        ROS LaserScan convention: +x forward, +y left, +z up, angle CCW from +x.
        Image convention: +x right, +y down. We map so that forward (+x) is up
        and left (+y) is left in the displayed image.
        """
        h, w = view.shape[:2]
        cx, cy = w // 2, h // 2
        max_r = max(0.5, self.lidar_max_range_m)
        scale = (min(w, h) / 2.0 - 14) / max_r  # pixels per meter

        # Distance rings (integer meters)
        max_ring = int(math.floor(max_r))
        for r in range(1, max_ring + 1):
            cv2.circle(view, (cx, cy), int(r * scale), COLOR_LIDAR_RING, 1,
                       cv2.LINE_AA)
            cv2.putText(view, f'{r}m',
                        (cx + int(r * scale) - 22, cy - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, COLOR_LIDAR_RING, 1,
                        cv2.LINE_AA)

        # Cross axes
        cv2.line(view, (cx, 8), (cx, h - 8), COLOR_LIDAR_AXIS, 1, cv2.LINE_AA)
        cv2.line(view, (8, cy), (w - 8, cy), COLOR_LIDAR_AXIS, 1, cv2.LINE_AA)

        # FRONT marker
        cv2.arrowedLine(view, (cx, cy), (cx, cy - 34),
                        COLOR_LIDAR_ROBOT, 2, cv2.LINE_AA, tipLength=0.3)
        cv2.putText(view, 'FRONT', (cx + 6, cy - 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_LIDAR_ROBOT, 1,
                    cv2.LINE_AA)

        # Title and stats
        cv2.putText(view, 'LIDAR 2D  (top-down)', (10, 18),
                    cv2.FONT_HERSHEY_DUPLEX, 0.5, COLOR_TEXT, 1, cv2.LINE_AA)
        cv2.putText(view, f'range {max_r:.1f} m', (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_LABEL, 1, cv2.LINE_AA)

        # Scan points
        if scan is not None and len(scan.ranges) > 0:
            ranges = np.asarray(scan.ranges, dtype=np.float32)
            n = ranges.shape[0]
            angles = scan.angle_min + np.arange(n) * scan.angle_increment
            valid = np.isfinite(ranges)
            valid &= ranges > max(0.0, scan.range_min)
            valid &= ranges < min(scan.range_max, max_r)
            r_v = ranges[valid]
            a_v = angles[valid]
            xs = r_v * np.cos(a_v)
            ys = r_v * np.sin(a_v)
            px = (cx - ys * scale).astype(np.int32)
            py = (cy - xs * scale).astype(np.int32)
            in_view = (px >= 0) & (px < w) & (py >= 0) & (py < h)
            for x_pt, y_pt in zip(px[in_view], py[in_view]):
                cv2.circle(view, (int(x_pt), int(y_pt)), 2,
                           COLOR_LIDAR_POINT, -1, cv2.LINE_AA)

            stats = f'{int(in_view.sum())} pts / {n} rays'
            (tw_, _), _ = cv2.getTextSize(stats, cv2.FONT_HERSHEY_SIMPLEX,
                                          0.45, 1)
            cv2.putText(view, stats, (w - tw_ - 10, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_LABEL, 1,
                        cv2.LINE_AA)
        else:
            msg = 'waiting for /scan...'
            (tw_, th_), _ = cv2.getTextSize(msg, cv2.FONT_HERSHEY_SIMPLEX,
                                            0.55, 1)
            cv2.putText(view, msg, ((w - tw_) // 2, cy + 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, COLOR_WARN, 1,
                        cv2.LINE_AA)

        # Robot origin marker on top
        cv2.circle(view, (cx, cy), 5, COLOR_LIDAR_ROBOT, -1, cv2.LINE_AA)
        cv2.circle(view, (cx, cy), 5, (255, 255, 255), 1, cv2.LINE_AA)

    # -------------------- side panel --------------------

    def _draw_side_panel(self, panel, detection, line, gyro, accel):
        ph, pw = panel.shape[:2]
        cursor = {'y': 28}

        def section(title: str) -> None:
            cv2.putText(panel, title, (16, cursor['y']),
                        cv2.FONT_HERSHEY_DUPLEX, 0.6, COLOR_TEXT, 1, cv2.LINE_AA)
            cursor['y'] += 8
            cv2.line(panel, (16, cursor['y']), (pw - 16, cursor['y']),
                     COLOR_DIVIDER, 1)
            cursor['y'] += 20

        def kv(key: str, val: str, color=None) -> None:
            cv2.putText(panel, key, (16, cursor['y']),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_LABEL, 1, cv2.LINE_AA)
            cv2.putText(panel, val, (160, cursor['y']),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        color if color is not None else COLOR_TEXT, 1, cv2.LINE_AA)
            cursor['y'] += 22

        def stream_row(label: str, rate: RateTracker) -> None:
            hz = rate.hz()
            age = rate.age()
            if age == float('inf'):
                tag, color = 'NONE', COLOR_BAD
            elif age > 2.0:
                tag, color = 'STALE', COLOR_WARN
            else:
                tag, color = 'OK', COLOR_GOOD
            cv2.putText(panel, label, (16, cursor['y']),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_LABEL, 1, cv2.LINE_AA)
            cv2.putText(panel, tag, (160, cursor['y']),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
            cv2.putText(panel, f'{hz:5.1f} Hz', (230, cursor['y']),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, COLOR_TEXT, 1, cv2.LINE_AA)
            cursor['y'] += 22

        section('PERCEPTION')
        tcount = detection.trunk_count if detection is not None else 0
        kv('Trunks', f'{tcount}', COLOR_GOOD if tcount > 0 else COLOR_LABEL)
        kv('Detect FPS', f'{self.det_rate.hz():5.1f} Hz')
        kv('Frame FPS', f'{self.image_rate.hz():5.1f} Hz')
        cursor['y'] += 6

        section('NAVIGATION LINE')
        if line is not None and line.line_valid:
            kv('Valid', 'YES', COLOR_GOOD)
            kv('Steering',
               f'{line.steering_angle_rad:+.3f} rad '
               f'({math.degrees(line.steering_angle_rad):+.1f} deg)')
            kv('Line angle',
               f'{line.line_angle_rad:+.3f} rad '
               f'({math.degrees(line.line_angle_rad):+.1f} deg)')
            kv('Target',
               f'({line.trunk_center_x:6.1f}, {line.trunk_center_y:6.1f}) px')
            kv('Fit error', f'{line.fitting_error:.2f}')
            kv('Points used', f'{line.points_used_for_fitting}')
        else:
            kv('Valid', 'NO', COLOR_BAD)
            kv('Steering', '---')
            kv('Target', '---')
        cursor['y'] += 6

        section('CAMERA IMU')
        if gyro is not None:
            wx = gyro.angular_velocity.x
            wy = gyro.angular_velocity.y
            wz = gyro.angular_velocity.z
            kv('Yaw rate',   f'{wz:+6.3f} rad/s')
            kv('Pitch rate', f'{wy:+6.3f} rad/s')
            kv('Roll rate',  f'{wx:+6.3f} rad/s')
        else:
            kv('Gyro', 'waiting...', COLOR_WARN)
        if accel is not None:
            ax = accel.linear_acceleration.x
            ay = accel.linear_acceleration.y
            az = accel.linear_acceleration.z
            amag = math.sqrt(ax * ax + ay * ay + az * az)
            kv('Accel X', f'{ax:+6.2f} m/s^2')
            kv('Accel Y', f'{ay:+6.2f} m/s^2')
            kv('Accel Z', f'{az:+6.2f} m/s^2')
            kv('|a|',     f'{amag:6.2f} m/s^2')
        else:
            kv('Accel', 'waiting...', COLOR_WARN)
        cursor['y'] += 6

        section('STREAMS')
        stream_row('Image',  self.image_rate)
        stream_row('Detect', self.det_rate)
        stream_row('Line',   self.line_rate)
        stream_row('Gyro',   self.gyro_rate)
        stream_row('Accel',  self.accel_rate)
        stream_row('Lidar',  self.scan_rate)

    def destroy_node(self) -> bool:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DemoVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
