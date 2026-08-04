#!/usr/bin/env python3
"""
AOA Localization Debug Viewer — Real-time 2D + time-series visualisation.

Independent read-only ROS2 node, purpose-built for the AOA test stack.
Does NOT import anything from systems/uwb/*.  Does NOT compute its own local
coordinates — local positions come from the backend via /uav/local_pose and
/ground_station/absolute_pose_local, guaranteeing a single consistent
local_origin frame.

Layout
    ┌─────────────────────────┬────────────────────────┐
    │ 2D Map (X=East,Y=North) │ Info Panel             │
    │  UAV dot + trail        │   UAV  Local / Map     │
    │  Base dot + trail       │   Base Local / Map     │
    │  UAV↔Base link line     │   Relative ENU         │
    │  Origin cross           │   Bearing / Distance   │
    │                         │   Link Status          │
    │                         │   Gate counters        │
    ├─────────────────────────┴────────────────────────┤
    │  Azimuth(t): raw vs body(=smoothed)  [deg]       │
    ├──────────────────────────────────────────────────┤
    │  Range(t):   raw vs cal             [m]          │
    └──────────────────────────────────────────────────┘

Usage:
    python3 aoa_debug_viewer.py

Dependencies:
    pip3 install pyqtgraph PyQt5
"""

import sys
import threading
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray


# ── UI Config ─────────────────────────────────────────────────
MAX_TRAIL        = 500        # 2D trail length
UI_REFRESH_MS    = 100        # GUI repaint period
GRID_ALPHA       = 60
PLOT_RANGE       = 15.0       # initial ± metres on the 2D map

TIMESERIES_SEC   = 15.0       # sliding window on bottom plots
TIMESERIES_CAP   = 2000       # hard cap on deque length (40 Hz × 15 s ≈ 600, keep headroom)

# ── Gate codes mirror those in aoa_localization_node.py ──────
GATE_NAMES = {
    0: 'OK',
    1: 'POSE_STALE',
    2: 'NO_STATE',
    3: 'RANGE_PHYSICS',
    4: 'RANGE_RATE',
    5: 'ANGLE_RATE',
    6: 'EMA_SING',
    7: 'LOW_ELEVATION',
    8: 'BASE_HEADING_STALE',
}


# ==============================================================
#  ROS2 subscriber (spins in a background thread)
# ==============================================================
class AoaTopicCollector(Node):
    def __init__(self):
        super().__init__('aoa_debug_viewer')
        self.lock = threading.Lock()

        # Cached latest values
        self.uav_local     = None   # /uav/local_pose
        self.gs_local      = None   # /ground_station/absolute_pose_local
        self.uav_abs       = None   # /uav/utm_pose
        self.gs_abs        = None   # /ground_station/absolute_pose_utm
        self.gs_relative   = None   # /ground_station/relative_pose
        self.gs_relative_frame = ''
        self.debug         = None   # /ground_station/debug

        # AOA raw observation cache (from /aoa/measurement)
        self.raw_az_deg    = None
        self.body_az_deg   = None   # = azimuth_body_deg (already sign/offset applied)
        self.r_raw         = None
        self.r_cal         = None
        self.base_yaw_deg  = None
        self.enu_az_deg    = None
        self.last_gate     = None
        self.link_status   = 0

        self.uav_trail = deque(maxlen=MAX_TRAIL)
        self.gs_trail  = deque(maxlen=MAX_TRAIL)

        # Time series: each entry is (t_rel, value)
        self.t0 = time.monotonic()
        self.ts_az_raw    = deque(maxlen=TIMESERIES_CAP)
        self.ts_az_body   = deque(maxlen=TIMESERIES_CAP)
        self.ts_r_raw     = deque(maxlen=TIMESERIES_CAP)
        self.ts_r_cal     = deque(maxlen=TIMESERIES_CAP)

        # Gate counters keyed by gate_code
        self.gate_counts = {k: 0 for k in GATE_NAMES.keys()}
        self.gate_counts_unknown = 0

        # Subscribe to everything we need
        self.create_subscription(
            PoseStamped, '/uav/local_pose', self._uav_local_cb, 10)
        self.create_subscription(
            PoseStamped, '/ground_station/absolute_pose_local', self._gs_local_cb, 10)
        self.create_subscription(
            PoseStamped, '/uav/utm_pose', self._uav_abs_cb, 10)
        self.create_subscription(
            PoseStamped, '/ground_station/absolute_pose_utm', self._gs_abs_cb, 10)
        self.create_subscription(
            PoseStamped, '/ground_station/relative_pose', self._rel_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/ground_station/debug', self._debug_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/aoa/measurement', self._meas_cb, 10)

    # ── callbacks ────────────────────────────────────────────
    def _uav_local_cb(self, msg: PoseStamped):
        p = msg.pose.position
        with self.lock:
            self.uav_local = np.array([p.x, p.y, p.z])
            self.uav_trail.append(self.uav_local[:2].copy())

    def _gs_local_cb(self, msg: PoseStamped):
        p = msg.pose.position
        with self.lock:
            self.gs_local = np.array([p.x, p.y, p.z])
            self.gs_trail.append(self.gs_local[:2].copy())

    def _uav_abs_cb(self, msg: PoseStamped):
        p = msg.pose.position
        with self.lock:
            self.uav_abs = np.array([p.x, p.y, p.z])

    def _gs_abs_cb(self, msg: PoseStamped):
        p = msg.pose.position
        with self.lock:
            self.gs_abs = np.array([p.x, p.y, p.z])

    def _rel_cb(self, msg: PoseStamped):
        p = msg.pose.position
        with self.lock:
            self.gs_relative = np.array([p.x, p.y, p.z])
            self.gs_relative_frame = msg.header.frame_id

    def _debug_cb(self, msg: Float32MultiArray):
        # [0..2]=p_uav_rel (E,N,U) [3]=h-dist [4]=ENU bearing(deg)
        # [5..7]=position sigma [8]=UAV yaw [9]=raw azimuth
        # [10]=link status [11]=ground base yaw
        if len(msg.data) >= 9:
            with self.lock:
                self.debug = list(msg.data)
                if len(msg.data) >= 11:
                    self.link_status = int(msg.data[10])

    def _meas_cb(self, msg: Float32MultiArray):
        # [0]=d_raw_m [1]=d_cal_m [2]=az_raw_deg [3]=az_body_deg
        # [4]=el_raw_deg [5]=tag_status [6]=seq [7]=anchor_id
        # [8]=gate_code [9]=link_status [10]=base_yaw_enu_deg
        # [11]=azimuth_enu_deg
        if len(msg.data) < 10:
            return
        d_raw  = float(msg.data[0])
        d_cal  = float(msg.data[1])
        az_raw = float(msg.data[2])
        az_b   = float(msg.data[3])
        gate   = int(msg.data[8])
        link   = int(msg.data[9])
        base_yaw = float(msg.data[10]) if len(msg.data) >= 12 else None
        enu_az = float(msg.data[11]) if len(msg.data) >= 12 else None

        t_rel = time.monotonic() - self.t0
        with self.lock:
            self.raw_az_deg  = az_raw
            self.body_az_deg = az_b
            self.r_raw       = d_raw
            self.r_cal       = d_cal
            self.last_gate   = gate
            self.link_status = link
            self.base_yaw_deg = base_yaw
            self.enu_az_deg = enu_az

            self.ts_az_raw.append((t_rel, az_raw))
            self.ts_az_body.append((t_rel, az_b))
            self.ts_r_raw.append((t_rel, d_raw))
            self.ts_r_cal.append((t_rel, d_cal))

            if gate in self.gate_counts:
                self.gate_counts[gate] += 1
            else:
                self.gate_counts_unknown += 1

    # ── snapshot ─────────────────────────────────────────────
    def snapshot(self) -> dict:
        with self.lock:
            return {
                'uav_local'        : self.uav_local.copy() if self.uav_local is not None else None,
                'gs_local'         : self.gs_local.copy()  if self.gs_local  is not None else None,
                'uav_abs'          : self.uav_abs.copy()   if self.uav_abs   is not None else None,
                'gs_abs'           : self.gs_abs.copy()    if self.gs_abs    is not None else None,
                'gs_relative'      : self.gs_relative.copy() if self.gs_relative is not None else None,
                'gs_relative_frame': self.gs_relative_frame,
                'debug'            : list(self.debug) if self.debug else None,
                'raw_az'           : self.raw_az_deg,
                'body_az'          : self.body_az_deg,
                'r_raw'            : self.r_raw,
                'r_cal'            : self.r_cal,
                'base_yaw_deg'     : self.base_yaw_deg,
                'enu_az_deg'       : self.enu_az_deg,
                'last_gate'        : self.last_gate,
                'link_status'      : self.link_status,
                'uav_trail'        : np.array(self.uav_trail) if self.uav_trail else None,
                'gs_trail'         : np.array(self.gs_trail)  if self.gs_trail  else None,
                'ts_az_raw'        : np.array(self.ts_az_raw)  if self.ts_az_raw  else None,
                'ts_az_body'       : np.array(self.ts_az_body) if self.ts_az_body else None,
                'ts_r_raw'         : np.array(self.ts_r_raw)   if self.ts_r_raw   else None,
                'ts_r_cal'         : np.array(self.ts_r_cal)   if self.ts_r_cal   else None,
                'gate_counts'      : dict(self.gate_counts),
                'gate_unknown'     : self.gate_counts_unknown,
                't_now_rel'        : time.monotonic() - self.t0,
            }


# ==============================================================
#  PyQt5 + pyqtgraph main window
# ==============================================================
class AoaDebugViewerWindow(QtWidgets.QMainWindow):
    def __init__(self, collector: AoaTopicCollector):
        super().__init__()
        self.collector = collector
        self.setWindowTitle('AOA Localization Debug Viewer')
        self.resize(1280, 820)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(4, 4, 4, 4)

        # ─── Vertical splitter: top=map+info, bottom=timeseries ───
        vsplit = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        root.addWidget(vsplit)

        # ── Top: horizontal splitter (map | info) ────────────
        top_widget = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        vsplit.addWidget(top_widget)

        # 2D map
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.showGrid(x=True, y=True, alpha=GRID_ALPHA / 255.0)
        self.plot_widget.setLabel('bottom', 'X (East)', units='m')
        self.plot_widget.setLabel('left',   'Y (North)', units='m')
        self.plot_widget.setXRange(-PLOT_RANGE, PLOT_RANGE)
        self.plot_widget.setYRange(-PLOT_RANGE, PLOT_RANGE)
        self.plot_widget.addLegend(offset=(10, 10))
        top_widget.addWidget(self.plot_widget)

        self.uav_trail_plot = self.plot_widget.plot(
            [], [], pen=pg.mkPen('#2196F3', width=1.5), name='UAV Trail')
        self.gs_trail_plot = self.plot_widget.plot(
            [], [], pen=pg.mkPen('#4CAF50', width=1.5), name='Base Trail')
        self.uav_dot = self.plot_widget.plot(
            [], [], pen=None, symbol='o', symbolSize=14, symbolBrush='#2196F3',
            name='UAV (tag)')
        self.gs_dot = self.plot_widget.plot(
            [], [], pen=None, symbol='s', symbolSize=14, symbolBrush='#4CAF50',
            name='Base (solved)')
        self.origin_dot = self.plot_widget.plot(
            [0], [0], pen=None, symbol='+', symbolSize=18, symbolBrush='#FF5722',
            name='local_origin')
        self.link_line = self.plot_widget.plot(
            [], [], pen=pg.mkPen('#999999', width=1, style=QtCore.Qt.DotLine))

        # Info panel
        right_panel = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_panel)
        right_layout.setContentsMargins(6, 6, 6, 6)
        top_widget.addWidget(right_panel)

        self.info_label = QtWidgets.QLabel('Waiting for data...')
        self.info_label.setFont(QtGui.QFont('Monospace', 10))
        self.info_label.setAlignment(QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft)
        self.info_label.setWordWrap(False)
        self.info_label.setTextFormat(QtCore.Qt.RichText)
        self.info_label.setStyleSheet(
            'background-color: #1e1e1e; color: #d4d4d4; padding: 10px; '
            'border-radius: 6px;')
        right_layout.addWidget(self.info_label)

        top_widget.setStretchFactor(0, 3)
        top_widget.setStretchFactor(1, 2)

        # ── Bottom: two stacked time-series plots ────────────
        bottom_widget = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        vsplit.addWidget(bottom_widget)

        self.az_plot = pg.PlotWidget(title='Azimuth vs time (raw vs body-frame)')
        self.az_plot.showGrid(x=True, y=True, alpha=GRID_ALPHA / 255.0)
        self.az_plot.setLabel('bottom', 't', units='s')
        self.az_plot.setLabel('left', 'angle', units='deg')
        self.az_plot.addLegend(offset=(10, 10))
        self.az_raw_curve  = self.az_plot.plot(
            [], [], pen=pg.mkPen('#FFC107', width=1.2), name='raw (wire)')
        self.az_body_curve = self.az_plot.plot(
            [], [], pen=pg.mkPen('#E91E63', width=1.8), name='body (after sign/offset)')
        bottom_widget.addWidget(self.az_plot)

        self.rng_plot = pg.PlotWidget(title='Range vs time (raw vs calibrated)')
        self.rng_plot.showGrid(x=True, y=True, alpha=GRID_ALPHA / 255.0)
        self.rng_plot.setLabel('bottom', 't', units='s')
        self.rng_plot.setLabel('left', 'range', units='m')
        self.rng_plot.addLegend(offset=(10, 10))
        self.r_raw_curve = self.rng_plot.plot(
            [], [], pen=pg.mkPen('#FFC107', width=1.2), name='r_raw')
        self.r_cal_curve = self.rng_plot.plot(
            [], [], pen=pg.mkPen('#00BCD4', width=1.8), name='r_cal')
        bottom_widget.addWidget(self.rng_plot)

        # Splitter proportions: map+info : timeseries ≈ 5 : 3
        vsplit.setStretchFactor(0, 5)
        vsplit.setStretchFactor(1, 3)

        # ── Refresh timer ────────────────────────────────────
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._refresh)
        self.timer.start(UI_REFRESH_MS)

    # ── utility ─────────────────────────────────────────────
    @staticmethod
    def _trim_window(arr: np.ndarray, t_now: float, window: float) -> np.ndarray:
        """Return rows whose t >= t_now - window."""
        if arr is None or len(arr) == 0:
            return arr
        mask = arr[:, 0] >= (t_now - window)
        return arr[mask]

    # ── refresh tick ────────────────────────────────────────
    def _refresh(self):
        s = self.collector.snapshot()
        t_now = s['t_now_rel']

        # ---- 2D Map ----
        if s['uav_trail'] is not None and len(s['uav_trail']) > 0:
            self.uav_trail_plot.setData(s['uav_trail'][:, 0], s['uav_trail'][:, 1])
        if s['gs_trail'] is not None and len(s['gs_trail']) > 0:
            self.gs_trail_plot.setData(s['gs_trail'][:, 0], s['gs_trail'][:, 1])

        if s['uav_local'] is not None:
            self.uav_dot.setData([s['uav_local'][0]], [s['uav_local'][1]])
        if s['gs_local'] is not None:
            self.gs_dot.setData([s['gs_local'][0]], [s['gs_local'][1]])

        if s['uav_local'] is not None and s['gs_local'] is not None:
            self.link_line.setData(
                [s['uav_local'][0], s['gs_local'][0]],
                [s['uav_local'][1], s['gs_local'][1]])

        # ---- Time-series ----
        az_raw  = self._trim_window(s['ts_az_raw'],  t_now, TIMESERIES_SEC)
        az_body = self._trim_window(s['ts_az_body'], t_now, TIMESERIES_SEC)
        r_raw   = self._trim_window(s['ts_r_raw'],   t_now, TIMESERIES_SEC)
        r_cal   = self._trim_window(s['ts_r_cal'],   t_now, TIMESERIES_SEC)

        if az_raw is not None and len(az_raw) > 0:
            self.az_raw_curve.setData(az_raw[:, 0], az_raw[:, 1])
        if az_body is not None and len(az_body) > 0:
            self.az_body_curve.setData(az_body[:, 0], az_body[:, 1])
        if r_raw is not None and len(r_raw) > 0:
            self.r_raw_curve.setData(r_raw[:, 0], r_raw[:, 1])
        if r_cal is not None and len(r_cal) > 0:
            self.r_cal_curve.setData(r_cal[:, 0], r_cal[:, 1])

        # Slide x-range on bottom plots (so they scroll)
        if t_now > TIMESERIES_SEC:
            self.az_plot.setXRange(t_now - TIMESERIES_SEC, t_now, padding=0)
            self.rng_plot.setXRange(t_now - TIMESERIES_SEC, t_now, padding=0)

        # ---- Info panel (HTML for colour) ----
        self.info_label.setText(self._build_info_html(s))

    # ── info-panel composer ─────────────────────────────────
    @staticmethod
    def _fmt_vec3(v, fmt='{:+8.2f}'):
        return '(' + ', '.join(fmt.format(x) for x in v) + ')'

    def _build_info_html(self, s: dict) -> str:
        def hdr(text: str) -> str:
            return f'<div style="color:#888;">── {text} ──</div>'

        lines = []

        # UAV
        lines.append(hdr('UAV'))
        if s['uav_local'] is not None:
            u = s['uav_local']
            lines.append(f'&nbsp;&nbsp;Local:&nbsp;&nbsp;{self._fmt_vec3(u)} m')
        else:
            lines.append('&nbsp;&nbsp;Local:&nbsp;&nbsp;waiting for /uav/local_pose ...')
        if s['uav_abs'] is not None:
            a = s['uav_abs']
            lines.append(f'&nbsp;&nbsp;UTM:&nbsp;&nbsp;&nbsp;&nbsp;({a[0]:.2f}, {a[1]:.2f}, {a[2]:.2f})')

        # Base
        lines.append('')
        lines.append(hdr('Base Station (solved)'))
        if s['gs_local'] is not None:
            g = s['gs_local']
            lines.append(f'&nbsp;&nbsp;Local:&nbsp;&nbsp;{self._fmt_vec3(g)} m')
        else:
            lines.append('&nbsp;&nbsp;Local:&nbsp;&nbsp;waiting...')
        if s['gs_abs'] is not None:
            ga = s['gs_abs']
            lines.append(f'&nbsp;&nbsp;UTM:&nbsp;&nbsp;&nbsp;&nbsp;({ga[0]:.2f}, {ga[1]:.2f}, {ga[2]:.2f})')

        # Relative (UAV relative to base)
        lines.append('')
        lines.append(hdr('Relative (UAV − Base)'))
        if s['gs_relative'] is not None:
            r = s['gs_relative']
            lines.append(
                f'&nbsp;&nbsp;frame={s["gs_relative_frame"]}: '
                f'{self._fmt_vec3(r)} m')
        else:
            lines.append('&nbsp;&nbsp;waiting for /ground_station/relative_pose ...')

        if s['debug'] is not None:
            d = s['debug']
            dist    = d[3]
            bearing = d[4]
            lines.append(f'&nbsp;&nbsp;Horiz Dist (projected):&nbsp;&nbsp;{dist:.2f} m')
            lines.append(
                f'&nbsp;&nbsp;ENU azimuth of UAV: {bearing:+.1f}° '
                f'<span style="color:#888;">(0=East, +90=North)</span>')
            if len(d) >= 9:
                yaw = np.degrees(d[8])
                lines.append(
                    f'&nbsp;&nbsp;<span style="color:#888;">UAV yaw (diag only):'
                    f' {yaw:+.1f}°</span>')
            if len(d) >= 12:
                base_yaw = np.degrees(d[11])
                lines.append(
                    f'&nbsp;&nbsp;Ground base yaw (ENU): {base_yaw:+.1f}°')

        # AOA raw observation
        lines.append('')
        lines.append(hdr('AOA Observation (raw)'))
        if s['r_raw'] is not None:
            lines.append(
                f'&nbsp;&nbsp;slant_raw: {s["r_raw"]:6.3f} m&nbsp;&nbsp;&nbsp;'
                f'slant_cal: {s["r_cal"]:6.3f} m')
        else:
            lines.append('&nbsp;&nbsp;waiting for /aoa/measurement ...')
        if s['raw_az'] is not None:
            lines.append(
                f'&nbsp;&nbsp;az_raw: {s["raw_az"]:+7.2f}°&nbsp;&nbsp;'
                f'az_body: {s["body_az"]:+7.2f}°')
            if (s['base_yaw_deg'] is not None
                    and np.isfinite(s['base_yaw_deg'])):
                lines.append(
                    f'&nbsp;&nbsp;base_yaw_ENU: {s["base_yaw_deg"]:+7.2f}°'
                    f'&nbsp;&nbsp;az_ENU: {s["enu_az_deg"]:+7.2f}°')

        # Link status
        link = s['link_status']
        if link >= 1:
            link_html = '<span style="color:#4CAF50;font-weight:bold;">OK</span>'
        else:
            link_html = '<span style="color:#FFC107;font-weight:bold;">STALE</span>'
        lines.append(f'&nbsp;&nbsp;Link: {link_html} (code={link})')

        # Gate counters
        lines.append('')
        lines.append(hdr('Gate Counters (since viewer start)'))
        gc = s['gate_counts']
        total = sum(gc.values()) + s['gate_unknown']
        ok    = gc.get(0, 0)
        ratio = (ok / total * 100.0) if total > 0 else 0.0
        lines.append(
            f'&nbsp;&nbsp;Total frames:&nbsp;{total}&nbsp;&nbsp;'
            f'OK ratio:&nbsp;{ratio:5.1f}%')
        for code, name in GATE_NAMES.items():
            n = gc.get(code, 0)
            if code == 0:
                colour = '#4CAF50'
            elif n > 0:
                colour = '#F44336'
            else:
                colour = '#666'
            lines.append(
                f'&nbsp;&nbsp;<span style="color:{colour};">{name:<15}</span>'
                f' {n}')
        if s['gate_unknown'] > 0:
            lines.append(
                f'&nbsp;&nbsp;<span style="color:#F44336;">UNKNOWN&nbsp;&nbsp;'
                f'&nbsp;&nbsp;&nbsp;&nbsp;</span> {s["gate_unknown"]}')

        # Wrap in monospace div so &nbsp; align visually
        return '<div style="font-family:monospace;">' + '<br/>'.join(lines) + '</div>'


# ==============================================================
#  Entry point
# ==============================================================
def main():
    rclpy.init()
    collector = AoaTopicCollector()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(collector,), daemon=True)
    spin_thread.start()

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')

    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.Window, QtGui.QColor('#2b2b2b'))
    palette.setColor(QtGui.QPalette.WindowText, QtGui.QColor('#d4d4d4'))
    app.setPalette(palette)

    win = AoaDebugViewerWindow(collector)
    win.show()

    exit_code = app.exec_()

    try:
        collector.destroy_node()
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
