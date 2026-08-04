"""ROS 2 node that bridges an NTRIP correction stream to a UM982."""

import base64
import json
import math
import socket
import threading
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import serial
from rtk_interfaces.msg import RtkFix
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String, UInt8, UInt64

from .nmea import GgaFix, parse_gga
from .rtcm3 import Rtcm3Inspector


def rtcm_stream_stalled(
        last_receive_s: Optional[float],
        now_s: float,
        timeout_s: float) -> bool:
    """Return true when a connected stream has exceeded its data timeout."""
    return (
        last_receive_s is not None
        and now_s - last_receive_s >= timeout_s
    )


class Um982NtripNode(Node):
    """Read GGA, publish fixes, and feed NTRIP RTCM data to a UM982."""

    def __init__(self) -> None:
        super().__init__('um982_ntrip')

        self.declare_parameter(
            'serial_port',
            '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('configure_gga', True)
        self.declare_parameter('gga_command', 'GPGGA 1')
        self.declare_parameter('receiver_mode_command', 'MODE ROVER SURVEY')
        self.declare_parameter(
            'receiver_pvt_command', 'CONFIG PVTALG MULTI')
        self.declare_parameter('receiver_rtk_timeout_sec', 60)
        self.declare_parameter('query_receiver_info', True)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('ntrip_host', '')
        self.declare_parameter('ntrip_port', 8002)
        self.declare_parameter('ntrip_mountpoint', '')
        self.declare_parameter('ntrip_username', '')
        self.declare_parameter('ntrip_password', '')
        self.declare_parameter('send_gga_interval_sec', 1.0)
        self.declare_parameter('reconnect_delay_sec', 5.0)
        self.declare_parameter('rtcm_stall_timeout_sec', 10.0)
        self.declare_parameter('rtcm_status_interval_sec', 5.0)
        self.declare_parameter('service_mask', 15)

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.configure_gga = bool(self.get_parameter('configure_gga').value)
        self.gga_command = str(self.get_parameter('gga_command').value)
        self.receiver_mode_command = str(
            self.get_parameter('receiver_mode_command').value)
        self.receiver_pvt_command = str(
            self.get_parameter('receiver_pvt_command').value)
        self.receiver_rtk_timeout = int(
            self.get_parameter('receiver_rtk_timeout_sec').value)
        self.query_receiver_info = bool(
            self.get_parameter('query_receiver_info').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.ntrip_host = str(self.get_parameter('ntrip_host').value)
        self.ntrip_port = int(self.get_parameter('ntrip_port').value)
        self.ntrip_mountpoint = str(
            self.get_parameter('ntrip_mountpoint').value).lstrip('/')
        self.ntrip_username = str(
            self.get_parameter('ntrip_username').value)
        self.ntrip_password = str(
            self.get_parameter('ntrip_password').value)
        self.send_gga_interval = float(
            self.get_parameter('send_gga_interval_sec').value)
        self.reconnect_delay = float(
            self.get_parameter('reconnect_delay_sec').value)
        self.rtcm_stall_timeout = float(
            self.get_parameter('rtcm_stall_timeout_sec').value)
        self.rtcm_status_interval = float(
            self.get_parameter('rtcm_status_interval_sec').value)
        self.service_mask = int(self.get_parameter('service_mask').value)
        if self.rtcm_stall_timeout <= 0.0:
            raise ValueError('rtcm_stall_timeout_sec must be positive')
        if self.rtcm_status_interval <= 0.0:
            raise ValueError('rtcm_status_interval_sec must be positive')

        self.fix_pub = self.create_publisher(
            NavSatFix, 'fix', qos_profile_sensor_data)
        self.rtk_fix_pub = self.create_publisher(RtkFix, 'rtk/fix_info', 10)
        self.status_pub = self.create_publisher(String, 'rtk/status', 10)
        self.quality_pub = self.create_publisher(UInt8, 'rtk/fix_quality', 10)
        self.satellites_pub = self.create_publisher(
            UInt8, 'rtk/satellites', 10)
        self.hdop_pub = self.create_publisher(Float64, 'rtk/hdop', 10)
        self.height_msl_pub = self.create_publisher(
            Float64, 'rtk/height_msl', qos_profile_sensor_data)
        self.geoid_pub = self.create_publisher(
            Float64, 'rtk/geoid_separation', qos_profile_sensor_data)
        self.correction_age_pub = self.create_publisher(
            Float64, 'rtk/correction_age', 10)
        self.gga_pub = self.create_publisher(
            String, 'rtk/gga', qos_profile_sensor_data)
        self.rtcm_bytes_pub = self.create_publisher(
            UInt64, 'rtk/rtcm_bytes', 10)
        self.rtcm_status_pub = self.create_publisher(
            String, 'rtk/rtcm_status', 10)
        self.receiver_response_pub = self.create_publisher(
            String, 'rtk/receiver_response', 10)

        self._stop = threading.Event()
        self._serial: Optional[serial.Serial] = None
        self._ntrip_socket: Optional[socket.socket] = None
        self._serial_buffer = bytearray()
        self._latest_gga = ''
        self._latest_fix: Optional[GgaFix] = None
        self._last_status = ''
        self._rtcm_bytes = 0
        self._last_rtcm_monotonic: Optional[float] = None
        self._last_rtcm_status_monotonic = 0.0
        self._rtcm_inspector = Rtcm3Inspector()
        self._worker = threading.Thread(
            target=self._run, name='um982-io', daemon=True)
        self._worker.start()

    @property
    def _ntrip_configured(self) -> bool:
        return all((
            self.ntrip_host,
            self.ntrip_mountpoint,
            self.ntrip_username,
            self.ntrip_password,
        ))

    def _open_serial(self) -> None:
        self._serial = serial.Serial(
            self.serial_port,
            self.baudrate,
            timeout=0.05,
            write_timeout=1.0,
        )
        self._serial.reset_input_buffer()
        if self.receiver_mode_command:
            self._send_receiver_command(self.receiver_mode_command)
        if self.receiver_pvt_command:
            self._send_receiver_command(self.receiver_pvt_command)
        if self.receiver_rtk_timeout > 0:
            self._send_receiver_command(
                f'CONFIG RTK TIMEOUT {self.receiver_rtk_timeout}')
        if self.configure_gga and self.gga_command:
            self._send_receiver_command(self.gga_command)
        if self.query_receiver_info:
            self._send_receiver_command('MODE')
            self._send_receiver_command('VERSIONA')
        self.get_logger().info(
            f'UM982 serial opened: {self.serial_port} @ {self.baudrate}; '
            f'receiver mode command="{self.receiver_mode_command}"')

    def _send_receiver_command(self, command: str) -> None:
        if self._serial is None:
            return
        payload = command.rstrip('\r\n') + '\r\n'
        self._serial.write(payload.encode('ascii'))
        time.sleep(0.05)

    def _read_serial(self) -> None:
        if self._serial is None:
            return
        waiting = self._serial.in_waiting
        data = self._serial.read(waiting if waiting > 0 else 1)
        if not data:
            return
        self._serial_buffer.extend(data)
        while b'\n' in self._serial_buffer:
            line, _, remainder = self._serial_buffer.partition(b'\n')
            self._serial_buffer = bytearray(remainder)
            text = line.rstrip(b'\r').decode('ascii', errors='ignore')
            self._handle_serial_line(text)

        if len(self._serial_buffer) > 65536:
            self.get_logger().warning('Discarding oversized serial buffer')
            self._serial_buffer.clear()

    def _handle_serial_line(self, sentence: str) -> None:
        try:
            fix = parse_gga(sentence)
        except ValueError as error:
            self.get_logger().warning(f'Invalid GGA ignored: {error}')
            return
        if fix is None:
            receiver_prefixes = (
                '#MODE', '#VERSION', '$command', '$CONFIG')
            if sentence.startswith(receiver_prefixes):
                self.receiver_response_pub.publish(String(data=sentence))
                self.get_logger().info(f'UM982 response: {sentence}')
            return

        self._latest_gga = fix.sentence
        self._latest_fix = fix
        self._publish_fix(fix)

    def _publish_fix(self, fix: GgaFix) -> None:
        now = self.get_clock().now().to_msg()
        message = NavSatFix()
        message.header.stamp = now
        message.header.frame_id = self.frame_id
        message.latitude = fix.latitude
        message.longitude = fix.longitude
        message.altitude = fix.altitude_ellipsoid
        message.status.service = self.service_mask
        if fix.quality == 0 or not (
                math.isfinite(fix.latitude) and
                math.isfinite(fix.longitude)):
            message.status.status = NavSatStatus.STATUS_NO_FIX
        elif fix.quality in (2, 4, 5):
            message.status.status = NavSatStatus.STATUS_GBAS_FIX
        else:
            message.status.status = NavSatStatus.STATUS_FIX
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.fix_pub.publish(message)

        self.status_pub.publish(String(data=fix.status_name))
        self.quality_pub.publish(UInt8(data=max(0, min(fix.quality, 255))))
        self.satellites_pub.publish(
            UInt8(data=max(0, min(fix.satellites, 255))))
        self.hdop_pub.publish(Float64(data=fix.hdop))
        self.height_msl_pub.publish(Float64(data=fix.altitude_msl))
        self.geoid_pub.publish(Float64(data=fix.geoid_separation))
        self.correction_age_pub.publish(Float64(data=fix.correction_age))
        self.gga_pub.publish(String(data=fix.sentence))

        rtk_message = RtkFix()
        rtk_message.header.stamp = now
        rtk_message.header.frame_id = self.frame_id
        rtk_message.latitude = fix.latitude
        rtk_message.longitude = fix.longitude
        rtk_message.altitude_ellipsoid = fix.altitude_ellipsoid
        rtk_message.height_msl = fix.altitude_msl
        rtk_message.geoid_separation = fix.geoid_separation
        rtk_message.fix_quality = max(0, min(fix.quality, 255))
        rtk_message.status = fix.status_name
        rtk_message.position_valid = (
            fix.quality > 0 and
            math.isfinite(fix.latitude) and
            math.isfinite(fix.longitude)
        )
        rtk_message.rtk_fixed = fix.quality == RtkFix.QUALITY_RTK_FIXED
        rtk_message.satellites = max(0, min(fix.satellites, 255))
        rtk_message.hdop = fix.hdop
        rtk_message.correction_age = fix.correction_age
        rtk_message.rtcm_bytes = self._rtcm_bytes
        rtk_message.gga = fix.sentence
        self.rtk_fix_pub.publish(rtk_message)

        if fix.status_name != self._last_status:
            self.get_logger().info(
                f'GNSS status: {fix.status_name}, '
                f'satellites={fix.satellites}, '
                f'hdop={fix.hdop}')
            self._last_status = fix.status_name

    def _connect_ntrip(self) -> Tuple[socket.socket, bytes]:
        sock = socket.create_connection(
            (self.ntrip_host, self.ntrip_port), timeout=10.0)
        sock.settimeout(3.0)
        token = base64.b64encode(
            f'{self.ntrip_username}:{self.ntrip_password}'.encode('utf-8')
        ).decode('ascii')
        request = (
            f'GET /{self.ntrip_mountpoint} HTTP/1.0\r\n'
            'User-Agent: NTRIP um982_ntrip/0.1\r\n'
            'Accept: */*\r\n'
            f'Authorization: Basic {token}\r\n'
            '\r\n'
        )
        sock.sendall(request.encode('ascii'))
        initial_body = self._read_ntrip_header(sock)
        sock.sendall(self._latest_gga.rstrip('\r\n').encode('ascii') + b'\r\n')
        sock.settimeout(0.02)
        self.get_logger().info(
            f'NTRIP connected: {self.ntrip_host}:{self.ntrip_port}/'
            f'{self.ntrip_mountpoint}')
        return sock, initial_body

    @staticmethod
    def _read_ntrip_header(sock: socket.socket) -> bytes:
        response = bytearray()
        while len(response) < 16384:
            chunk = sock.recv(4096)
            if not chunk:
                raise ConnectionError('NTRIP caster closed during login')
            response.extend(chunk)

            first_line_end = response.find(b'\r\n')
            if first_line_end < 0:
                continue
            first_line = bytes(response[:first_line_end])
            if b'200 OK' not in first_line:
                readable = first_line.decode('ascii', errors='replace')
                raise ConnectionError(f'NTRIP login rejected: {readable}')

            header_end = response.find(b'\r\n\r\n')
            if header_end >= 0:
                return bytes(response[header_end + 4:])

            if first_line.startswith(b'ICY 200 OK'):
                remainder = bytes(response[first_line_end + 2:])
                rtcm_start = remainder.find(b'\xd3')
                if rtcm_start >= 0:
                    return remainder[rtcm_start:]
                # NTRIP v1 casters commonly return only the ICY status line.
                if not remainder:
                    return b''

        raise ConnectionError('NTRIP response header is too large')

    def _write_rtcm(self, data: bytes) -> None:
        if not data or self._serial is None:
            return
        frames = self._rtcm_inspector.feed(data)
        if not frames:
            self._publish_rtcm_status_if_due()
            return
        payload = b''.join(frames)
        self._serial.write(payload)
        self._rtcm_bytes += len(payload)
        self._last_rtcm_monotonic = time.monotonic()
        self.rtcm_bytes_pub.publish(UInt64(data=self._rtcm_bytes))
        self._publish_rtcm_status_if_due()

    def _publish_rtcm_status_if_due(self) -> None:
        now = time.monotonic()
        if now - self._last_rtcm_status_monotonic < self.rtcm_status_interval:
            return
        status = self._rtcm_inspector.status()
        status['validated_bytes'] = self._rtcm_bytes
        self.rtcm_status_pub.publish(String(data=json.dumps(status)))
        self._last_rtcm_status_monotonic = now

    def _close_ntrip(self) -> None:
        if self._ntrip_socket is not None:
            try:
                self._ntrip_socket.close()
            except OSError:
                pass
            self._ntrip_socket = None
        self._last_rtcm_monotonic = None

    def _run(self) -> None:
        try:
            self._open_serial()
        except (OSError, serial.SerialException) as error:
            self.get_logger().error(f'Cannot open UM982 serial port: {error}')
            return

        if not self._ntrip_configured:
            self.get_logger().warning(
                'NTRIP credentials are empty; publishing GNSS data without '
                'network corrections')

        next_connect_time = 0.0
        last_gga_send = 0.0
        while not self._stop.is_set() and rclpy.ok():
            try:
                self._read_serial()
            except (OSError, serial.SerialException) as error:
                self.get_logger().error(f'UM982 serial read failed: {error}')
                break

            now = time.monotonic()
            has_position = (
                self._latest_fix is not None and
                self._latest_fix.quality > 0 and
                math.isfinite(self._latest_fix.latitude) and
                math.isfinite(self._latest_fix.longitude)
            )
            if (self._ntrip_socket is None and self._ntrip_configured and
                    has_position and now >= next_connect_time):
                try:
                    self._ntrip_socket, initial = self._connect_ntrip()
                    # Start the stall watchdog even when the HTTP response did
                    # not contain an initial RTCM payload.
                    self._last_rtcm_monotonic = time.monotonic()
                    self._write_rtcm(initial)
                    last_gga_send = now
                except (OSError, ConnectionError) as error:
                    self.get_logger().error(
                        f'NTRIP connection failed: {error}')
                    self._close_ntrip()
                    next_connect_time = (
                        time.monotonic() + self.reconnect_delay)

            if self._ntrip_socket is None:
                continue

            try:
                if now - last_gga_send >= self.send_gga_interval:
                    payload = self._latest_gga.rstrip('\r\n').encode('ascii')
                    self._ntrip_socket.sendall(payload + b'\r\n')
                    last_gga_send = now
                try:
                    corrections = self._ntrip_socket.recv(4096)
                    if not corrections:
                        raise ConnectionError('NTRIP caster closed connection')
                    self._write_rtcm(corrections)
                except socket.timeout:
                    last_rtcm = self._last_rtcm_monotonic
                    if rtcm_stream_stalled(
                            last_rtcm,
                            time.monotonic(),
                            self.rtcm_stall_timeout):
                        raise ConnectionError(
                            'RTCM stream stalled for '
                            f'{self.rtcm_stall_timeout:.1f}s')
            except (OSError, ConnectionError, serial.SerialException) as error:
                if self._stop.is_set() or not rclpy.ok():
                    break
                self.get_logger().error(f'NTRIP stream failed: {error}')
                self._close_ntrip()
                next_connect_time = time.monotonic() + self.reconnect_delay

        self._close_ntrip()
        if self._serial is not None:
            try:
                self._serial.close()
            except OSError:
                pass
            self._serial = None

    def destroy_node(self) -> bool:
        """Stop I/O before letting rclpy destroy publishers and logging."""
        self._stop.set()
        self._close_ntrip()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Um982NtripNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
