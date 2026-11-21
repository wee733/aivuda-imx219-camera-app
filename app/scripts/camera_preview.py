#!/usr/bin/env python3
"""On-demand ROS /camera/image_raw/compressed publisher and low-latency Web preview server.

Zero-overhead design: subscribes to the CUDA-backed /camera/image_raw stream ONLY
while at least one ROS subscriber (/camera/image_raw/compressed) or Web UI viewer
is connected, and destroys the subscription after a short idle grace period so
autonomous perception (AprilTag / QRLand) pays zero GPU->CPU copy overhead.
"""

from collections import deque
import ctypes
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class _TurboJpegEncoder:
    """Optional ctypes binding to libturbojpeg for direct RGB8 -> JPEG encoding (no RGB->BGR copy)."""

    _TJPF_RGB = 0
    _TJSAMP_420 = 2
    _TJFLAG_FASTDCT = 2048

    def __init__(self):
        self._lib = None
        self._handle = None
        for name in ('libturbojpeg.so.0', 'libturbojpeg.so'):
            try:
                lib = ctypes.CDLL(name)
                lib.tjInitCompress.restype = ctypes.c_void_p
                lib.tjDestroy.argtypes = [ctypes.c_void_p]
                lib.tjFree.argtypes = [ctypes.c_void_p]
                lib.tjCompress2.argtypes = [
                    ctypes.c_void_p, ctypes.c_void_p,
                    ctypes.c_int, ctypes.c_int, ctypes.c_int,
                    ctypes.c_int,
                    ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_ulong),
                    ctypes.c_int, ctypes.c_int, ctypes.c_int,
                ]
                handle = lib.tjInitCompress()
                if handle:
                    self._lib = lib
                    self._handle = handle
                    break
            except OSError:
                continue

    @property
    def available(self):
        return self._handle is not None

    def encode_rgb(self, rgb, quality):
        if not rgb.flags['C_CONTIGUOUS']:
            rgb = np.ascontiguousarray(rgb)
        h, w = rgb.shape[:2]
        out_buf = ctypes.c_void_p()
        out_size = ctypes.c_ulong(0)
        rc = self._lib.tjCompress2(
            self._handle,
            rgb.ctypes.data,
            w, rgb.strides[0], h,
            self._TJPF_RGB,
            ctypes.byref(out_buf), ctypes.byref(out_size),
            self._TJSAMP_420, int(quality), self._TJFLAG_FASTDCT,
        )
        if rc != 0 or not out_buf.value:
            raise RuntimeError('tjCompress2 failed')
        try:
            return ctypes.string_at(out_buf.value, out_size.value)
        finally:
            self._lib.tjFree(out_buf)

    def close(self):
        if self._lib is not None and self._handle is not None:
            try:
                self._lib.tjDestroy(self._handle)
            except Exception:
                pass
            self._handle = None


def probe_video_capabilities(turbo_available):
    """Detect available hardware and software video encoders inside the container."""
    caps = {
        'turbojpeg': bool(turbo_available),
        'opencv_jpeg': True,
        'isaac_ros_h264': Path('/opt/ros/lyrical/lib/libh264_encoder_node.so').is_file(),
        'nvenc_gst': False,
        'nvenc_ffmpeg': False,
        'v4l2_h264_ffmpeg': False,
        'libx264_ffmpeg': False,
        'h264_stream_backend': None,
    }
    if shutil.which('gst-inspect-1.0'):
        res = subprocess.run(['gst-inspect-1.0', 'nvv4l2h264enc'],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
        caps['nvenc_gst'] = (res.returncode == 0)
    if shutil.which('ffmpeg'):
        res = subprocess.run(['ffmpeg', '-hide_banner', '-encoders'],
                             capture_output=True, text=True, check=False)
        out = res.stdout if res.returncode == 0 else ''
        caps['nvenc_ffmpeg'] = ('h264_nvenc' in out)
        caps['v4l2_h264_ffmpeg'] = ('h264_v4l2m2m' in out)
        caps['libx264_ffmpeg'] = ('libx264' in out)
    if caps['nvenc_gst'] and shutil.which('gst-launch-1.0'):
        caps['h264_stream_backend'] = 'gst_nvenc'
    elif caps['nvenc_ffmpeg']:
        caps['h264_stream_backend'] = 'ffmpeg_nvenc'
    elif caps['v4l2_h264_ffmpeg']:
        caps['h264_stream_backend'] = 'ffmpeg_v4l2'
    elif caps['libx264_ffmpeg']:
        caps['h264_stream_backend'] = 'ffmpeg_libx264'
    return caps


class _PreviewHttpServer:
    """Localhost HTTP server for MJPEG stream, H.264 fMP4 stream, snapshot and live settings."""

    def __init__(self, node, port):
        self.node = node

        class Handler(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'

            def log_message(self, fmt, *args):
                pass

            def _send_json(self, status, payload):
                raw = json.dumps(payload, ensure_ascii=False).encode('utf-8')
                self.send_response(status)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)

            def do_GET(self):
                path = self.path.split('?', 1)[0].rstrip('/') or '/'
                if path in ('/', '/status', '/health'):
                    self._send_json(200, node.status_payload())
                    return
                if path == '/snapshot.jpg':
                    jpg = node.capture_snapshot(timeout=3.0)
                    if jpg is None:
                        self._send_json(503, {'error': 'Timed out waiting for a camera frame'})
                        return
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('Content-Disposition', 'inline; filename="imx219_snapshot.jpg"')
                    self.send_header('Content-Length', str(len(jpg)))
                    self.end_headers()
                    self.wfile.write(jpg)
                    return
                if path == '/stream.mjpg':
                    self._serve_mjpeg()
                    return
                if path == '/stream.mp4':
                    self._serve_h264_fmp4()
                    return
                self._send_json(404, {'error': f'not found: {path}'})

            def do_POST(self):
                path = self.path.split('?', 1)[0].rstrip('/')
                length = int(self.headers.get('Content-Length', '0') or '0')
                body = self.rfile.read(min(length, 65536)) if length > 0 else b'{}'
                try:
                    data = json.loads(body.decode('utf-8') or '{}')
                except ValueError as exc:
                    self._send_json(400, {'error': f'invalid JSON: {exc}'})
                    return
                if path == '/settings':
                    try:
                        updated = node.update_settings(data)
                    except (TypeError, ValueError) as exc:
                        self._send_json(400, {'error': str(exc)})
                        return
                    self._send_json(200, {'ok': True, 'settings': updated})
                    return
                self._send_json(404, {'error': f'not found: {path}'})

            def _serve_mjpeg(self):
                boundary = b'frame'
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
                self.send_header('Pragma', 'no-cache')
                self.send_header('Connection', 'close')
                self.end_headers()
                node.add_web_client(1)
                last_seq = 0
                try:
                    while not node.is_shutting_down():
                        seq, jpg = node.wait_for_jpeg(last_seq, timeout=1.5)
                        if jpg is None:
                            continue
                        last_seq = seq
                        header = (
                            b'--' + boundary + b'\r\n'
                            b'Content-Type: image/jpeg\r\n'
                            b'Content-Length: ' + str(len(jpg)).encode('ascii') + b'\r\n\r\n'
                        )
                        self.wfile.write(header)
                        self.wfile.write(jpg)
                        self.wfile.write(b'\r\n')
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                finally:
                    node.add_web_client(-1)

            def _serve_h264_fmp4(self):
                backend = node.capabilities.get('h264_stream_backend')
                if not backend:
                    self._send_json(501, {'error': 'No H.264 encoder available in this container'})
                    return
                node.add_web_client(1)
                proc = None
                feeder = None
                stop_ev = threading.Event()
                try:
                    w, h, fps, bitrate = node.h264_params()
                    if backend == 'gst_nvenc':
                        cmd = [
                            'gst-launch-1.0', '-q',
                            'fdsrc', '!',
                            f'rawvideoparse', f'width={w}', f'height={h}', 'format=rgb',
                            f'framerate={int(round(fps))}/1', '!',
                            'videoconvert', '!', 'video/x-raw,format=NV12', '!',
                            'nvvidconv', '!', 'video/x-raw(memory:NVMM),format=NV12', '!',
                            'nvv4l2h264enc', f'bitrate={bitrate * 1000}',
                            'preset-level=1', 'insert-sps-pps=true', 'iframeinterval=30', '!',
                            'h264parse', '!',
                            'mp4mux', 'fragment-duration=100', 'streamable=true', '!',
                            'fdsink', 'fd=1',
                        ]
                    else:
                        vcodec = {'ffmpeg_nvenc': 'h264_nvenc',
                                  'ffmpeg_v4l2': 'h264_v4l2m2m',
                                  'ffmpeg_libx264': 'libx264'}[backend]
                        extra = (['-preset', 'ultrafast', '-tune', 'zerolatency']
                                 if vcodec == 'libx264' else ['-preset', 'p1', '-tune', 'll'])
                        cmd = [
                            'ffmpeg', '-hide_banner', '-loglevel', 'error',
                            '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{w}x{h}',
                            '-r', f'{fps:.2f}', '-i', 'pipe:0',
                            '-c:v', vcodec, *extra, '-b:v', f'{bitrate}k',
                            '-g', str(max(10, int(round(fps)))),
                            '-pix_fmt', 'yuv420p',
                            '-f', 'mp4', '-movflags', 'frag_keyframe+empty_moov+default_base_moof',
                            'pipe:1',
                        ]
                    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=subprocess.DEVNULL)

                    def feed():
                        last_seq = 0
                        try:
                            while not stop_ev.is_set() and not node.is_shutting_down():
                                seq, rgb = node.wait_for_rgb(last_seq, w, h, timeout=1.0)
                                if rgb is None:
                                    continue
                                last_seq = seq
                                proc.stdin.write(rgb.tobytes())
                                proc.stdin.flush()
                        except (BrokenPipeError, OSError, ValueError):
                            pass
                        finally:
                            try:
                                proc.stdin.close()
                            except OSError:
                                pass

                    feeder = threading.Thread(target=feed, daemon=True)
                    feeder.start()
                    self.send_response(200)
                    self.send_header('Content-Type', 'video/mp4')
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('Connection', 'close')
                    self.end_headers()
                    while not node.is_shutting_down():
                        chunk = proc.stdout.read(16384)
                        if not chunk:
                            break
                        self.wfile.write(chunk)
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                finally:
                    stop_ev.set()
                    if proc is not None:
                        try:
                            proc.terminate()
                            proc.wait(timeout=2)
                        except Exception:
                            try:
                                proc.kill()
                            except Exception:
                                pass
                    if feeder is not None:
                        feeder.join(timeout=1.0)
                    node.add_web_client(-1)

        self._server = ThreadingHTTPServer(('127.0.0.1', int(port)), Handler)
        self._server.daemon_threads = True
        self._thread = threading.Thread(target=self._server.serve_forever,
                                        name='camera-preview-http', daemon=True)
        self._thread.start()

    def close(self):
        try:
            self._server.shutdown()
            self._server.server_close()
        except OSError:
            pass


class CameraPreviewNode(Node):
    """On-demand compressed ROS publisher + Web UI MJPEG/H.264 streaming sidecar."""

    IDLE_UNSUBSCRIBE_S = 2.0

    def __init__(self, config_path):
        super().__init__('imx219_preview')
        cfg = self._load_config(config_path)
        prev = cfg.get('preview') or {}
        self._exposure_us = int(cfg.get('exposure_us', 5000))
        self._ros_domain_id = int(cfg.get('ros_domain_id', int(os.environ.get('ROS_DOMAIN_ID', '0'))))

        self._lock = threading.Lock()
        self._settings = {
            'fps': float(prev.get('fps', 15.0)),
            'width': int(prev.get('width', 960)),
            'jpeg_quality': int(prev.get('jpeg_quality', 85)),
            'codec': str(prev.get('codec', 'auto')),
            'h264_bitrate_kbps': int(prev.get('h264_bitrate_kbps', 3000)),
        }
        self._turbo = _TurboJpegEncoder()
        self.capabilities = probe_video_capabilities(self._turbo.available)

        self._jpeg_cond = threading.Condition()
        self._jpeg_seq = 0
        self._latest_jpeg = None
        self._latest_rgb = None
        self._rgb_seq = 0
        self._web_clients = 0
        self._snapshot_waiters = 0
        self._last_needed_mono = 0.0
        self._last_encode_mono = 0.0
        self._shutting_down = False

        self._cam_info = {'width': 1280, 'height': 720, 'frame_id': 'imx219_optical_frame'}
        self._cam_times = deque(maxlen=60)
        self._enc_times = deque(maxlen=60)
        self._enc_bytes = deque(maxlen=60)
        self._enc_ms = deque(maxlen=30)
        self._image_sub = None

        self.compressed_pub = self.create_publisher(
            CompressedImage, '/camera/image_raw/compressed', qos_profile_sensor_data)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.on_camera_info, qos_profile_sensor_data)
        self._demand_timer = self.create_timer(0.25, self._sync_subscription)

        ui_port = int(prev.get('ui_port', 19311))
        self._http = None
        if ui_port > 0:
            try:
                self._http = _PreviewHttpServer(self, ui_port)
                self.get_logger().info(f'Camera preview UI API listening on http://127.0.0.1:{ui_port}')
            except OSError as exc:
                self.get_logger().warning(f'Camera preview HTTP API disabled (port {ui_port}): {exc}')

        self.get_logger().info(
            f'Camera preview ready (on-demand /camera/image_raw/compressed, '
            f'jpeg={"turbojpeg" if self._turbo.available else "opencv"}, '
            f'h264={self.capabilities["h264_stream_backend"] or "unavailable"}, '
            f'isaac_h264={self.capabilities["isaac_ros_h264"]})'
        )

    @staticmethod
    def _load_config(config_path):
        path = Path(config_path)
        if path.is_file():
            return json.loads(path.read_text(encoding='utf-8'))
        return {'ros_domain_id': 0, 'exposure_us': 5000, 'preview': {}}

    def is_shutting_down(self):
        return self._shutting_down or not rclpy.ok()

    def add_web_client(self, delta):
        with self._lock:
            self._web_clients = max(0, self._web_clients + delta)
            if self._web_clients > 0:
                self._last_needed_mono = time.monotonic()

    def _has_demand(self):
        ros_subs = self.compressed_pub.get_subscription_count()
        with self._lock:
            needed = ros_subs > 0 or self._web_clients > 0 or self._snapshot_waiters > 0
            if needed:
                self._last_needed_mono = time.monotonic()
            return needed, ros_subs, (time.monotonic() - self._last_needed_mono)

    def _sync_subscription(self):
        needed, _, idle_s = self._has_demand()
        if needed and self._image_sub is None:
            self._image_sub = self.create_subscription(
                Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
            self.get_logger().info('Preview demand active: subscribed to /camera/image_raw')
        elif not needed and self._image_sub is not None and idle_s >= self.IDLE_UNSUBSCRIBE_S:
            self.destroy_subscription(self._image_sub)
            self._image_sub = None
            self.get_logger().info('No preview consumers: unsubscribed from /camera/image_raw (0% overhead)')

    def on_camera_info(self, msg):
        now_m = time.monotonic()
        with self._lock:
            self._cam_times.append(now_m)
            self._cam_info = {
                'width': int(msg.width),
                'height': int(msg.height),
                'frame_id': msg.header.frame_id or 'imx219_optical_frame',
            }

    def on_image(self, msg):
        now_m = time.monotonic()
        with self._lock:
            target_fps = self._settings['fps']
            target_w = self._settings['width']
            quality = self._settings['jpeg_quality']
            snap_pending = self._snapshot_waiters > 0
        min_period = 1.0 / max(target_fps, 1.0)
        if not snap_pending and (now_m - self._last_encode_mono) < min_period * 0.92:
            return
        if msg.encoding != 'rgb8':
            return
        t0 = time.perf_counter()
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        rgb = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        if target_w and 0 < target_w < msg.width:
            target_h = max(2, int(round(msg.height * (target_w / msg.width))) & ~1)
            rgb_out = cv2.resize(rgb, (target_w, target_h), interpolation=cv2.INTER_AREA)
        else:
            rgb_out = rgb
        if self._turbo.available:
            jpg = self._turbo.encode_rgb(rgb_out, quality)
        else:
            ok, buf = cv2.imencode(
                '.jpg',
                cv2.cvtColor(rgb_out, cv2.COLOR_RGB2BGR),
                [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)],
            )
            if not ok:
                return
            jpg = buf.tobytes()
        enc_ms = (time.perf_counter() - t0) * 1e3
        self._last_encode_mono = now_m

        if self.compressed_pub.get_subscription_count() > 0:
            cmsg = CompressedImage()
            cmsg.header = msg.header
            cmsg.format = 'jpeg'
            cmsg.data = jpg
            self.compressed_pub.publish(cmsg)

        with self._lock:
            self._enc_times.append(now_m)
            self._enc_bytes.append(len(jpg))
            self._enc_ms.append(enc_ms)
        with self._jpeg_cond:
            self._jpeg_seq += 1
            self._latest_jpeg = jpg
            self._rgb_seq += 1
            self._latest_rgb = rgb_out
            self._jpeg_cond.notify_all()

    def wait_for_jpeg(self, last_seq, timeout=1.5):
        with self._jpeg_cond:
            self._jpeg_cond.wait_for(
                lambda: self._jpeg_seq > last_seq or self.is_shutting_down(), timeout=timeout)
            if self._jpeg_seq > last_seq:
                return self._jpeg_seq, self._latest_jpeg
            return last_seq, None

    def wait_for_rgb(self, last_seq, width, height, timeout=1.0):
        with self._jpeg_cond:
            self._jpeg_cond.wait_for(
                lambda: self._rgb_seq > last_seq or self.is_shutting_down(), timeout=timeout)
            if self._rgb_seq > last_seq and self._latest_rgb is not None:
                rgb = self._latest_rgb
                seq = self._rgb_seq
            else:
                return last_seq, None
        if rgb.shape[1] != width or rgb.shape[0] != height:
            rgb = cv2.resize(rgb, (width, height), interpolation=cv2.INTER_AREA)
        return seq, rgb

    def capture_snapshot(self, timeout=3.0):
        with self._lock:
            self._snapshot_waiters += 1
            self._last_needed_mono = time.monotonic()
            start_seq = self._jpeg_seq
        try:
            _, jpg = self.wait_for_jpeg(start_seq, timeout=timeout)
            return jpg
        finally:
            with self._lock:
                self._snapshot_waiters = max(0, self._snapshot_waiters - 1)

    def h264_params(self):
        with self._lock:
            src_w = self._cam_info['width'] or 1280
            src_h = self._cam_info['height'] or 720
            w = self._settings['width'] or src_w
            w = min(w, src_w) & ~1
            h = max(2, int(round(src_h * (w / src_w))) & ~1)
            return w, h, self._settings['fps'], self._settings['h264_bitrate_kbps']

    def update_settings(self, data):
        with self._lock:
            next_s = dict(self._settings)
            if 'fps' in data:
                fps = float(data['fps'])
                if not math.isfinite(fps) or not 1.0 <= fps <= 30.0:
                    raise ValueError('fps must be in [1, 30]')
                next_s['fps'] = fps
            if 'width' in data:
                w = int(data['width'])
                if w != 0 and not 320 <= w <= 1280:
                    raise ValueError('width must be 0 (native) or in [320, 1280]')
                next_s['width'] = w
            if 'jpeg_quality' in data:
                q = int(data['jpeg_quality'])
                if not 40 <= q <= 95:
                    raise ValueError('jpeg_quality must be in [40, 95]')
                next_s['jpeg_quality'] = q
            if 'h264_bitrate_kbps' in data:
                br = int(data['h264_bitrate_kbps'])
                if not 500 <= br <= 12000:
                    raise ValueError('h264_bitrate_kbps must be in [500, 12000]')
                next_s['h264_bitrate_kbps'] = br
            self._settings = next_s
            return dict(next_s)

    def status_payload(self):
        now_m = time.monotonic()
        ros_subs = self.compressed_pub.get_subscription_count()
        with self._lock:
            cam_t = [t for t in self._cam_times if now_m - t <= 2.0]
            cam_fps = round((len(cam_t) - 1) / (cam_t[-1] - cam_t[0]), 1) if len(cam_t) >= 2 and cam_t[-1] > cam_t[0] else 0.0
            enc_pairs = [(t, b) for t, b in zip(self._enc_times, self._enc_bytes) if now_m - t <= 2.0]
            if len(enc_pairs) >= 2 and enc_pairs[-1][0] > enc_pairs[0][0]:
                dt = enc_pairs[-1][0] - enc_pairs[0][0]
                prev_fps = round((len(enc_pairs) - 1) / dt, 1)
                bitrate_kbps = round(sum(b for _, b in enc_pairs[1:]) * 8.0 / dt / 1000.0, 1)
            else:
                prev_fps = 0.0
                bitrate_kbps = 0.0
            frame_kb = round(enc_pairs[-1][1] / 1024.0, 1) if enc_pairs else 0.0
            enc_ms = round(sum(self._enc_ms) / len(self._enc_ms), 2) if self._enc_ms else 0.0
            return {
                'running': True,
                'subscribed': self._image_sub is not None,
                'ros_compressed_subscribers': ros_subs,
                'web_clients': self._web_clients,
                'camera_fps': cam_fps,
                'preview_fps': prev_fps,
                'bitrate_kbps': bitrate_kbps,
                'frame_kb': frame_kb,
                'encode_ms': enc_ms,
                'exposure_us': self._exposure_us,
                'ros_domain_id': self._ros_domain_id,
                'discovery_range': os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE', ''),
                'camera': dict(self._cam_info),
                'settings': dict(self._settings),
                'capabilities': dict(self.capabilities),
                'topics': {
                    'raw_cuda': '/camera/image_raw',
                    'compressed': '/camera/image_raw/compressed',
                    'camera_info': '/camera/camera_info',
                },
            }

    def destroy_node(self):
        self._shutting_down = True
        with self._jpeg_cond:
            self._jpeg_cond.notify_all()
        if self._http is not None:
            self._http.close()
        self._turbo.close()
        return super().destroy_node()


def main():
    config_path = sys.argv[1] if len(sys.argv) > 1 else '/data/app_config.json'
    rclpy.init(args=[])
    node = None
    try:
        node = CameraPreviewNode(config_path)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
