"""Run inside the Isaac ROS image with private networking/IPC; needs no CSI device.

Usage: python3 tests/jetson_preview_smoke.py app/scripts/camera_preview.py
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
from urllib.error import URLError
from urllib.request import Request, urlopen

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


BASE = 'http://127.0.0.1:19311'


def main():
    with tempfile.TemporaryDirectory() as tmp:
        cfg = Path(tmp) / 'preview.json'
        cfg.write_text(json.dumps({'ros_domain_id': int(os.environ['ROS_DOMAIN_ID']),
                                   'preview': {'fps': 12.5, 'width': 322, 'codec': 'jpeg'}}))
        proc = subprocess.Popen([sys.executable, sys.argv[1], str(cfg)])
        try:
            for _ in range(50):
                if proc.poll() is not None:
                    raise RuntimeError(f'preview exited with status {proc.returncode}')
                try:
                    with urlopen(BASE + '/status', timeout=1) as resp:
                        status = json.load(resp)
                    break
                except URLError:
                    time.sleep(.1)
            else:
                raise RuntimeError('preview did not become ready')
            assert status['settings']['fps'] == 12.5 and status['settings']['width'] == 322
            with urlopen(Request(BASE + '/settings', data=b'{"jpeg_quality": 87}',
                                 headers={'Content-Type': 'application/json'}), timeout=2) as resp:
                settings = json.load(resp)['settings']
            assert settings['fps'] == 12.5 and settings['jpeg_quality'] == 87
            rclpy.init()
            node = rclpy.create_node('imx219_smoke_source')
            try:
                pub = node.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
                # Leave one empty frame ahead of the valid ones: it must not kill the subscriber.
                img = Image(height=360, width=640, step=640 * 3, encoding='rgb8')
                pixels = np.zeros((360, 640, 3), np.uint8)
                pixels[:, :320, 0] = 255
                pixels[:, 320:, 1] = 255

                def publish():
                    for _ in range(60):
                        if pub.get_subscription_count():
                            if len(img.data) == 0:
                                pub.publish(img)
                                img.data = pixels.tobytes()
                            pub.publish(img)
                        if done.wait(.05):
                            return

                done = threading.Event()
                t = threading.Thread(target=publish)
                t.start()
                try:
                    with urlopen(BASE + '/snapshot.jpg', timeout=5) as resp:
                        actual = cv2.imdecode(np.frombuffer(resp.read(), np.uint8), cv2.IMREAD_COLOR)
                    assert actual.shape == (180, 322, 3)
                    assert actual[90, 50, 2] > 240 and actual[90, 270, 1] > 240
                    print(f"domain={status['ros_domain_id']} fps={settings['fps']} "
                          f"JPEG={actual.shape[1]}x{actual.shape[0]} color bars OK; "
                          f"H.264 backend={status['capabilities']['h264_stream_backend']}", flush=True)
                finally:
                    done.set()
                    t.join()
            finally:
                node.destroy_node()
                rclpy.try_shutdown()
        finally:
            if proc.poll() is None:
                proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()


if __name__ == '__main__':
    main()
