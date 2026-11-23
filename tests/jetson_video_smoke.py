"""Verify H.264 access units and their exact acquisition timestamp against live CSI CameraInfo."""
import argparse
import os
import re
from pathlib import Path
import time

import rclpy
from foxglove_msgs.msg import CompressedVideo
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage


def stamp(value):
    return value.sec * 10 ** 9 + value.nanosec


def nal_types(data):
    return {unit[0] & 31 for unit in re.split(b'\x00\x00(?:\x00)?\x01', data) if unit}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    parser.add_argument('--compressed-image', action='store_true')
    parser.add_argument('--frames', type=int, default=36)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('imx219_h264_check')
    info_stamps = set()
    frames = []
    frame_type = CompressedImage if args.compressed_image else CompressedVideo
    topic = '/camera/image_compressed' if args.compressed_image else '/camera/video/h264'

    def info(msg):
        assert msg.width == 1280 and msg.height == 720 and msg.k[0] > 0
        assert msg.header.frame_id == 'imx219_optical_frame'
        info_stamps.add(stamp(msg.header.stamp))

    def frame(msg):
        header = msg.header if args.compressed_image else msg
        timestamp = msg.header.stamp if args.compressed_image else msg.timestamp
        assert header.frame_id == 'imx219_optical_frame' and msg.format.lower() == 'h264'
        # rclpy on Lyrical exposes rosidl::Buffer as a buffer proxy.
        data = bytes(msg.data)
        assert data.startswith((b'\x00\x00\x01', b'\x00\x00\x00\x01'))
        # A late subscriber starts decoding at the next random-access point.
        if frames or 5 in nal_types(data):
            frames.append((stamp(timestamp), data))

    node.create_subscription(CameraInfo, '/camera/camera_info', info, qos_profile_sensor_data)
    # Establish a baseline of source timestamps before requesting NVENC.
    deadline = time.monotonic() + 18
    while len(info_stamps) < 4 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert len(info_stamps) >= 4, 'CameraInfo stream unavailable'
    node.create_subscription(frame_type, topic, frame, qos_profile_sensor_data)
    deadline = time.monotonic() + 24
    while len(frames) < args.frames and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert len(frames) >= args.frames, 'Encoded stream timed out after %d frames' % len(frames)
    assert all(a[0] < b[0] for a, b in zip(frames, frames[1:])), 'Reordered video timestamps'
    # The last CameraInfo can arrive immediately after its video access unit.
    deadline = time.monotonic() + 1
    while frames[-1][0] not in info_stamps and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    missing = [stamp_ns for stamp_ns, _ in frames if stamp_ns not in info_stamps]
    assert not missing, 'Video timestamps missing from CameraInfo: %s' % missing
    idrs = 0
    for _, data in frames:
        nals = nal_types(data)
        if 5 in nals:
            assert {7, 8}.issubset(nals), 'Missing SPS/PPS in keyframe'
            idrs += 1
    assert idrs > 0, 'No IDR frame in the captured interval'
    args.output.write_bytes(b''.join(data for _, data in frames))
    rate = (len(frames) - 1) * 1e9 / (frames[-1][0] - frames[0][0])
    print('%d matched %s frames, %.2f fps, %d bytes, domain %s' %
          (len(frames), topic, rate, args.output.stat().st_size, os.getenv('ROS_DOMAIN_ID')))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
