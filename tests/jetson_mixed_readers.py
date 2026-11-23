"""Check ground-station video after a temporary Lyrical reader leaves the Jetson."""
import argparse
import shlex
import subprocess
import time

import rclpy
from foxglove_msgs.msg import CompressedVideo
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ssh_host', help='Jetson SSH alias with key-based authentication')
    parser.add_argument('--prefix', default='/uav1', help='Ground-station robot namespace')
    parser.add_argument('--compressed-image', action='store_true')
    args = parser.parse_args()
    if args.compressed_image:
        msg_type, source = CompressedImage, '/camera/image_compressed'
    else:
        msg_type, source = CompressedVideo, '/camera/video/h264'
    local_topic = args.prefix.rstrip('/') + source
    rclpy.init()
    node = rclpy.create_node('mixed_video_reader_check')
    frames = []

    def receive(msg):
        assert msg.format.lower() == 'h264' and len(msg.data) > 0
        stamp = msg.header.stamp if args.compressed_image else msg.timestamp
        frames.append(stamp.sec * 10**9 + stamp.nanosec)

    def collect(after=0):
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            fresh = [stamp for stamp in frames if stamp > after]
            if len(fresh) >= 12:
                assert all(a < b for a, b in zip(fresh, fresh[1:]))
                return fresh[-1]
        raise RuntimeError(f'{local_topic} stalled with {len(frames)} received frames')

    try:
        node.create_subscription(msg_type, local_topic, receive, qos_profile_sensor_data)
        collect()
        type_name = f'{msg_type.__module__.split(".")[0]}/msg/{msg_type.__name__}'
        echo = shlex.join(['ros2', 'topic', 'echo', '--once', '--qos-reliability',
                           'best_effort', '--field', 'format', source, type_name])
        command = shlex.join(['docker', 'exec', 'aivuda_isaac_camera', 'bash', '-c',
                              'source /opt/ros/lyrical/setup.bash && ' + echo])
        print('Starting temporary Lyrical reader', flush=True)
        subprocess.run(['ssh', '-o', 'BatchMode=yes', '--', args.ssh_host, command],
                       check=True, stdout=subprocess.DEVNULL, timeout=20)
        print('Temporary Lyrical reader exited', flush=True)
        # Drain samples and allow DDS to unmatch the temporary reader.
        until = time.monotonic() + 2
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.1)
        last = frames[-1]
        frames.clear()
        collect(after=last)
        print(f'{local_topic} continued after the temporary camera reader exited')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
