import json
import os

from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def stop_launch(event, context):
    if context.is_shutdown:
        return []
    return [EmitEvent(event=Shutdown(reason='Camera component container exited'))]


def generate_launch_description():
    with open(os.environ['IMX219_SETTINGS'], encoding='utf-8') as source:
        cfg = json.load(source)
    stream = cfg['stream']
    intra = [{'use_intra_process_comms': True}]
    camera = ComposableNode(
        package='isaac_ros_imx219', plugin='imx219::Imx219Node',
        name='imx219', namespace='camera', extra_arguments=intra,
        parameters=['/app/config/driver.yaml', {
            'exposure_us': cfg['exposure_us'], 'stream.mode': stream['mode'],
            'stream.fps': stream['fps'],
            'stream.publish_compressed_image': stream['publish_compressed_image']}])
    encoder = ComposableNode(
        package='isaac_ros_h264_encoder', plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
        name='h264_encoder', namespace='camera', extra_arguments=intra,
        parameters=[{**{k: stream[k] for k in ('config', 'qp', 'iframe_interval')},
                     'input_width': 1280, 'input_height': 720, 'framerate': stream['fps'],
                     # Lyrical 5.0 takes a one-based Tegra preset and 0=Baseline, 1=Main.
                     'hw_preset_type': stream['hw_preset'] + 1,
                     'profile': {0: 1, 1: 0, 2: 2}[stream['profile']],
                     'entropy': 0 if stream['profile'] == 1 else 1,
                     'bitrate': stream['bitrate_kbps'] * 1000,
                     'idr_interval': stream['iframe_interval'], 'num_bframes': 0}],
        remappings=[('image_raw', 'internal/encode_input'), ('image_compressed', 'internal/encoded')])
    return LaunchDescription([ComposableNodeContainer(
        name='camera_container', namespace='', package='rclcpp_components',
        executable='component_container', composable_node_descriptions=[camera, encoder],
        arguments=['--ros-args', '--log-level', 'serialize_buffer_with_endpoint:=warn',
                   '--log-level', 'deserialize_buffer_with_endpoint:=warn'],
        output='screen', on_exit=stop_launch)])
