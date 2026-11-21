"""Exercise the real preview encoder with a synthetic RGB8 ROS image, no CSI device."""
from collections import Counter
from pathlib import Path
import shutil
import subprocess
import threading
from types import SimpleNamespace

import pytest


@pytest.fixture
def preview(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / 'app/scripts'))
    for dep in ('cv2', 'numpy', 'rclpy', 'sensor_msgs.msg'):
        pytest.importorskip(dep)
    import camera_preview
    return camera_preview


@pytest.fixture
def encoder():
    published = []
    lock = threading.Lock()
    node = SimpleNamespace(
        _lock=lock, _settings={'width': 960, 'fps': 15, 'jpeg_quality': 85,
                                            'h264_bitrate_kbps': 3000},
        _cam_info={'width': 1280, 'height': 720}, _snapshot_waiters=0, _last_encode_mono=0,
        _turbo=SimpleNamespace(available=False), _enc_times=[], _enc_ms=[], _enc_bytes=[],
        _frame_cond=threading.Condition(lock), _frame_seq=0, _web_clients=0,
        _h264_sizes=Counter(), _rgb_frames={},
        compressed_pub=SimpleNamespace(get_subscription_count=lambda: 1, publish=published.append),
    )
    return node, published


@pytest.mark.parametrize(('width', 'jpeg_w', 'jpeg_h', 'h264_w', 'h264_h'), [
    (0, 1280, 720, 1280, 720), (1, 1, 2, 320, 180), (160, 160, 90, 320, 180),
    (319, 319, 178, 320, 180), (320, 320, 180, 320, 180), (961, 961, 540, 960, 540),
    (1280, 1280, 720, 1280, 720),
])
def test_preview_geometry_and_rgb_order(preview, encoder, width, jpeg_w, jpeg_h, h264_w, h264_h):
    import cv2
    import numpy as np
    node, published = encoder
    CameraPreviewNode = preview.CameraPreviewNode
    CameraPreviewNode.update_settings(node, {'width': width})
    assert CameraPreviewNode.h264_params(node)[:2] == (h264_w, h264_h)
    image = preview.Image(width=1280, height=720, step=1280 * 3, encoding='rgb8')
    image.header.frame_id = 'imx219_optical_frame'
    image.header.stamp.sec = 10
    rgb = bytes([255, 0, 0]) * (image.width * image.height)
    image.data = rgb
    CameraPreviewNode.add_web_client(node, 1, (h264_w, h264_h))
    CameraPreviewNode.on_image(node, image)
    assert len(published) == 1 and published[0].header == image.header
    decoded = cv2.imdecode(np.frombuffer(published[0].data, np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (jpeg_h, jpeg_w, 3)
    seq, frame = CameraPreviewNode.wait_for_rgb(node, 0, h264_w, h264_h)
    assert seq == 1 and frame.shape == (h264_h, h264_w, 3)
    b, g, r = decoded[0, 0]
    assert r > 250 and b < 3 and g < 3
    assert (image.width, image.height, bytes(image.data)) == (1280, 720, rgb)


@pytest.mark.parametrize('width', [1, 319, 322])
def test_h264_detail_independent_of_jpeg_width(preview, encoder, width):
    import cv2
    import numpy as np
    node, _ = encoder
    api = preview.CameraPreviewNode
    # An existing connection retains its 960px negotiation after a JPEG width change.
    old_size = api.h264_params(node)[:2]
    api.update_settings(node, {'width': width})
    pixels = np.random.default_rng(0).integers(256, size=(720, 1280, 3), dtype=np.uint8)
    msg = preview.Image(width=1280, height=720, step=3840, encoding='rgb8', data=pixels.tobytes())
    sizes = [old_size, api.h264_params(node)[:2]]
    for size in sizes:
        api.add_web_client(node, 1, size)
    api.on_image(node, msg)
    for w, h in sizes:
        _, frame = api.wait_for_rgb(node, 0, w, h)
        np.testing.assert_array_equal(frame, cv2.resize(pixels, (w, h), interpolation=cv2.INTER_AREA))
        api.add_web_client(node, -1, (w, h))
    assert not node._rgb_frames


def test_matching_h264_viewers_share_jpeg_resize(preview, encoder, monkeypatch):
    node, _ = encoder
    api = preview.CameraPreviewNode
    size = api.h264_params(node)[:2]
    resized = []
    resize = preview.cv2.resize

    def track_resize(*args, **kwargs):
        frame = resize(*args, **kwargs)
        resized.append(frame)
        return frame

    monkeypatch.setattr(preview.cv2, 'resize', track_resize)
    api.add_web_client(node, 1, size)
    api.add_web_client(node, 1, size)
    msg = preview.Image(width=1280, height=720, step=3840, encoding='rgb8', data=bytes(720 * 3840))
    api.on_image(node, msg)
    assert len(resized) == 1
    assert api.wait_for_rgb(node, 0, *size)[1] is resized[0]
    api.add_web_client(node, -1, size)
    assert api.wait_for_rgb(node, 0, *size)[1] is resized[0]
    api.add_web_client(node, -1, size)
    assert not node._rgb_frames


def test_bad_images_dropped_then_padded_image_accepted(preview, encoder):
    node, published = encoder
    for width, height, step, size in [(0, 2, 6, 12), (2, 0, 6, 0), (2, 2, 5, 10),
                                      (2, 2, 8, 0), (2, 2, 8, 15)]:
        preview.CameraPreviewNode.on_image(node, preview.Image(
            width=width, height=height, step=step, encoding='rgb8', data=bytes(size)))
        assert not published and node._frame_seq == 0
    preview.CameraPreviewNode.add_web_client(node, 1, (2, 2))
    padded = bytes(range(16))
    preview.CameraPreviewNode.on_image(node, preview.Image(
        width=2, height=2, step=8, encoding='rgb8', data=padded))
    assert len(published) == 1 and node._rgb_frames[(2, 2)].tobytes() == padded[:6] + padded[8:14]


@pytest.mark.parametrize('width', [320, 322])
def test_gst_packed_rgb_preserves_rows_and_frame_boundaries(preview, width):
    import numpy as np
    gst = shutil.which('gst-launch-1.0')
    if not gst or not shutil.which('gst-inspect-1.0'):
        pytest.skip('GStreamer tools unavailable')
    for plugin in ('rawvideoparse', 'videoconvert'):
        if subprocess.run(['gst-inspect-1.0', plugin], capture_output=True).returncode:
            pytest.skip(f'{plugin} unavailable')
    height = 180
    frames = np.random.default_rng(1).integers(256, size=(3, height, width, 3), dtype=np.uint8)
    # RGBA forces downstream conversion and removes any ambiguity about output stride.
    cmd = [gst, '-q', 'fdsrc', '!', *preview._gst_rgb_parser(width, height, '2997/100'),
           '!', 'videoconvert', '!', 'video/x-raw,format=RGBA', '!', 'fdsink', 'fd=1']
    result = subprocess.run(cmd, input=frames.tobytes(), capture_output=True, check=True, timeout=10)
    rgba = np.frombuffer(result.stdout, dtype=np.uint8).reshape(3, height, width, 4)
    np.testing.assert_array_equal(rgba[:, :, :, :3], frames)
