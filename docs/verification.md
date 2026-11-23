# Camera release verification

Run `python3 -m pytest -q` at the repository root. Run driver CTest in the target image as described in [the developer guide](../README_dev.md). A synthetic CUDA inter-process test checks the installed buffer backend on DDS domains 0 and 42.

```bash
source /opt/ros/lyrical/setup.bash
cmake -S tests/jetson_ipc -B /tmp/imx219-ipc
cmake --build /tmp/imx219-ipc
bash tests/jetson_ipc/run.sh /tmp/imx219-ipc/ipc_smoke app/isaac_runtime/prefix
```

For hardware acceptance, start the app with a live IMX219. Run the subscriber below inside the configured Isaac ROS environment with the same DDS domain and network configuration as the camera. Mount `tests/` into the container. The subscriber validates timestamps against CameraInfo, frame identity, SPS/PPS at IDR and ordered Annex B access units.

```bash
python3 tests/jetson_video_smoke.py /tmp/camera.h264
ffprobe -v error -count_frames -show_entries stream=codec_name,profile,width,height,has_b_frames,nb_read_frames /tmp/camera.h264
ffmpeg -v error -xerror -c:v h264 -i /tmp/camera.h264 -f null -
```

Repeat after selecting each hardware mode and restarting the app. In official mode, confirm the default exposes only `/camera/video/h264` among the public video topics. Enable `stream.publish_compressed_image`, restart, and pass `--compressed-image` to validate the alternate topic. Native mode logs creation at first demand and release after two idle seconds. In off mode check the camera info rate with `ros2 topic hz /camera/camera_info` and verify that the camera publishes no video topic. For a ground-station check enable the Zenoh on-demand H.264 route, connect Foxglove to the ground bridge and select `/<robot>/camera/video/h264` in the Image panel.

On the release Orin NX, native NVENC produced two independent 36-frame streams at 960×540 and 14.79 Hz with a 15 Hz target. The session closed after disconnect and reopened successfully. Isaac ROS produced 36 Foxglove frames at 14.79 Hz and 36 CompressedImage frames at 15.00 Hz, both at 1280×720. Timestamp checks matched the published camera calibration messages. FFmpeg decoded all three paths as H.264 Constrained Baseline with zero B-frames. In off mode 60 CameraInfo messages measured 30.01 Hz and the node advertised only the two camera data topics plus standard ROS infrastructure. This test used an isolated versioned app directory and ROS domain 141.

For mixed ROS distributions, keep a ground station reader on the forwarded video
while a Lyrical reader in the camera container subscribes and exits. Check that
the ground station keeps receiving at the encoder rate after that reader exits,
then repeat a ground station disconnect and reconnect. Decode starting at an
IDR with SPS/PPS. Run this check for both public video message types when the
Isaac compatibility output is enabled. The raw CUDA path can be checked
independently by importing an RGB frame from a second Lyrical process.

On the ground station, source its ROS environment and run
`python3 tests/jetson_mixed_readers.py JETSON_SSH_ALIAS --prefix /uav1` with
the video egress and ingress enabled. The check waits for video, briefly adds a
reader in the camera container over SSH, then requires fresh ground-station
frames after that reader leaves. With the Isaac compatibility output and its
test route enabled, repeat with `--compressed-image`.
