# IMX219 Isaac ROS Camera

App `airborne_isaac_camera` 2.0.3 publishes calibrated 1280 × 720 RGB8 at 30 Hz through the Lyrical CUDA buffer backend. Hardware H.264 video is available on demand at `/camera/video/h264` for Foxglove and Zenoh. Select native NVENC, the official Isaac ROS encoder, or `off` in the AivudaOS configuration center. In Isaac mode, enable `stream.publish_compressed_image` if a legacy subscriber needs `/camera/image_compressed`.

See the [hardware and configuration notes](app/README.md), [developer guide](README_dev.md), and [release checks](docs/verification.md). The package includes the Orin binaries, calibration and runtime adapter for the provisioned Isaac ROS 5 image.

Run `python3 -m pytest -q` for host tests and `./pack.sh` to build the App Store archive. The packager includes the working copies of tracked files in `app/`. Stage new files before packaging. AivudaOS supports preserving the effective camera DDS domain across upgrades through `x-ros-domain-migration`.
