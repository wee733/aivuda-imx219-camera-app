# Camera development

The calibrated Argus/CUDA producer, the native hardware encoder and the GPU kernels live in `app/driver/`. The native encoder imports DMA-BUF surfaces from the Jetson Multimedia API. The official encoder receives the timestamped CUDA RGB buffer via an intra-process input. The app retains the upstream allocation for external CUDA IPC consumers. The companion's public compressed topics use standard ROS CDR.

Build on Orin NX (SM87, L4T R39.2.2) inside the target Isaac ROS 5 Lyrical image. Mount the host's `/usr/src/jetson_multimedia_api` read-only at the same path and mount the application directory at `/app`. The image needs the CUDA 13 compiler, ROS development packages and foxglove_msgs.

```bash
source /opt/ros/lyrical/setup.bash
cmake -S /app/driver -B /tmp/imx219-build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/imx219-build --parallel 2
ctest --test-dir /tmp/imx219-build --output-on-failure
cmake --install /tmp/imx219-build --prefix /app
cd /app
sha256sum bin/imx219_node lib/libimx219_component.so config/imx219_mode2_1280x720_rot180.yaml > SHA256SUMS
```

On the host, config resolution and schema tests use Python 3.8+, PyYAML, pytest and jsonschema. The calibration, two ELF artifacts, ament index resources and launch files must be included in the archive. See the [hardware release checks](docs/verification.md).
