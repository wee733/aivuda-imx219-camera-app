# IMX219 Isaac ROS Camera

The Jetson driver captures calibrated 1280 × 720 RGB8 frames from the IMX219 and publishes them on `/camera/image_raw` with matching `/camera/camera_info`. Isaac ROS consumers can use CUDA buffers directly. A separate subscriber provides compressed ROS images and the browser preview on demand.

The [camera driver](app/driver/) includes its CUDA kernel, CMake project and tests. The app also bundles the [Isaac ROS runtime adapter](app/isaac_runtime/), the compiled Orin binaries and the camera calibration. It uses the provisioned Isaac ROS 5 Lyrical container on the Jetson. See the [hardware and build notes](app/README.md) and [default settings](app/config/default_config.yaml).

## Regression checks

Run `python3 -m pytest -q` with PyYAML, pytest and jsonschema installed. Source the ROS 2 Lyrical environment and install OpenCV and NumPy to exercise JPEG preview encoding as well. The launch tests simulate Docker: they verify settings and ownership across `exec`, not CSI capture or CUDA IPC.

For a Jetson release, also run the [driver tests](app/README.md) and check the camera with a matching CUDA subscriber in domain 0 and a nonzero domain. Verify `/camera/image_raw` is 1280 × 720 RGB8, its timestamps match `/camera/camera_info`, and the consumer actually imports a CUDA buffer. DDS discovery or a working JPEG preview alone does not prove zero-copy transfer.

To test the release artifacts **without opening the CSI sensor**, run these commands at the repo root in a matching Jetson development container with GPU access and private network/IPC namespaces (no host `/tmp`, no Argus socket):

```bash
source /opt/ros/lyrical/setup.bash
ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 tests/jetson_preview_smoke.py app/scripts/camera_preview.py
cmake -S tests/jetson_ipc -B /tmp/imx219-ipc-smoke && cmake --build /tmp/imx219-ipc-smoke
bash tests/jetson_ipc/run.sh /tmp/imx219-ipc-smoke/ipc_smoke app/isaac_runtime/prefix
```

To validate a camera/detector release pair, append the detector package’s `isaac_runtime/prefix` as a third argument to `run.sh`. Each process then loads its own package’s plugin. The first test requests a JPEG snapshot of synthetic color bars after an invalid frame. The second requires the subscriber in a separate process to read the expected bytes from a **CUDA** buffer in both DDS domains; a CPU fallback is not a pass.

**Jetson validation (2026-11-22).** The target-device test used the previous `libcuda_buffer_backend.so` build with SHA-256 `6aa9c703e7e731868a969deb36fef1f51fae18e4bb52ba47e9148e11f272a92f` in container image `066375d846e9a3d987e96e52c9b644f9af116db9d278796a5a45656e9eb399e4`. The rebuilt plugin still needs this on-device check. Separate publisher and subscriber processes verified all 384 bytes of a synthetic CUDA frame in domain 0 and again in domain 42 without CPU fallback. All three descriptor tests passed, covering expired buffer UIDs, invalid events and reference counting. This run did not open the CSI sensor or test a publisher and subscriber in different domains.

## Build the aivudaOS package

App ID `airborne_isaac_camera` · Version `1.1.1`

From a Git checkout, run the following command with Bash, Git, GNU tar, gzip and Python 3 with PyYAML installed.

```bash
./pack.sh
```

The archive in `dist/` has the app manifest at its root. The script uses the working copies of files tracked under `app/`. Stage new application files with Git before packaging. Upload the archive through the aivudaOS app installer.
