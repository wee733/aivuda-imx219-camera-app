# IMX219 Isaac ROS Camera

The Jetson driver captures calibrated 1280 × 720 RGB8 frames from the IMX219 and publishes them on `/camera/image_raw` with matching `/camera/camera_info`. Isaac ROS consumers can use CUDA buffers directly. A separate subscriber provides compressed ROS images and the browser preview on demand.

The [camera driver](app/driver/) includes its CUDA kernel, CMake project and tests. The app also bundles the [Isaac ROS runtime adapter](app/isaac_runtime/), the compiled Orin binaries and the camera calibration. It uses the provisioned Isaac ROS 5 Lyrical container on the Jetson. See the [hardware and build notes](app/README.md) and [default settings](app/config/default_config.yaml).

## Build the aivudaOS package

App ID `airborne_isaac_camera` · Version `1.1.0`

From a Git checkout, run the following command with Bash, Git, GNU tar, gzip and Python 3 with PyYAML installed.

```bash
./pack.sh
```

The archive in `dist/` has the app manifest at its root. The script uses the working copies of files tracked under `app/`. Stage new application files with Git before packaging. Upload the archive through the aivudaOS app installer.
