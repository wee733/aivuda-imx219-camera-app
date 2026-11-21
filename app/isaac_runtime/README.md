# Isaac ROS 5 sensor runtime

The camera and AprilTag archives each include this directory. The runtime starts the installed Isaac ROS image with the application user, a read-only app mount, local GPU device access, and shared host networking, IPC, PID namespace and /tmp. CUDA IPC exchanges buffer descriptors between local ROS processes. Startup clears the cached image’s DISPLAY to allow headless EGL on this Jetson. The in-container supervisor forwards shutdown and watches the foreground Docker client with a Linux pidfd.

The launcher exports the resolved `ROS_DOMAIN_ID` to both DDS and the bundled CUDA backend. The backend looks up the peer's DDS endpoint GID in that domain's local endpoint registry before exporting an IPC descriptor. Both peers must use the same registry: on a nonzero domain, upgrade consumers that still use the old domain-0-only backend, or use ordinary CPU image transport.

**Receiver failure contract (Lyrical 9.4.x Fast DDS).** The installed endpoint-aware deserializer does not accept a null implementation or catch plugin exceptions. If a descriptor is stale or an advertised write event cannot be imported, the plugin returns an empty CPU buffer; it never substitutes fabricated pixels. This does *not* automatically drop the ROS message. Subscribers, including GPU consumers, must reject positive-sized RGB8 images unless `step >= 3 * width` and `data.size() == step * height` before accessing memory. The bundled preview does this; third-party subscribers must implement the same check. Valid CPU-backed images remain supported.

### Domain and middleware evidence

In [Lyrical Fast DDS 9.4.10](https://github.com/ros2/rmw_fastrtps/tree/e175f84a861e8a66fb53035d27d34bc3212c161e/rmw_fastrtps_cpp/src), `publisher.cpp` and `subscription.cpp` notify the loaded backends of local endpoint creation; `rmw_publisher.cpp` and `rmw_subscription.cpp` notify them of remote GIDs. A native Lyrical 9.4.10 loopback test on domain 194 confirmed this with two real `sensor_msgs/Image` processes and a recording backend: each peer's discovered GID exactly matched the other peer's created GID, with the correct endpoint kind and a null `topic_type`. The callback's `rmw_topic_endpoint_info_t` has no domain field. It is the **plugin**, not a second domain-0 middleware registry, that registers and queries the GIDs. Its shared-memory key also incorporates the hostname. The launcher uses one canonical decimal domain (0–232) per process; contexts that override that domain internally are not supported. A peer must share the hostname, IPC namespace, UID and GPU for inter-process CUDA IPC. A lookup miss returns no descriptor, which Lyrical's `serialize_buffer_with_endpoint()` encodes as CPU bytes. This does not bridge separate DDS domains.

### Rebuilding the shipped plugin

The plugin source is based on `ros2/rosidl_buffer_backends` 0.1.2; see `cuda_buffer_backend/UPSTREAM.txt`. The camera component and previous plugin both import `cuda_buffer_backend::get_or_create_global_pool()` from `/opt/ros/lyrical/lib/libcuda_buffer.so`. The stock 0.1.2 headers instead use an inline, per-DSO pool. The old binaries' pool import and CUDA event diagnostics (including header lines 48/195/198 and 128/131/226) match upstream **`d26b5aaf3377a2b019070866ecca4b67c50a2949`** for the companion `cuda_buffer` headers. A stock CUDA-12 Noble `libcuda_buffer.so` is **not** a replacement for the image's CUDA-13 companion.

- **Native:** In a development copy of the matching `airborne/isaac-ros:5.0-imx219-preview-20261022` image with its patched `cuda_buffer` headers, source `/opt/ros/lyrical/setup.bash` and use `colcon build --base-paths /path/to/cuda_buffer_backend --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release` from an empty workspace. Preserve the global pool import and `NODELETE` flag.
- **Offline cross-build used for this ELF:** Stage the extracted Ubuntu Noble `g++-13-aarch64-linux-gnu` toolchain (13.3.0-6ubuntu2~24.04.1cross1), cross libc/libstdc++ and binutils, ARM64 `libconsole-bridge-dev`, NVIDIA's [signed Noble Lyrical arm64 packages](https://isaac.download.nvidia.com/isaac-ros/ubuntu/main/dists/noble/) and JetPack CUDA 13.2.2 SDK (cudart, cudart-dev, driver-dev, crt and cccl **13.2.86**, `targets/sbsa-linux`) under one temporary sysroot. ROS inputs include `cuda_buffer_backend_msgs` 0.1.2, `pluginlib`, `class_loader`, `rcutils`, `rcpputils`, `rmw` and their rosidl buffer, runtime and typesupport headers/libraries. The buildfarm `InRelease` signature and package SHA-256s were checked. No host ROS headers or libraries are used.

  ```sh
  # COMPANION is cuda_buffer_backend/cuda_buffer from the pinned d26b5aa revision.
  out=$(mktemp -d)
  bash cuda_buffer_backend/cross-build.sh "$SYSROOT" "$COMPANION" "$out"
  # Only libcuda_buffer_backend.so is distributable here, NOT libcuda_buffer.so or libcuda.so.
  install -m 755 "$out/libcuda_buffer_backend.so" prefix/lib/libcuda_buffer_backend.so
  sha256sum prefix/lib/libcuda_buffer_backend.so > SHA256SUMS
  bash check.sh --artifact-only
  ```

### What was verified

`check.sh --artifact-only` verifies the shipped checksum, AArch64 shared ELF, CUDA 13 and image companion dependencies, process-wide pool import, and `NODELETE`. The default pre-install check also requires the target architecture, local Docker image and GPU device. On the x86 build host, QEMU AArch64 loaded the plugin through ROS `class_loader` with immediate symbol resolution. Separate emulated processes registered publisher/subscriber GIDs on domains 0 (unset), 37 and 232: same-domain discovery succeeded; foreign domain, unknown GID, device/UID mismatch and malformed domain tests were rejected. The test mocked **only GPU pool initialization/capability**, and used a driver link stub and synthetic GIDs, not a physical GPU or a running DDS graph. The companion was reconstructed for linking/testing, not copied from the private image. Actual Jetson driver loading, camera frames, write-event handling and end-to-end GPU transfer with a matching consumer still require a target smoke test.
