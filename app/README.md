# IMX219 Isaac ROS Camera

Captures calibrated RGB images from the IMX219 camera on Jetson.

这个 app 独占 CSI 传感器 0。Argus ISP 采集 NV12，原驱动在一个 CUDA kernel 内完成 180° 旋转、颜色校正和 RGB8 转换，随后发布 CUDA-backed ROS Image。Isaac ROS 5 使用 Lyrical 的 `rosidl::Buffer` 和 `cuda_buffer`。镜头标定、传感器和输出方向应配套使用。

| 接口 | 值 |
| --- | --- |
| 图像 | `/camera/image_raw`，`sensor_msgs/msg/Image`，RGB8 |
| 标定 | `/camera/camera_info`，`sensor_msgs/msg/CameraInfo`，与图像使用同一采集时间戳 |
| 坐标与队列 | `imx219_optical_frame`，best-effort，volatile，depth 3 |
| 采集 | mode 2，1280 × 720，30 Hz，180°，室内白平衡 |
| AivudaOS 配置 | `camera.ros_domain_id` 默认 -1（继承系统全局 `sys.ros2_default_domain_id`），`camera.exposure_us` 默认 5000，设为 0 启用自动曝光 |

升级后，已有的 `ros.domain_id` 继续生效。若同时配置了两个字段，优先使用 `camera.ros_domain_id`。继承模式先读取启动环境的 `ROS_DOMAIN_ID`，旧版平台未传入时读取工作区 `config/sys.yaml` 中的 `sys.ros2_default_domain_id`，两处均未设置时使用 0。显式指定的无效域会导致启动失败并报告其值。

使用机上 2026-10-22 实测的 rational_polynomial 标定。驱动和参数来自 airborne 的 `isaac_ros_imx219` 及 `calibrated_720p_5ms`。固定时间区间保留 GPU 缓冲区，便于同机消费者通过 CUDA IPC 读取。普通 CPU 图像订阅者会带来额外复制，测量采集时可订阅轻量的 CameraInfo 统计帧率。

## 安装和运行

目标为 Orin NX SM87、Isaac ROS 5.0、Lyrical、CUDA 13.2。最初的镜像在 JetPack 7.2.1 构建，本次适配设备为 L4T R39.2.2。需要原机本地 Docker 镜像 `airborne/isaac-ros:5.0-imx219-preview-20261022`、NVIDIA 容器运行时、正在运行的 nvargus-daemon、已接入的 IMX219 和当前用户的 Docker/video/render 权限。Python 3 和 PyYAML 用于读取宿主配置。机上已配置的主机 Jazzy 与容器 Lyrical 通过 DDS 互通。

在仓库根目录运行 `./pack.sh` 打包后，上传 aivudaAppStore 并在目标 aivudaOS 安装。安装和启动均使用包内 `isaac_runtime/`。若提示缺少启动器，请重新安装完整发布包。通过 aivudaOS 启停 app。安装钩子核对校验和、GPU 设备和镜像。运行时不需要原 airborne 检出目录。若它仍存在，会获取与旧服务相同的独占锁，以防两个管理器同时打开相机。检测 app 单独管理。

## 驱动源码与构建

`driver/` 包含实际使用的 Apache 2.0 C++ 驱动、CUDA 内核及转换测试。发布包内 `bin/` 和 `lib/` 包含在本机 Isaac ROS 镜像内重建的二进制。R39.2.2 实机首帧可能晚于原先的 250 ms 限时，驱动在启动时等待最多 2 s，后续维持原有的 250 ms 采集等待。原工作区提交为 `c896f25303f9d8ff400e943399fa2ec51290d15d`。在匹配的 Lyrical 开发容器内，挂载主机的 `/usr/src/jetson_multimedia_api`，从本 app 目录执行以下命令可重新构建。

```bash
source /opt/ros/lyrical/setup.bash
cmake -S driver -B /tmp/imx219-build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/imx219-build --parallel 2
ctest --test-dir /tmp/imx219-build --output-on-failure
cmake --install /tmp/imx219-build --prefix "$PWD"
sha256sum bin/imx219_node lib/libimx219_component.so config/imx219_mode2_1280x720_rot180.yaml > SHA256SUMS
```

`isaac_runtime/` 随包携带镜像启动、进程清理和 CUDA IPC 后端。源码仓库的两个视觉 app 共享该目录。内核 pidfd 监视宿主前台进程，应用停止或失去所有者时释放 ROS 进程。两项视觉服务共享宿主网络、IPC、PID 和 /tmp，以支持 DDS 及 CUDA 进程间缓冲区描述符。
