# IMX219 机载相机

App 使用 Orin NX 的 Argus ISP 采集 sensor 0、mode 2，以 1280×720、30 Hz 发布旋转 180° 的 RGB8 CUDA 图像和相同采集时间戳的相机内参。标定使用机上实测的 rational_polynomial 模型。原图及内参话题分别为 `/camera/image_raw` 和 `/camera/camera_info`。通用 CPU 图像订阅者会触发整帧下载，测帧率时可订阅 CameraInfo。

| 配置 | 取值和作用 |
| --- | --- |
| `camera.ros_domain_id` | 默认 -1，继承运行环境 ROS_DOMAIN_ID 或工作区系统配置，均缺失时为 0 |
| `camera.exposure_us` | 自动曝光上限，默认 5000 µs，0 表示不额外设限；室内可试 8000 µs 或补光 |
| `stream.mode` | `native_nvenc` 默认、`isaac_ros_h264`、`off` |
| `stream.publish_compressed_image` | 默认 `false`。在 `isaac_ros_h264` 模式下设为 `true`，额外发布 `/camera/image_compressed` 给旧版订阅者 |
| `stream.width` | 原生编码宽度 640、960 或 1280，固定 16:9。默认 960×540 |
| `stream.fps` | 两条编码路线的输入限速，1–30 Hz，默认 15 Hz |
| `stream.bitrate_kbps` | 原生编码和 Isaac custom 的目标码率 300–8000 kbit/s，默认 1500 |
| `stream.iframe_interval` | 原生编码和 Isaac 自定义配置的关键帧间隔，默认 15 帧 |
| `stream.config` | 官方编码预设 `pframe_cqp`、`iframe_cqp` 或 `custom`，默认 `custom` |
| `stream.qp`、`stream.hw_preset`、`stream.profile` | Isaac 自定义配置使用，默认为 24、0、1 |

在宿主验证镜头专用 ISP override 后使用自动白平衡、0 EV 和 1:1:1 RGB 增益。color_gain_* 只影响 RGB8，不改变送给原生 NVENC 的 NV12。

原生路线按需创建硬件编码器，用 CUDA 直接从 Argus YUV 平面缩放并旋转到编码器的 NV12 DMA-BUF。输出为 `/camera/video/h264`（`foxglove_msgs/msg/CompressedVideo`、SensorDataQoS）。最后一位订阅者离开即停止送帧，2 秒后释放 NVENC 会话。官方路线将相机、官方 EncoderNode 装入同一进程，从私有话题接收按需投递的 1280×720 GPU 图像。该路线默认只输出上述公共视频话题，开启兼容开关后也可订阅 `sensor_msgs/msg/CompressedImage`。`off` 仅运行原图与内参主干。Isaac ROS 5.0 的 custom 使用 CBR，两个 CQP 预设由官方组件指定量化参数。修改配置后重新启动 App。

机上需安装本地镜像 `airborne/isaac-ros:5.0-imx219-preview-20261022`、NVIDIA 容器运行时和 nvargus-daemon，并接入 IMX219。当前用户需要 Docker、video 和 render 访问权限。预安装钩子校验 GPU、镜像及随包二进制。容器启动器会通过锁协调不同版本及旧 airborne 相机服务对 CSI 的使用。

原生 NVENC 保留 Argus YUV 采样值；按实际 EGL 输入设置 NV12 surface、V4L2 量化范围与 H.264 VUI。在本机 mode 2 中为全范围 BT.601 (SMPTE 170M)，避免解码端误按电视范围扩展。H.264 为有损编码，不保证和 RGB8 逐像素相等。
