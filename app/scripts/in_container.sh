#!/usr/bin/env bash
set -eo pipefail
mode=${1:?Missing stream mode}
settings=${2:?Missing camera settings}
export AMENT_PREFIX_PATH="/app:$AMENT_PREFIX_PATH"
if [[ ! -d /opt/ros/lyrical/share/foxglove_msgs ]]; then
  echo 'Install ros-lyrical-foxglove-msgs in the Isaac ROS image.' >&2
  exit 1
fi
if [[ "$mode" == isaac_ros_h264 && ! -r /opt/ros/lyrical/lib/libisaac_ros_h264_encoder_node.so ]]; then
  echo 'Install ros-lyrical-isaac-ros-h264-encoder in the Isaac ROS image.' >&2
  exit 1
fi
if [[ "$mode" == isaac_ros_h264 ]]; then
  export IMX219_SETTINGS="$settings"
  exec ros2 launch /app/scripts/camera.launch.py
fi
read -r exposure width fps bitrate interval < <(python3 - "$settings" <<'PY'
import json
import sys
with open(sys.argv[1], encoding='utf-8') as source:
    cfg = json.load(source)
stream = cfg['stream']
print(cfg['exposure_us'], stream['width'], stream['fps'], stream['bitrate_kbps'], stream['iframe_interval'])
PY
)
exec /app/bin/imx219_node --ros-args -r __ns:=/camera --params-file /app/config/driver.yaml \
  -p "exposure_us:=$exposure" -p "stream.mode:='$mode'" -p "stream.width:=$width" \
  -p "stream.fps:=$fps" -p "stream.bitrate_kbps:=$bitrate" -p "stream.iframe_interval:=$interval" \
  --log-level serialize_buffer_with_endpoint:=warn --log-level deserialize_buffer_with_endpoint:=warn
