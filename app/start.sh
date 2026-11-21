#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")" && pwd)
runtime="$root/isaac_runtime"
[[ -d "$runtime" ]] || runtime="$root/../isaac_runtime"
[[ -S /tmp/argus_socket ]] || { echo 'Argus daemon socket /tmp/argus_socket is unavailable.' >&2; exit 1; }
[[ -c /dev/video0 ]] || { echo 'IMX219 device /dev/video0 is unavailable.' >&2; exit 1; }
config=${AIVUDA_APP_CONFIG_PATH:-"$root/config/default_config.yaml"}
data_dir="${AIVUDA_APP_RUNTIME_DATA_PATH:-/tmp/aivuda-isaac-camera}"
mkdir -p "$data_dir"
export AIVUDA_APP_RUNTIME_DATA_PATH="$data_dir"

settings=$(/usr/bin/python3 - "$config" "$data_dir/app_config.json" <<'PY'
import json
import math
from pathlib import Path
import sys
import yaml

cfg_path, out_path = Path(sys.argv[1]), Path(sys.argv[2])
config = yaml.safe_load(cfg_path.read_text(encoding='utf-8')) or {}
domain = config['ros']['domain_id']
exposure = config['camera']['exposure_us']
if type(domain) is not int or not 0 <= domain <= 232:
    raise SystemExit('ros.domain_id must be an integer in [0, 232]')
if type(exposure) is not int or not 0 <= exposure <= 33333:
    raise SystemExit('camera.exposure_us must be an integer in [0, 33333]')

preview = config.get('preview') or {}
enabled = bool(preview.get('enabled', True))
fps = float(preview.get('fps', 15.0))
width = int(preview.get('width', 960))
quality = int(preview.get('jpeg_quality', 85))
codec = str(preview.get('codec', 'auto'))
bitrate = int(preview.get('h264_bitrate_kbps', 3000))
if not math.isfinite(fps) or not 1.0 <= fps <= 30.0:
    raise SystemExit('preview.fps must be in [1, 30]')
if not 0 <= width <= 1280 or (width > 0 and width < 320):
    raise SystemExit('preview.width must be 0 or in [320, 1280]')
if not 40 <= quality <= 95:
    raise SystemExit('preview.jpeg_quality must be in [40, 95]')
if codec not in ('auto', 'nvenc_h264', 'jpeg'):
    raise SystemExit('preview.codec must be auto, nvenc_h264 or jpeg')
if not 500 <= bitrate <= 12000:
    raise SystemExit('preview.h264_bitrate_kbps must be in [500, 12000]')

payload = {
    'ros_domain_id': domain,
    'exposure_us': exposure,
    'preview': {
        'enabled': enabled,
        'fps': fps,
        'width': width,
        'jpeg_quality': quality,
        'codec': codec,
        'h264_bitrate_kbps': bitrate,
        'ui_port': 19311,
    },
}
tmp = out_path.with_suffix('.tmp')
tmp.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
tmp.replace(out_path)
print(domain, exposure, '1' if enabled else '0')
PY
)
read -r domain exposure preview_enabled <<<"$settings"
exec /usr/bin/python3 "$runtime/run.py" camera "$root" "$domain" \
    /app/scripts/in_container.sh "$exposure" "$preview_enabled" /data/app_config.json
