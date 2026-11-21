#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")" && pwd)
runtime="$root/isaac_runtime"
[[ -f "$runtime/run.py" ]] || { echo "Missing bundled launcher: $runtime/run.py" >&2; exit 1; }
[[ -S /tmp/argus_socket ]] || { echo 'Argus daemon socket /tmp/argus_socket is unavailable.' >&2; exit 1; }
[[ -c /dev/video0 ]] || { echo 'IMX219 device /dev/video0 is unavailable.' >&2; exit 1; }
config=${AIVUDA_APP_CONFIG_PATH:-"$root/config/default_config.yaml"}
exec /usr/bin/python3 "$runtime/run.py" camera "$root" --config "$config"
