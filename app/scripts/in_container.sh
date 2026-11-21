#!/usr/bin/env bash
set -eo pipefail
exposure="${1:-5000}"
preview_enabled="${2:-1}"
config_json="${3:-/data/app_config.json}"

preview_pid=""
camera_pid=""

cleanup() {
  if [[ -n "$preview_pid" ]] && kill -0 "$preview_pid" 2>/dev/null; then
    kill -INT "$preview_pid" 2>/dev/null || true
  fi
  if [[ -n "$camera_pid" ]] && kill -0 "$camera_pid" 2>/dev/null; then
    kill -INT "$camera_pid" 2>/dev/null || true
  fi
}
trap cleanup INT TERM

/app/bin/imx219_node --ros-args -r __ns:=/camera --params-file /app/config/driver.yaml \
  -p "exposure_us:=$exposure" --log-level serialize_buffer_with_endpoint:=warn \
  --log-level deserialize_buffer_with_endpoint:=warn &
camera_pid=$!

if [[ "$preview_enabled" == "1" ]]; then
  /usr/bin/python3 /app/scripts/camera_preview.py "$config_json" &
  preview_pid=$!
fi

wait "$camera_pid"
rc=$?
if [[ -n "$preview_pid" ]] && kill -0 "$preview_pid" 2>/dev/null; then
  kill -INT "$preview_pid" 2>/dev/null || true
  wait "$preview_pid" 2>/dev/null || true
fi
exit "$rc"
