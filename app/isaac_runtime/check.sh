#!/usr/bin/env bash
# Read-only; never opens the camera or starts a node or container.
set -euo pipefail
export LC_ALL=C
cd -- "$(dirname -- "$0")"
case "$#:${1-}" in
  0:) ;;
  1:--artifact-only) ;;
  *) echo "Usage: $0 [--artifact-only]" >&2; exit 2 ;;
esac
sha256sum --check SHA256SUMS
plugin=prefix/lib/libcuda_buffer_backend.so
header=$(readelf -hW "$plugin")
dynamic=$(readelf -dW "$plugin")
symbols=$(readelf --dyn-syms -W "$plugin")
grep -Eq 'Machine:[[:space:]]+AArch64$' <<<"$header"
grep -Eq 'Type:[[:space:]]+DYN ' <<<"$header"
grep -Eq 'FLAGS_1.*NODELETE' <<<"$dynamic"
grep -Fq 'Shared library: [/opt/ros/lyrical/lib/libcuda_buffer.so]' <<<"$dynamic"
grep -Fq 'Shared library: [libcudart.so.13]' <<<"$dynamic"
# The driver and plugin must use the same VMM allocation pool across DSOs.
grep -Eq 'UND[[:space:]]+_ZN19cuda_buffer_backend25get_or_create_global_poolEv$' <<<"$symbols"
[[ "${1-}" != --artifact-only ]] || exit 0
[[ "$(uname -m)" == aarch64 ]] || { echo 'This build targets Jetson Orin aarch64.' >&2; exit 1; }
docker image inspect airborne/isaac-ros:5.0-imx219-preview-20261022 --format 'Isaac ROS base image {{.Id}}'
[[ -r /dev/nvhost-gpu ]] || { echo 'Missing access to Jetson GPU.' >&2; exit 1; }
