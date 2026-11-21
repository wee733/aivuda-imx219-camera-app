#!/usr/bin/env bash
# Installation checks are read-only and never open the camera or start a node.
set -euo pipefail
[[ "$(uname -m)" == aarch64 ]] || { echo 'This build targets Jetson Orin aarch64.' >&2; exit 1; }
docker image inspect airborne/isaac-ros:5.0-imx219-preview-20261022 --format 'Isaac ROS base image {{.Id}}'
[[ -r /dev/nvhost-gpu ]] || { echo 'Missing access to Jetson GPU.' >&2; exit 1; }
cd -- "$(dirname -- "$0")"
sha256sum --check SHA256SUMS
