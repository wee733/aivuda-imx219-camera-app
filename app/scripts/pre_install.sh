#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd)
runtime="$root/isaac_runtime"
[[ -d "$runtime" ]] || runtime="$root/../isaac_runtime"
bash "$runtime/check.sh"
cd "$root"
sha256sum --check SHA256SUMS
