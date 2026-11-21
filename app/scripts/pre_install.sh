#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd)
runtime="$root/isaac_runtime"
[[ -f "$runtime/run.py" ]] || { echo "Missing bundled launcher: $runtime/run.py" >&2; exit 1; }
bash "$runtime/check.sh"
cd "$root"
sha256sum --check SHA256SUMS
