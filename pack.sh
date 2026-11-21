#!/usr/bin/env bash
set -euo pipefail

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
out_dir="${1:-$root/dist}"
mkdir -p -- "$out_dir"
name=$(python3 - "$root/app/manifest.yaml" <<'PYTHON'
from pathlib import Path
import re
import sys

import yaml

manifest = yaml.safe_load(Path(sys.argv[1]).read_text())
fields = [str(manifest[key]) for key in ("app_id", "version")]
if not all(re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._+-]*", field) for field in fields):
    raise SystemExit("Invalid app ID or version in manifest.yaml")
print("-".join(fields) + ".tar.gz")
PYTHON
)
tmp=$(mktemp -- "$out_dir/.package.XXXXXXXX")
trap 'rm -f -- "$tmp"' EXIT
git -C "$root/app" ls-files -z -- . |
    tar -C "$root/app" --no-recursion --null --verbatim-files-from \
        --mtime=@0 --owner=0 --group=0 --numeric-owner --mode='u+rwX,go+rX,go-w' \
        -cf - -T - | gzip -n > "$tmp"
chmod 644 "$tmp"
mv -f -- "$tmp" "$out_dir/$name"
printf '%s\n' "$out_dir/$name"
