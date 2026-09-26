#!/usr/bin/env bash
set -euo pipefail
proto_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
roo_pb_dir="${ROO_PB_DIR:-$proto_dir/../../roo_pb}"
python3 "$roo_pb_dir/tools/generate.py" -I "$proto_dir" \
  --out "$proto_dir/../src" roo_transceivers.proto
if command -v clang-format >/dev/null 2>&1; then
  clang-format -i "$proto_dir/../src/roo_transceivers.pb.h"
fi
