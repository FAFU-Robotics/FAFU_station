#!/usr/bin/env bash
# ASCII alias of 启动真机.sh (same role as start_live_arm.bat on Windows).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
exec bash "$HERE/启动真机.sh" "$@"
