#!/usr/bin/env bash
# Docker entry: repo is mounted at /src (FAFU_station root).
set -euo pipefail
cd /src/robot_station
export FAFU_ARM_SDK="${FAFU_ARM_SDK:-/src/fafu_arm_sdk}"
export APPIMAGE_EXTRACT_AND_RUN=1
mkdir -p /src/robot_station/dist
LOG=/src/robot_station/dist/appimage-build.log
set +e
bash packaging/linux/build_appimage.sh "$@" 2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}
set -e
if [ "$rc" -ne 0 ]; then
  echo "build_appimage.sh failed; log: $LOG" >&2
  exit "$rc"
fi
exit 0
