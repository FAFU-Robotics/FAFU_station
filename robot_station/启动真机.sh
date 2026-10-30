#!/usr/bin/env bash
# Source launch selects a Python matching the built SDK extension.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"
export STATION_ALLOW_LIVE_ARM=1

if [ -f "$ROOT/app/station_desktop.py" ]; then
  APP_DIR="$ROOT/app"
  DESKTOP_PY="$ROOT/app/station_desktop.py"
else
  APP_DIR="$ROOT"
  DESKTOP_PY="$ROOT/station_desktop.py"
fi

if [ -d "$APP_DIR/urdf" ]; then
  export STATION_URDF_DIR="$APP_DIR/urdf"
elif [ -d "$ROOT/urdf" ]; then
  export STATION_URDF_DIR="$ROOT/urdf"
fi

if [ -f "$APP_DIR/vendor/fafu_arm_sdk/fafu_robot_python/fafu_robot_controller.py" ]; then
  export FAFU_ARM_SDK="$APP_DIR/vendor/fafu_arm_sdk"
elif [ -f "$ROOT/vendor/fafu_arm_sdk/fafu_robot_python/fafu_robot_controller.py" ]; then
  export FAFU_ARM_SDK="$ROOT/vendor/fafu_arm_sdk"
fi

supports_sdk() {
  local candidate="$1"
  [ -x "$candidate" ] || return 1
  LD_LIBRARY_PATH="${FAFU_ARM_SDK:+$FAFU_ARM_SDK/fafu_robot_python:}${LD_LIBRARY_PATH:-}" \
    "$candidate" -c "import sys; sys.path.insert(0, sys.argv[1]); import fafu_motor; import fafu_robot_controller" \
    "${FAFU_ARM_SDK:-}/fafu_robot_python" >/dev/null 2>&1
}

pick_python() {
  if [ -n "${STATION_PYTHON:-}" ]; then
    supports_sdk "$STATION_PYTHON" || return 1
    echo "$STATION_PYTHON"
    return 0
  fi
  for candidate in "${VIRTUAL_ENV:-}/bin/python" "${CONDA_PREFIX:-}/bin/python"; do
    if supports_sdk "$candidate"; then echo "$candidate"; return 0; fi
  done
  for candidate in python3.12 python3.10 python3; do
    local executable
    executable="$(command -v "$candidate" || true)"
    if [ -n "$executable" ] && supports_sdk "$executable"; then echo "$executable"; return 0; fi
  done
  return 1
}

run_preflight() {
  local py="$1"
  if "$py" -c "import robot_station.linux_preflight" 2>/dev/null; then
    PYTHONPATH="$APP_DIR${PYTHONPATH:+:$PYTHONPATH}" "$py" -m robot_station.linux_preflight --interactive
  fi
}

if [ -x "$ROOT/runtime/python310/bin/python3" ] || [ -x "$ROOT/runtime/python310/bin/python" ]; then
  export STATION_PORTABLE=1
  export STATION_INSTALL_ROOT="$ROOT"
  export PYTHONNOUSERSITE=1
  export PIP_USER=0
  unset PYTHONSTARTUP || true
  unset PYTHONHOME || true
  unset PYTHONUSERBASE || true
  export PYTHONPATH="$APP_DIR"
  PY="$ROOT/runtime/python310/bin/python3"
  if [ ! -x "$PY" ]; then
    PY="$ROOT/runtime/python310/bin/python"
  fi
  export PATH="$ROOT/runtime/python310/bin:$PATH"
  export LD_LIBRARY_PATH="$ROOT/runtime/python310/lib:${FAFU_ARM_SDK:+$FAFU_ARM_SDK/fafu_robot_python:}${LD_LIBRARY_PATH:-}"
  echo "Live station: portable Python  STATION_ALLOW_LIVE_ARM=1  arm=fafu  allow-motion"
  echo "SDK=${FAFU_ARM_SDK:-}"
  echo "Private runtime in place. Does not change system PATH."
  run_preflight "$PY"
  exec "$PY" -u "$DESKTOP_PY" "$@"
fi

if [ -z "${FAFU_ARM_SDK:-}" ]; then
  for cand in "$ROOT/../fafu_arm_sdk" "$HOME/fafu_arm_sdk"; do
    if [ -f "$cand/fafu_robot_python/fafu_robot_controller.py" ]; then
      export FAFU_ARM_SDK="$cand"
      break
    fi
  done
fi

STATION_PY="$(pick_python || true)"
if [ -z "$STATION_PY" ]; then
  echo "No Python could import the selected SDK extension."
  echo "Build fafu_motor for the target Python first, including Python 3.12."
  echo "Build: bash ../fafu_arm_sdk/fafu_robot_cpp/linux/build.sh --python python3.12"
  echo "Or set STATION_PYTHON to a compatible interpreter."
  exit 1
fi

if [ "${1:-}" = "--check-python" ]; then echo "$STATION_PY"; exit 0; fi

if ! "$STATION_PY" -c "import websockets, yaml, numpy, webview" 2>/dev/null; then
  echo "Installing station dependencies ..."
  "$STATION_PY" -m pip install -q websockets PyYAML pywebview numpy pyrealsense2 PyGObject
fi

PIN=NO
if "$STATION_PY" -c "import pinocchio" 2>/dev/null; then
  PIN=YES
fi

export PYTHONPATH="$APP_DIR${PYTHONPATH:+:$PYTHONPATH}"
echo "Live station: source Python  STATION_ALLOW_LIVE_ARM=1  arm=fafu  allow-motion"
echo "PY=$STATION_PY"
echo "pinocchio=$PIN"
echo "SDK=${FAFU_ARM_SDK:-}"
echo "E-stop: page button or unplug USB. Clear the workspace before Enable."
run_preflight "$STATION_PY"
exec "$STATION_PY" -u "$DESKTOP_PY" "$@"
