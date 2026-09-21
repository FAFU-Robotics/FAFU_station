#!/usr/bin/env bash
# Live arm on Ubuntu: CPython 3.10 matches fafu_motor cp310.
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

pick_py310() {
  if [ -n "${STATION_PYTHON:-}" ] && [ -x "$STATION_PYTHON" ]; then
    if "$STATION_PYTHON" -c "import sys; assert sys.version_info[:2]==(3,10)" 2>/dev/null; then
      echo "$STATION_PYTHON"
      return 0
    fi
  fi
  if [ -n "${CONDA_PREFIX:-}" ] && [ -x "$CONDA_PREFIX/bin/python" ]; then
    if "$CONDA_PREFIX/bin/python" -c "import sys; assert sys.version_info[:2]==(3,10)" 2>/dev/null; then
      echo "$CONDA_PREFIX/bin/python"
      return 0
    fi
  fi
  for cand in python3.10 python3; do
    if command -v "$cand" >/dev/null 2>&1; then
      if "$cand" -c "import sys; assert sys.version_info[:2]==(3,10)" 2>/dev/null; then
        command -v "$cand"
        return 0
      fi
    fi
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

PY310="$(pick_py310 || true)"
if [ -z "$PY310" ]; then
  echo "Python 3.10 not found."
  echo "Official fafu_motor is cp310. Do not use Python 3.11 for live USB."
  echo "Install: sudo apt install python3.10 python3.10-venv"
  echo "Or set STATION_PYTHON to a 3.10 interpreter."
  exit 1
fi

if ! "$PY310" -c "import websockets, yaml, numpy" 2>/dev/null; then
  echo "Installing station deps for Python 3.10 ..."
  "$PY310" -m pip install -q websockets PyYAML pywebview numpy pyrealsense2 PyGObject
fi

PIN=NO
if "$PY310" -c "import pinocchio" 2>/dev/null; then
  PIN=YES
fi

export PYTHONPATH="$APP_DIR${PYTHONPATH:+:$PYTHONPATH}"
echo "Live station: Python 3.10  STATION_ALLOW_LIVE_ARM=1  arm=fafu  allow-motion"
echo "PY=$PY310"
echo "pinocchio=$PIN"
echo "SDK=${FAFU_ARM_SDK:-}"
echo "E-stop: page button or unplug USB. Clear the workspace before Enable."
run_preflight "$PY310"
exec "$PY310" -u "$DESKTOP_PY" "$@"
