#!/usr/bin/env bash
# Developer/CI check of an AppDir / AppImage. Does not launch the GUI.
set -euo pipefail

PACK_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$PACK_DIR/../.." && pwd)"
APPDIR=""
IMAGE=""
ALLOW_NO_SDK=0

while [ $# -gt 0 ]; do
  case "$1" in
    --appdir) shift; APPDIR="$1" ;;
    --image) shift; IMAGE="$1" ;;
    --allow-no-sdk) ALLOW_NO_SDK=1 ;;
    *)
      echo "unknown arg: $1" >&2
      exit 1
      ;;
  esac
  shift
done

APPDIR="${APPDIR:-$REPO/dist/FAFUArmStation.AppDir}"
IMAGE="${IMAGE:-$REPO/dist/FAFUArmStation-x86_64.AppImage}"

fail=0
note() { echo "verify: $*"; }
die() { echo "FAIL: $*" >&2; fail=1; }

if [ ! -d "$APPDIR" ]; then
  echo "AppDir not present: $APPDIR"
  exit 1
fi

[ -x "$APPDIR/AppRun" ] || die "AppRun missing or not executable"
grep -q 'STATION_PORTABLE' "$APPDIR/AppRun" || die "AppRun must set STATION_PORTABLE"
grep -q 'STATION_ALLOW_LIVE_ARM' "$APPDIR/AppRun" || die "AppRun must set STATION_ALLOW_LIVE_ARM"
grep -q 'GDK_BACKEND' "$APPDIR/AppRun" || die "AppRun must set GDK_BACKEND"
grep -q 'linux_preflight' "$APPDIR/AppRun" || die "AppRun must run linux_preflight"
grep -q 'station_desktop.py' "$APPDIR/AppRun" || die "AppRun must exec station_desktop.py"
[ -f "$APPDIR/fafu-arm-station.desktop" ] || die "desktop file missing"
[ -f "$APPDIR/app/station_desktop.py" ] || die "app/station_desktop.py missing"
[ -f "$APPDIR/app/run_station.py" ] || die "app/run_station.py missing"
[ -x "$APPDIR/runtime/python310/bin/python3" ] || [ -x "$APPDIR/runtime/python310/bin/python" ] || die "bundled python3 missing"

if [ -f "$APPDIR/app/vendor/PUT_SDK_HERE.txt" ]; then
  if [ "$ALLOW_NO_SDK" -eq 0 ]; then
    die "PUT_SDK_HERE.txt still present; do not send this package"
  else
    note "sim-only AppDir (PUT_SDK_HERE.txt)"
  fi
else
  [ -f "$APPDIR/app/vendor/fafu_arm_sdk/fafu_robot_python/fafu_robot_controller.py" ] || die "SDK controller missing"
  so="$(ls "$APPDIR/app/vendor/fafu_arm_sdk/fafu_robot_python"/fafu_motor.cpython-310*-linux-gnu.so 2>/dev/null | head -n1 || true)"
  [ -n "$so" ] || die "Linux fafu_motor.so missing"
fi

[ -x "$APPDIR/usr/bin/gtk-probe" ] || die "gtk-probe missing; linuxdeploy GTK/WebKit hook absent"
webkit_so="$(find "$APPDIR/usr" -name 'libwebkit2gtk-4.0.so*' 2>/dev/null | head -n1 || true)"
jsc_so="$(find "$APPDIR/usr" -name 'libjavascriptcoregtk-4.0.so*' 2>/dev/null | head -n1 || true)"
[ -n "$webkit_so" ] || die "libwebkit2gtk-4.0.so missing from AppDir; do not send this package"
[ -n "$jsc_so" ] || die "libjavascriptcoregtk-4.0.so missing from AppDir; do not send this package"
note "webkit $webkit_so"
note "javascriptcore $jsc_so"

LIB_DIRS=()
for d in \
  "$APPDIR/usr/lib" \
  "$APPDIR/usr/lib/x86_64-linux-gnu" \
  "$APPDIR/usr/lib64" \
  "$APPDIR/runtime/python310/lib" \
  "$APPDIR/app/vendor/fafu_arm_sdk/fafu_robot_python"
do
  [ -d "$d" ] && LIB_DIRS+=("$d")
done
joined=""
if [ "${#LIB_DIRS[@]}" -gt 0 ]; then
  joined="$(IFS=:; echo "${LIB_DIRS[*]}")"
fi
GI_DIRS=()
for d in \
  "$APPDIR/usr/lib/girepository-1.0" \
  "$APPDIR/usr/lib/x86_64-linux-gnu/girepository-1.0"
do
  [ -d "$d" ] && GI_DIRS+=("$d")
done
gi_joined=""
if [ "${#GI_DIRS[@]}" -gt 0 ]; then
  gi_joined="$(IFS=:; echo "${GI_DIRS[*]}")"
fi

PY="$APPDIR/runtime/python310/bin/python3"
[ -x "$PY" ] || PY="$APPDIR/runtime/python310/bin/python"
if [ -x "$PY" ]; then
  env -u PYTHONPATH -u PYTHONHOME -u PYTHONSTARTUP \
    PYTHONNOUSERSITE=1 PIP_USER=0 STATION_PORTABLE=1 STATION_INSTALL_ROOT="$APPDIR" PYTHONPATH="$APPDIR/app" \
    LD_LIBRARY_PATH="${joined}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
    GI_TYPELIB_PATH="${gi_joined}${GI_TYPELIB_PATH:+:$GI_TYPELIB_PATH}" \
    "$PY" -c "import websockets, yaml, numpy, webview; print('deps-ok')" || die "bundled import failed"
  env -u PYTHONPATH -u PYTHONHOME -u PYTHONSTARTUP \
    PYTHONNOUSERSITE=1 PIP_USER=0 STATION_PORTABLE=1 STATION_INSTALL_ROOT="$APPDIR" PYTHONPATH="$APPDIR/app" \
    LD_LIBRARY_PATH="${joined}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
    GI_TYPELIB_PATH="${gi_joined}${GI_TYPELIB_PATH:+:$GI_TYPELIB_PATH}" \
    "$PY" -c "import gi; gi.require_version('Gtk','3.0'); gi.require_version('WebKit2','4.0'); from gi.repository import WebKit2; print('webkit-gi-ok', WebKit2)" \
    || die "bundled WebKit2 GI import failed; do not send this package"
  if [ "$ALLOW_NO_SDK" -eq 0 ] && [ ! -f "$APPDIR/app/vendor/PUT_SDK_HERE.txt" ]; then
    env -u PYTHONPATH -u PYTHONHOME -u PYTHONSTARTUP \
      PYTHONNOUSERSITE=1 PIP_USER=0 STATION_PORTABLE=1 STATION_INSTALL_ROOT="$APPDIR" PYTHONPATH="$APPDIR/app" \
      FAFU_ARM_SDK="$APPDIR/app/vendor/fafu_arm_sdk" \
      LD_LIBRARY_PATH="${joined}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
      "$PY" -c "import sys; sys.path.insert(0, r'$APPDIR/app'); from robot_station.adapters.fafu_arm import import_fafu_sdk; import_fafu_sdk(); import fafu_motor; print('fafu_motor-ok')" \
      || die "bundled fafu_motor import failed"
  fi
fi

if [ -f "$IMAGE" ]; then
  note "AppImage $(wc -c < "$IMAGE") bytes"
  man="$(dirname "$IMAGE")/BUILD_MANIFEST.json"
  if [ -f "$man" ]; then
    python3 - "$IMAGE" "$man" <<'PY' || die "BUILD_MANIFEST.json does not match the AppImage"
import hashlib, json, sys
from pathlib import Path
image, man_path = Path(sys.argv[1]), Path(sys.argv[2])
man = json.loads(man_path.read_text(encoding="utf-8"))
blob = image.read_bytes()
digest = hashlib.sha256(blob).hexdigest()
assert man.get("magic") == "FAFUAPP1", man.get("magic")
assert int(man.get("image_bytes") or 0) == len(blob)
assert man.get("image_sha256") == digest
assert man.get("webkit_present") is True, man
print("manifest-ok", man.get("version"), len(blob), man.get("webkit"))
PY
  else
    die "BUILD_MANIFEST.json missing next to AppImage"
  fi
else
  note "Sendable AppImage not present: $IMAGE"
fi

if [ "$fail" -ne 0 ]; then
  echo "verify failed"
  exit 1
fi
echo "verify ok"
