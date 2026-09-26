#!/usr/bin/env bash
# ACCEPTANCE B helper: real x86 Ubuntu desktop (Live USB is OK).
# Refuses WSL / headless SSH. Does not replace the 仿真臂 slider check.
set -euo pipefail

die() { echo "FAIL: $*" >&2; exit 1; }
skip() { echo "NOT B: $*" >&2; exit 2; }
note() { echo "handtest: $*"; }

usage() {
  cat <<'EOF'
Usage: bash packaging/linux/handtest_open_window.sh /path/to/FAFUArmStation-x86_64.AppImage

Run on Ubuntu 22.04 or 24.04 x86_64 desktop. A Live USB session is OK.
WSL is rejected and must not be treated as customer click-acceptance.

The script checks bundled WebKit, launches the AppImage, and looks for the
independent GTK window. After the window appears, switch to 仿真臂, drag
sliders, and send a pose. Close the window when finished.
EOF
}

IMAGE="${1:-}"
if [ -z "$IMAGE" ] || [ "$IMAGE" = "-h" ] || [ "$IMAGE" = "--help" ]; then
  usage
  if [ -z "$IMAGE" ]; then
    exit 1
  fi
  exit 0
fi

if grep -qiE 'microsoft|wsl' /proc/version 2>/dev/null \
  || [ -d /run/WSL ] \
  || [ -n "${WSL_DISTRO_NAME:-}" ] \
  || [ -n "${WSL_INTEROP:-}" ]; then
  skip "WSL is not ACCEPTANCE B. Use a physical Ubuntu desktop or Live USB."
fi

arch="$(uname -m)"
[ "$arch" = "x86_64" ] || skip "need x86_64, got $arch"

display="${DISPLAY:-}${WAYLAND_DISPLAY:-}"
[ -n "$display" ] || skip "no DISPLAY/WAYLAND_DISPLAY; boot a desktop session (Live USB is OK)"

[ -f "$IMAGE" ] || die "AppImage not found: $IMAGE"
IMAGE="$(cd "$(dirname "$IMAGE")" && pwd)/$(basename "$IMAGE")"
chmod +x "$IMAGE"
note "image $IMAGE ($(wc -c < "$IMAGE") bytes)"

WORKDIR="$(mktemp -d /tmp/fafu-handtest.XXXXXX)"
APPDIR=""
PID=""
cleanup() {
  if [ -n "${PID:-}" ] && kill -0 "$PID" 2>/dev/null; then
    note "leaving station pid $PID running so you can finish 仿真臂"
  fi
}
trap cleanup EXIT

cd "$WORKDIR"
note "extracting squashfs (WebKit presence)"
if ! "$IMAGE" --appimage-extract >/dev/null; then
  die "could not extract AppImage (try: sudo apt install libfuse2, or this file is corrupt)"
fi
APPDIR="$WORKDIR/squashfs-root"
[ -x "$APPDIR/AppRun" ] || die "extracted AppDir has no AppRun"

webkit_so="$(find "$APPDIR/usr" -name 'libwebkit2gtk-4.0.so*' 2>/dev/null | head -n1 || true)"
jsc_so="$(find "$APPDIR/usr" -name 'libjavascriptcoregtk-4.0.so*' 2>/dev/null | head -n1 || true)"
[ -n "$webkit_so" ] || die "libwebkit2gtk-4.0.so missing; do not send this package"
[ -n "$jsc_so" ] || die "libjavascriptcoregtk-4.0.so missing; do not send this package"
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
[ -x "$PY" ] || die "bundled python3 missing"

man="$(dirname "$IMAGE")/BUILD_MANIFEST.json"
if [ -f "$man" ]; then
  "$PY" - "$IMAGE" "$man" <<'PY' || die "BUILD_MANIFEST.json does not match the AppImage"
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
  note "no BUILD_MANIFEST.json next to AppImage; continuing with squashfs checks"
fi

env -u PYTHONPATH -u PYTHONHOME -u PYTHONSTARTUP \
  PYTHONNOUSERSITE=1 PIP_USER=0 STATION_PORTABLE=1 STATION_INSTALL_ROOT="$APPDIR" PYTHONPATH="$APPDIR/app" \
  LD_LIBRARY_PATH="${joined}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  GI_TYPELIB_PATH="${gi_joined}${GI_TYPELIB_PATH:+:$GI_TYPELIB_PATH}" \
  "$PY" -c "import gi; gi.require_version('Gtk','3.0'); gi.require_version('WebKit2','4.0'); from gi.repository import WebKit2; print('webkit-gi-ok', WebKit2)" \
  || die "bundled WebKit2 GI import failed; do not send this package"

CACHE="${XDG_CACHE_HOME:-$HOME/.cache}"
mkdir -p "$CACHE"
LOG="$CACHE/fafu-station-app.log"
: > "$LOG"
HOST_LOG="$CACHE/fafu-station-host.log"
: > "$HOST_LOG"

note "launching independent GTK window"
LAUNCH_LOG="$WORKDIR/launch.log"
set +e
"$IMAGE" --appimage-extract-and-run >"$LAUNCH_LOG" 2>&1 &
PID=$!
set -e
note "pid $PID"

deadline=$((SECONDS + 45))
window_ok=0
webkit_fail=0
while kill -0 "$PID" 2>/dev/null && [ "$SECONDS" -lt "$deadline" ]; do
  if grep -qE '无法打开独立控制窗口|libwebkit2gtk-4\.0\.so|libjavascriptcoregtk-4\.0\.so' "$LOG" "$LAUNCH_LOG" 2>/dev/null; then
    webkit_fail=1
    break
  fi
  if grep -qE 'window http|control page loaded' "$LOG" 2>/dev/null; then
    window_ok=1
    break
  fi
  if command -v wmctrl >/dev/null 2>&1 && wmctrl -l 2>/dev/null | grep -q 'FAFU'; then
    window_ok=1
    break
  fi
  sleep 1
done

if [ "$webkit_fail" -eq 1 ]; then
  echo "----- $LOG -----" >&2
  tail -n 40 "$LOG" >&2 || true
  echo "----- launch -----" >&2
  tail -n 40 "$LAUNCH_LOG" >&2 || true
  kill "$PID" 2>/dev/null || true
  PID=""
  die "independent window failed (WebKit). Official AppImage must not ask the customer to apt-install WebKit. Do not send this package."
fi

if [ "$window_ok" -ne 1 ]; then
  echo "----- $LOG -----" >&2
  tail -n 40 "$LOG" >&2 || true
  echo "----- launch -----" >&2
  tail -n 40 "$LAUNCH_LOG" >&2 || true
  if kill -0 "$PID" 2>/dev/null; then
    note "process still running but window was not confirmed in 45s"
  else
    wait "$PID" || true
    PID=""
    die "AppImage exited before an independent window appeared"
  fi
  kill "$PID" 2>/dev/null || true
  PID=""
  die "could not confirm independent GTK window"
fi

cat <<EOF
handtest: independent window started (pid $PID)

Remaining human checks (ACCEPTANCE B — required before sending to customers):
  [ ] Window is the GTK app, not the system browser
  [ ] No prompt to apt-install WebKitGTK / gir1.2-webkit2-4.0
  [ ] Browser can open http://127.0.0.1:9400/
  [ ] Switch to 仿真臂, drag sliders, send pose; 3D moves
  [ ] Same file opens a window on Ubuntu 24.04 if you have it

Close the window when finished. Do not send this file until those boxes are ticked.
EOF
exit 0
