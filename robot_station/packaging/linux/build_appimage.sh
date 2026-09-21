#!/usr/bin/env bash
# Build dist/FAFUArmStation.AppDir and dist/FAFUArmStation-x86_64.AppImage
# Must run on Ubuntu 22.04 x86_64 (or the packaging/linux Docker image).
set -euo pipefail

PACK_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$PACK_DIR/../.." && pwd)"
OUT_DIR="${OUT_DIR:-$REPO/dist/FAFUArmStation.AppDir}"
CACHE="${CACHE:-$PACK_DIR/cache}"
PY_VER="${PY_VER:-3.10.15}"
STANDALONE_TAG="${STANDALONE_TAG:-20241016}"
# Pinned tools (never linuxdeploy/appimagetool "continuous" tags).
LINUXDEPLOY_URL="${LINUXDEPLOY_URL:-https://github.com/linuxdeploy/linuxdeploy/releases/download/1-alpha-20250213-2/linuxdeploy-x86_64.AppImage}"
APPIMAGETOOL_URL="${APPIMAGETOOL_URL:-https://github.com/AppImage/AppImageKit/releases/download/13/appimagetool-x86_64.AppImage}"
PLUGIN_GTK_URL="${PLUGIN_GTK_URL:-https://raw.githubusercontent.com/linuxdeploy/linuxdeploy-plugin-gtk/3b67a1d1c1b0c8268f57f2bce40fe2d33d409cea/linuxdeploy-plugin-gtk.sh}"
ALLOW_NO_SDK=0
SKIP_IMAGE=0
FORCE=0
# AppImages of the pack tools must not depend on host FUSE (Docker/CI).
export APPIMAGE_EXTRACT_AND_RUN=1

while [ $# -gt 0 ]; do
  case "$1" in
    --allow-no-sdk) ALLOW_NO_SDK=1 ;;
    --skip-image) SKIP_IMAGE=1 ;;
    --force) FORCE=1 ;;
    --out) shift; OUT_DIR="$1" ;;
    -h|--help)
      sed -n '2,8p' "$0"
      exit 0
      ;;
    *)
      echo "unknown arg: $1" >&2
      exit 1
      ;;
  esac
  shift
done

echo "Repo:    $REPO"
echo "AppDir:  $OUT_DIR"

fetch_url() {
  local url="$1" dest="$2"
  echo "Downloading $url"
  rm -f "$dest"
  mkdir -p "$(dirname "$dest")"
  if command -v curl >/dev/null 2>&1; then
    curl -fL --retry 5 --retry-delay 2 --connect-timeout 30 \
      -A "FAFUArmStation-pack/1.0" -o "$dest" "$url"
  else
    wget -U "FAFUArmStation-pack/1.0" -O "$dest" "$url"
  fi
  if [ ! -s "$dest" ]; then
    echo "download empty: $url" >&2
    exit 1
  fi
}

require_gzip() {
  local f="$1"
  local magic
  magic="$(od -An -tx1 -N2 "$f" | tr -d ' \n')"
  if [ "$magic" != "1f8b" ]; then
    echo "not gzip: $f (magic $magic)" >&2
    file "$f" >&2 || true
    head -c 240 "$f" >&2 || true
    echo >&2
    exit 1
  fi
}

find_sdk() {
  local c
  for c in \
    "${FAFU_ARM_SDK:-}" \
    "$REPO/vendor/fafu_arm_sdk" \
    "$REPO/../fafu_arm_sdk" \
    "$REPO/../fafu_arm_sdk-main" \
    "$HOME/fafu_arm_sdk"
  do
    [ -n "$c" ] || continue
    if [ -f "$c/fafu_robot_python/fafu_robot_controller.py" ]; then
      (cd "$c" && pwd)
      return 0
    fi
  done
  return 1
}

copy_tree() {
  local src="$1" dst="$2"
  mkdir -p "$dst"
  python3 - "$src" "$dst" <<'PY'
import shutil, sys
from pathlib import Path
src, dst = Path(sys.argv[1]), Path(sys.argv[2])
ignore = shutil.ignore_patterns(
    "__pycache__", ".git", ".venv", "venv", "dist", "build", "build_linux",
    "*.pyc", "*.pyo", ".github",
)
if dst.exists():
    shutil.rmtree(dst)
shutil.copytree(src, dst, ignore=ignore, symlinks=True)
PY
}

mkdir -p "$CACHE"
if [ "$FORCE" -eq 1 ] && [ -d "$OUT_DIR" ]; then
  rm -rf "$OUT_DIR"
fi
mkdir -p "$OUT_DIR/runtime" "$OUT_DIR/app" "$OUT_DIR/usr/bin" "$OUT_DIR/usr/share/fafu"

PYDIR="$OUT_DIR/runtime/python310"
PY="$PYDIR/bin/python3"

install_python() {
  if [ -x "$PY" ] && [ "$FORCE" -eq 0 ]; then
    echo "Using existing $PY"
    return 0
  fi
  local tar="cpython-${PY_VER}+${STANDALONE_TAG}-x86_64-unknown-linux-gnu-install_only.tar.gz"
  local url="https://github.com/astral-sh/python-build-standalone/releases/download/${STANDALONE_TAG}/${tar}"
  local zip="$CACHE/$tar"
  echo "Downloading relocatable CPython $PY_VER"
  if [ ! -f "$zip" ]; then
    fetch_url "$url" "$zip"
  fi
  require_gzip "$zip"
  rm -rf "$PYDIR"
  mkdir -p "$OUT_DIR/runtime"
  tar -xzf "$zip" -C "$OUT_DIR/runtime"
  if [ -d "$OUT_DIR/runtime/python" ] && [ ! -d "$PYDIR" ]; then
    mv "$OUT_DIR/runtime/python" "$PYDIR"
  fi
  chmod +x "$PY" 2>/dev/null || true
  if [ ! -x "$PY" ]; then
    echo "python3 missing after extract: $PYDIR" >&2
    ls -la "$OUT_DIR/runtime" >&2 || true
    ls -la "$PYDIR/bin" >&2 || true
    exit 1
  fi
}

install_python

export PYTHONNOUSERSITE=1
export PIP_USER=0
unset PYTHONHOME || true
export PYTHONPATH="$OUT_DIR/app"

echo "Installing pip packages into private runtime"
"$PY" -m ensurepip --upgrade >/dev/null 2>&1 || true
"$PY" -m pip install --no-warn-script-location --no-user -U pip setuptools wheel
core_req="$(mktemp)"
grep -vE '^[[:space:]]*(#|$)' "$REPO/requirements.txt" | grep -viE '^pyrealsense2' > "$core_req"
"$PY" -m pip install --no-warn-script-location --no-user -r "$core_req" numpy \
  'pybind11>=2.12,<3' 'pycairo>=1.20,<2' 'PyGObject>=3.42,<3.52'
rm -f "$core_req"
if ! "$PY" -m pip install --no-warn-script-location --no-user 'pyrealsense2>=2.54'; then
  echo "WARNING: pyrealsense2 not installed; camera stays mock until a cp310 Linux wheel is available"
fi

echo "Copying app"
copy_tree "$REPO/robot_station" "$OUT_DIR/app/robot_station"
copy_tree "$REPO/urdf" "$OUT_DIR/app/urdf"
cp -f "$REPO/run_station.py" "$OUT_DIR/app/"
cp -f "$REPO/station_desktop.py" "$OUT_DIR/app/"
cp -f "$REPO/station_client.conf" "$OUT_DIR/app/"
cp -f "$REPO/requirements.txt" "$OUT_DIR/app/"
copy_tree "$REPO/configs" "$OUT_DIR/app/configs"
rm -f "$OUT_DIR/app/configs/station.yaml"

mkdir -p "$OUT_DIR/app/docs"
customer=""
for md in "$REPO/docs/"*.md; do
  [ -f "$md" ] || continue
  cp -f "$md" "$OUT_DIR/app/docs/"
  if grep -q "station-customer-readme-linux" "$md" 2>/dev/null; then
    customer="$md"
  fi
done
if [ -z "$customer" ]; then
  for md in "$REPO/docs/"*.md; do
    if grep -q "station-customer-readme" "$md" 2>/dev/null; then
      customer="$md"
      break
    fi
  done
fi
if [ -z "$customer" ]; then
  echo "customer readme missing" >&2
  exit 1
fi
cp -f "$customer" "$OUT_DIR/app/README.md"

cp -f "$PACK_DIR/99-fafu-debug-board.rules" "$OUT_DIR/usr/share/fafu/99-fafu-debug-board.rules"

pkg_ver="1.6.1"
if grep -q '__version__' "$REPO/robot_station/__init__.py"; then
  pkg_ver="$(python3 -c "import pathlib,re; t=pathlib.Path('$REPO/robot_station/__init__.py').read_text(encoding='utf-8'); print(re.search(r'__version__\\s*=\\s*\"([^\"]+)\"', t).group(1))")"
fi
printf '%s\npacked %s\n' "$pkg_ver" "$(date -Iseconds)" > "$OUT_DIR/app/VERSION"

SDK=""
if SDK="$(find_sdk)"; then
  echo "Copying SDK $SDK"
  mkdir -p "$OUT_DIR/app/vendor"
  copy_tree "$SDK" "$OUT_DIR/app/vendor/fafu_arm_sdk"
  echo "Building fafu_motor.so for bundled Python"
  "$PY" -c "import sys,sysconfig,pybind11; print('py', sys.executable); print(sys.version); print('inc', sysconfig.get_path('include')); print('lib', sysconfig.get_config_var('LIBDIR')); print('pybind11', pybind11.get_cmake_dir())"
  export Python3_ROOT_DIR="$(cd "$(dirname "$PY")/.." && pwd)"
  export CMAKE_PREFIX_PATH="${Python3_ROOT_DIR}${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
  export LD_LIBRARY_PATH="${Python3_ROOT_DIR}/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  "$PY" -m pip install --no-warn-script-location --no-user 'pybind11>=2.12,<3'
  bash "$OUT_DIR/app/vendor/fafu_arm_sdk/fafu_robot_cpp/linux/build.sh" \
    --module-only --python "$PY" --jobs 2
  so="$(ls "$OUT_DIR/app/vendor/fafu_arm_sdk/fafu_robot_python"/fafu_motor.cpython-310*-linux-gnu.so 2>/dev/null | head -n1 || true)"
  if [ -z "$so" ]; then
    echo "fafu_motor.so was not produced" >&2
    exit 1
  fi
  if command -v patchelf >/dev/null 2>&1; then
    patchelf --set-rpath '$ORIGIN' --force-rpath "$so" || true
  fi
else
  echo "WARNING: fafu_arm_sdk not found"
  mkdir -p "$OUT_DIR/app/vendor"
  printf '%s\n' \
    "Place official fafu_arm_sdk here (must contain fafu_robot_python)." \
    "Live USB will not work until this folder is present and the package is rebuilt." \
    > "$OUT_DIR/app/vendor/PUT_SDK_HERE.txt"
  if [ "$ALLOW_NO_SDK" -eq 0 ]; then
    echo "Refusing customer AppImage without SDK (pass --allow-no-sdk for a sim-only AppDir)." >&2
    exit 1
  fi
fi

cp -f "$PACK_DIR/AppRun" "$OUT_DIR/AppRun"
chmod +x "$OUT_DIR/AppRun"
cp -f "$PACK_DIR/fafu-arm-station.desktop" "$OUT_DIR/fafu-arm-station.desktop"

ICON_SRC="$REPO/packaging/windows/FAFUArmStation.ico"
ICON_DST="$OUT_DIR/FAFUArmStation.png"
if command -v convert >/dev/null 2>&1 && [ -f "$ICON_SRC" ]; then
  convert "$ICON_SRC[0]" -resize 256x256 "$ICON_DST" || true
fi
if [ ! -f "$ICON_DST" ]; then
  "$PY" - "$ICON_DST" <<'PY'
import struct, zlib, sys
from pathlib import Path
path = Path(sys.argv[1])
w = h = 32
raw = bytearray()
for y in range(h):
    raw.append(0)
    for x in range(w):
        raw.extend(b"\x1a\x3a\x52")
def chunk(tag, data):
    crc = zlib.crc32(tag + data) & 0xFFFFFFFF
    return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)
ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + chunk(b"IDAT", zlib.compress(bytes(raw), 1)) + chunk(b"IEND", b""))
PY
fi

if ! command -v cc >/dev/null 2>&1 || ! pkg-config --exists gtk+-3.0 2>/dev/null; then
  echo "cc and gtk+-3.0 are required to bundle GTK into the AppImage" >&2
  exit 1
fi
cc -O2 "$PACK_DIR/gtk_probe.c" -o "$OUT_DIR/usr/bin/gtk-probe" $(pkg-config --cflags --libs gtk+-3.0)
if [ ! -x "$OUT_DIR/usr/bin/gtk-probe" ]; then
  echo "gtk-probe was not produced; refusing to wrap without a GTK hook" >&2
  exit 1
fi

if [ "$SKIP_IMAGE" -eq 1 ]; then
  echo "Skip AppImage wrap. AppDir at $OUT_DIR"
  exit 0
fi

if [ "$ALLOW_NO_SDK" -eq 0 ]; then
  if [ -f "$OUT_DIR/app/vendor/PUT_SDK_HERE.txt" ]; then
    echo "No SDK; not wrapping a customer AppImage" >&2
    exit 1
  fi
  if ! ls "$OUT_DIR/app/vendor/fafu_arm_sdk/fafu_robot_python"/fafu_motor.cpython-310*-linux-gnu.so >/dev/null 2>&1; then
    echo "Missing Linux fafu_motor.so; not wrapping a customer AppImage" >&2
    exit 1
  fi
fi

LINUXDEPLOY="$CACHE/linuxdeploy-x86_64.AppImage"
PLUGIN_GTK="$CACHE/linuxdeploy-plugin-gtk.sh"
APPIMAGETOOL="$CACHE/appimagetool-x86_64.AppImage"
if [ ! -f "$LINUXDEPLOY" ]; then
  fetch_url "$LINUXDEPLOY_URL" "$LINUXDEPLOY"
  chmod +x "$LINUXDEPLOY"
fi
if [ ! -f "$PLUGIN_GTK" ]; then
  fetch_url "$PLUGIN_GTK_URL" "$PLUGIN_GTK"
  chmod +x "$PLUGIN_GTK"
fi
if [ ! -f "$APPIMAGETOOL" ]; then
  fetch_url "$APPIMAGETOOL_URL" "$APPIMAGETOOL"
  chmod +x "$APPIMAGETOOL"
fi

export PATH="$CACHE:$PATH"
export LINUXDEPLOY_PLUGIN_GTK="$PLUGIN_GTK"
export DEPLOY_GTK_VERSION=3
export APPIMAGE_EXTRACT_AND_RUN=1
LD_ARGS=(--appdir "$OUT_DIR" --desktop-file "$OUT_DIR/fafu-arm-station.desktop" --icon-file "$ICON_DST")
LD_ARGS+=(--executable "$OUT_DIR/usr/bin/gtk-probe" --plugin gtk)
if [ -x "$PY" ]; then
  LD_ARGS+=(--executable "$PY")
fi
# linuxdeploy may rewrite AppRun; restore ours afterwards.
# A failed deploy must not wrap a GTK-less squashfs for customers.
if ! "$LINUXDEPLOY" "${LD_ARGS[@]}"; then
  echo "linuxdeploy failed; refusing to wrap a customer AppImage without bundled GTK" >&2
  exit 1
fi
cp -f "$PACK_DIR/AppRun" "$OUT_DIR/AppRun"
chmod +x "$OUT_DIR/AppRun"

IMAGE_OUT="${IMAGE_OUT:-$REPO/dist/FAFUArmStation-x86_64.AppImage}"
mkdir -p "$(dirname "$IMAGE_OUT")"
ARCH=x86_64 "$APPIMAGETOOL" "$OUT_DIR" "$IMAGE_OUT"
echo "Wrote $IMAGE_OUT"

python3 - "$IMAGE_OUT" "$OUT_DIR" "$pkg_ver" <<'PY'
import hashlib, json, os, platform, sys
from pathlib import Path
image, appdir, version = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
blob = image.read_bytes() if image.is_file() else b""
digest = hashlib.sha256(blob).hexdigest() if blob else ""
sdk_ctrl = appdir / "app/vendor/fafu_arm_sdk/fafu_robot_python/fafu_robot_controller.py"
motors = sorted(p.name for p in (appdir / "app/vendor/fafu_arm_sdk/fafu_robot_python").glob("fafu_motor.cpython-310*-linux-gnu.so")) if sdk_ctrl.is_file() else []
man = {
    "magic": "FAFUAPP1",
    "version": version,
    "image": image.name,
    "image_bytes": len(blob),
    "image_sha256": digest,
    "sdk_present": sdk_ctrl.is_file(),
    "fafu_motor": motors[0] if motors else "",
    "uname_m": platform.machine(),
    "glibc": os.popen("ldd --version 2>/dev/null | head -n1").read().strip(),
}
path = image.parent / "BUILD_MANIFEST.json"
path.write_text(json.dumps(man, indent=2) + "\n", encoding="utf-8")
print("Wrote", path)
PY

verify_args=(--appdir "$OUT_DIR" --image "$IMAGE_OUT")
if [ "$ALLOW_NO_SDK" -eq 1 ]; then
  verify_args+=(--allow-no-sdk)
fi
bash "$PACK_DIR/verify_appimage.sh" "${verify_args[@]}"
