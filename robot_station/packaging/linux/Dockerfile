# Ubuntu 22.04 builder for FAFUArmStation.AppImage
# Optional local path. Primary Windows-team pack path is GitHub Actions
# (.github/workflows/packaging-smoke.yml job linux-appimage).
# Build from the FAFU_station repo root (sibling fafu_arm_sdk + robot_station):
#   docker build -f robot_station/packaging/linux/Dockerfile -t fafu-appimage .
#   docker run --rm -v "${PWD}:/src" fafu-appimage
# Entrypoint defaults FAFU_ARM_SDK=/src/fafu_arm_sdk and tees a log to
# robot_station/dist/appimage-build.log
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
# Pack tools are AppImages; never require host FUSE inside this container.
ENV APPIMAGE_EXTRACT_AND_RUN=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    desktop-file-utils \
    file \
    g++ \
    git \
    gir1.2-gtk-3.0 \
    gir1.2-webkit2-4.0 \
    libfuse2 \
    libgirepository1.0-dev \
    libgtk-3-dev \
    libssl-dev \
    libwebkit2gtk-4.0-dev \
    libcairo2-dev \
    pkg-config \
    patchelf \
    python3 \
    python3-dev \
    python3-pip \
    wget \
    xz-utils \
    zlib1g-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY robot_station/packaging/linux/docker-entrypoint.sh /usr/local/bin/fafu-appimage-build
RUN chmod +x /usr/local/bin/fafu-appimage-build
ENTRYPOINT ["/usr/local/bin/fafu-appimage-build"]
