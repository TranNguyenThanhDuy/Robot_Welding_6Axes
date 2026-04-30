#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  pkg-config \
  python3 \
  python3-numpy \
  python3-matplotlib \
  python3-tk \
  qtbase5-dev \
  qtchooser \
  qt5-qmake \
  qtbase5-dev-tools \
  libopencv-dev \
  v4l-utils \
  usbutils \
