# Robot Welding 6-Axis — Driver Setup & Usage Guide

This guide explains how to connect the EziMotion driver to WSL, configure the environment, and compile programs in this workspace.

## 1. Connect the Driver to WSL

1. Connect the driver to your laptop via USB.
2. Open Windows PowerShell as Administrator.
3. Check BusID:

```bash
usbipd list
```

4. Share and attach USB device to WSL:

```bash
usbipd bind --busid x-y
usbipd attach --wsl --busid x-y
```

Replace `x-y` with your actual BusID.

Note:
- Ensure no other application is using this USB device.
- If disconnected, run `usbipd attach --wsl --busid x-y` again.

## 2. Environment Setup in WSL

1. Install WSL (Ubuntu recommended).
2. Clone project:
   [Robot_Welding_6Axes](https://github.com/TranNguyenThanhDuy/Robot_Welding_6Axes)
3. Add library path in `~/.bashrc`:

```bash
export LD_LIBRARY_PATH=~/Robot_Welding_6_Axis/Robot_Welding_6_Axis/amd64/Examples/Include/lib:$LD_LIBRARY_PATH
```

4. Reload:

```bash
source ~/.bashrc
```

Important:
- Only use `amd64` folder in WSL.

## 3. Install Dependencies

From this workspace:

```bash
bash requirements.sh
```

This installs Qt5 + OpenCV + build tools.

## 4. Workspace Structure

- `lib/`: SDK headers + shared libraries (`libEziMOTIONPlusR*.so`, `MOTION_*.h`, `FAS_*.h`, ...)
- `source/`: motion/driver source
- `source/GUI/`: Qt GUI + USB camera (OpenCV)
- `Makefile`: build CLI and GUI
- `requirements.sh`: dependency installer

## 5. Build and Run (Recommended)

Build CLI:

```bash
make
```

Build GUI:

```bash
make gui AXIS=AXIS_x (thay x = số axis muốn sử dụng)
```

Run:

```bash
./test (nếu dùng CLI)
./gui (nếu dùng GUI)
```

Clean:

```bash
make clean
```

## 6. Axis Selection

Supported `AXIS` values:
- `AXIS_1`
- `AXIS_2`
- `AXIS_5` (default)
- `AXIS_6`

Example:

```bash
make clean
make AXIS=AXIS_6 gui
```

## 7. USB Camera in GUI

GUI camera source is `/dev/video0` (OpenCV backend), implemented in:
- `source/GUI/main_window.cpp`
- `source/GUI/usb_camera.cpp`

Quick checks in WSL:

```bash
ls -l /dev/video0
id
```

Your user should be in group `video`.

Share camera to WSL (if needed):
- If `/dev/video0` does not appear in WSL, attach USB camera from Windows using `usbipd`.
- Run PowerShell as Administrator:

```bash
usbipd list
usbipd bind --busid x-y
usbipd attach --wsl --busid x-y
```

- Replace `x-y` with your camera BusID.

## 8. Main Source Files

- `source/GUI/tc_6axis_gui.cpp`
- `source/GUI/main_window.h`
- `source/GUI/main_window.cpp`
- `source/GUI/usb_camera.h`
- `source/GUI/usb_camera.cpp`
- `source/GUI/gui_log_redirect.h`
- `source/GUI/gui_log_redirect.cpp`
- `source/axis_controller.h`
- `source/axis_controller.cpp`
- `source/DriverConnection.h`
- `source/DriverConnection.cpp`
