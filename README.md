# Robot Welding 6-Axis — Driver Setup & Usage Guide

This guide explains how to connect the EziMotion driver to WSL, configure the environment, and compile example programs included in this project. 
---

## 🧩 1. Connect the Driver to WSL

### Steps:
1. Connect the **driver** to your **laptop** via USB.  
2. Open **Windows PowerShell** (Run as Administrator).  
3. Check which port the driver is connected to:
   ```bash
   usbipd list
   ```
4. Share and attach the USB port to WSL:
   ```bash
   usbipd bind --busid x-y          # Replace x-y with the BusID found in step 3
   usbipd attach --wsl --busid x-y  # Replace x-y with the BusID found in step 3
   ```
⚠️ **Note**: Ensure that no other applications are currently using this USB port.

## ⚙️ 2. Environment Setup in WSL
### Steps:
1. Install WSL (Ubuntu or other preferred distribution).
2. [Clone](https://github.com/TranNguyenThanhDuy/Robot_Welding_6Axes) the project
3. Add the driver library path to your environment:
   ```
   vim ~/.bashrc
   ```
   Append the following line at the end of the file:
   ```
   export LD_LIBRARY_PATH=~/Robot_Welding_6_Axis/Robot_Welding_6_Axis/amd64/Examples/Include/
   ```
4. Reload the configuration:
   ```
   source ~/.bashrc
   ```

## 💻 3. Compile and Run Programs
You can compile and execute example files or create new ones using the provided driver library.
### Steps:
1. Place your new source file inside:
   ```swift
   Robot_Welding_6_Axis/Robot_Welding_6_Axis/amd64/Examples/Include
   ```
2. Compile your program:
   ```bash
   g++ -o test ConnectionExam.cpp -L. -lEziMOTIONPlusR
   ```
   * `test`: Output executable name (you can rename it).
   * `ConnectionExam.cpp`: Source file to compile (replace with your filename).
3. Run the program:
   ```bash
   ./test
   ```
   Replace `test` with your executable name if different.

⚠️ **Note**: ONLY USE `amd64 folder` IN WSL.
