# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Industrial vision inspection system (工业视觉检测系统) for deep learning-based defect detection on PCB boards, metal parts, and packaging materials. The project is currently in the **design/planning phase** — only the design document exists (`工业视觉检测系统方案.md`), no source code has been implemented yet.

## Planned Architecture

Two-tier embedded system:

- **Upper computer (Linux PC, C++17)**: Image acquisition (camera SDK), vision algorithm (YOLOv8 + TensorRT), Qt5 GUI, SQLite database, serial/Modbus communication
- **Lower computer (STM32, C)**: Conveyor belt control, light source PWM, rejection mechanism (pneumatic cylinders), photoelectric sensors

Communication between tiers uses UART with a custom frame protocol (0xAA 0x55 header + CMD + DATA + CRC8).

## Planned Build Commands

```bash
# PC upper computer (C++ / CMake)
mkdir build && cd build
cmake ..
make

# STM32 firmware
# Built via STM32CubeIDE or arm-none-eabi-gcc toolchain
```

## Planned Technology Stack

| Component | Technology |
|-----------|-----------|
| Language (PC) | C++17 |
| Language (STM32) | C |
| Build system | CMake 3.16+ |
| GUI | Qt5 |
| Image processing | OpenCV 4.x |
| Inference | TensorRT + CUDA |
| Model training | YOLOv8 (ultralytics, Python) |
| Serial comm | libserial / Boost.Asio |
| Logging | spdlog |
| Database | SQLite |
| RTOS (optional) | FreeRTOS |

## Planned Source Layout

```
vision_inspection/          # PC upper computer
├── camera/                 # Camera acquisition (Hikrobot MVS SDK)
├── algorithm/              # Detection (YOLOv8/TensorRT), preprocessing
├── communication/          # Serial port, custom protocol
├── ui/                     # Qt5 interface
├── database/               # SQLite operations
├── utils/                  # Logging (spdlog)
└── CMakeLists.txt

stm32_controller/           # STM32 lower computer
├── Core/Src/               # main.c, motor_control, sensor, actuator, communication
├── Drivers/
└── Middlewares/FreeRTOS/
```

## Development Environment

- OS: Ubuntu 20.04/22.04
- CPU: Intel i5+, NVIDIA GPU with CUDA support
- Fallback: OpenCV DNN module for CPU inference if no NVIDIA GPU available
- Portable to Jetson (Orin Nano) for edge deployment
