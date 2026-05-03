# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

2026 RoboMaster 厦门大学RCS战队空中机器人(无人机)电控系统. Target: 达妙MC-02开发板 (STM32H723VGT6, Cortex-M7, ARMv7E-M with FPU). Based on 辽宁科技大学COD H7 BSP, restructured following 湖南大学跃鹿战队 software architecture.

## Build

```bash
# Configure (out-of-source, from repo root)
cmake -B build -G "Unix Makefiles" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=<path-to-arm-none-eabi-toolchain>.cmake

# Build
cmake --build build -j$(nproc)

# Outputs: build/COD_H7_Template.elf, build/COD_H7_Template.hex
```

Cross-compiler: `arm-none-eabi-gcc` (Arm Compiler V6.19 compatible). Also compilable via Keil V5.38 or VS2019+VisualGDB. CubeMX project file: `COD_H7_Template.ioc`.

## Architecture

```
main.c (HAL init → RobotInit → MCU_Init → MX_FREERTOS_Init → osKernelStart)
  └── FreeRTOS (CMSIS-OS v2 wrapper, static allocation, 6 tasks)
        ├── INS_Task        (osPriorityHigh, 1kHz) — BMI088 IMU read → EKF quaternion → Euler angles
        ├── CAN_Task        (osPriorityNormal, 2kHz) — DM motor control + DJI motor control
        ├── Control_Task    (osPriorityAboveNormal) — high-level control logic
        ├── Robot_Task      (osPriorityNormal) — GimbalTask + ShootTask dispatch
        ├── Detect_Task     (osPriorityBelowNormal) — detection/vision processing
        └── Referee_Task    (osPriorityNormal) — referee system communication
```

### Layer stack

| Layer | Path | Role |
|-------|------|------|
| BSP | `USER/BSP/` | HAL peripheral wrappers: CAN/FDCAN, UART, SPI, PWM, ADC, GPIO, DWT (cycle-accurate timing) |
| Components/Device | `USER/Components/Device/` | Device drivers: DJI smart motors, DM (达妙) motors, BMI088 IMU, MiniPC, VT03, WS2812B, Remote Control, Referee System |
| Components/Algorithm | `USER/Components/Algorithm/` | EKF quaternion attitude, Kalman filter, 1p/2p low-pass filters, Ramp generator, RLS, CRC, general controller |
| Components/Controller | `USER/Components/Controller/` | PID controller (position/velocity, cascaded loops, integral clamping, anti-windup) |
| Components/Message | `USER/Components/Message/` | Pub-sub message center (topic-based, FIFO queues per subscriber), referee protocol, referee UI |
| Application | `USER/Application/Task/` | FreeRTOS task implementations + `gimbal/` + `shoot/` subsystems |

### Motor ecosystem

Two motor families unified under `Motor_Control_Setting_s` + `Motor_Controller_s` (`USER/Components/Device/Inc/motor_def.h`):
- **DJI smart motors**: GM6020, M3508, M2006 — CAN-based, identified by CAN ID. Managed via `DJIMotorInstance` (`dji_motor.h`).
- **DM (达妙) motors**: DM3507, DM4310, DM8009, DM6006 — managed via `dm_motor.c/h`.

Each motor supports cascaded PID (angle → speed → current), with feedforward options and configurable feedback sources (motor encoder or external like IMU).

### Pub-sub message center

`message_center.h` — lightweight intra-application communication. Publishers register topics by name; subscribers receive via FIFO queues. Used to decouple CMD (commander), gimbal, chassis, and shoot applications. Defined in `robot_def.h`: `Chassis_Ctrl_Cmd_s`, `Gimbal_Ctrl_Cmd_s`, `Shoot_Ctrl_Cmd_s`, and corresponding upload feedback structs.

### Board configuration

In `robot_def.h`, exactly one of these must be defined:
- `ONE_BOARD` — single MCU controls entire robot (current config)
- `CHASSIS_BOARD` — chassis-only MCU
- `GIMBAL_BOARD` — gimbal-only MCU

Robot physical parameters (wheelbase, gimbal offsets, encoder zero positions, etc.) are also defined in `robot_def.h`.

### Key initialization sequence

1. `main()`: HAL init, clock config (PLL from HSE), MPU config (512KB non-cacheable region at 0x24000000), I/D-Cache enable
2. `RobotInit()`: `GimbalInit()` + `ShootInit()` (for ONE_BOARD)
3. `MCU_Init()`: DWT → PWM → GPIO → CAN → UART → ADC → USB VCP → BMI088
4. `MX_FREERTOS_Init()`: static thread creation, then `osKernelStart()`

### Peripherals in use

- FDCAN1/2/3: DJI motors, DM motors, remote control receiver
- SPI2: BMI088 IMU
- UART5/7, USART1/2/3/10: VT03 vision, referee system, MiniPC, debug (Vofa firewater protocol)
- USB VCP: vision data to host
- ADC1/3: analog inputs
- TIM1/3: PWM output (friction wheels, heater)

## Coding conventions

- All user code lives under `USER/` (CubeMX generates into `Core/`, `Drivers/`, `Middlewares/`)
- BSP functions are prefixed `BSP_` (e.g., `BSP_PWM_Init`)
- `robot_def.h` MUST be checked before flashing — wrong parameters cause physical robot errors
- `#pragma pack(1)` for structs that go over CAN/UART; `#pragma pack()` restores alignment
- Static memory allocation preferred (FreeRTOS `osThreadStaticDef`, fixed buffers)
- Float constants use `f` suffix (e.g., `9.7887f`)
