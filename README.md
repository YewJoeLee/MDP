# Autonomous Robot Car — STM32 Motion Firmware

My part of a team robot car built for NTU's Multidisciplinary Design Project (MDP, AY2026/27). A Raspberry Pi plans the route and sends short text commands over UART. This firmware runs on an **STM32F407** and executes each command precisely: drive an exact distance, turn to an exact angle, or approach an obstacle and stop at a set range.

Team repo (Android app, path planning, RPi code): https://github.com/YewJoeLee/MDP

## What the firmware does

- **Distance control.** It converts each command into wheel-encoder counts per motor, ramps down speed near the target and stops within a few cm (e.g. `FW080` → 80 cm forward).
- **Heading hold while driving.** It integrates the gyroscope to track heading and estimates sideways drift from encoder odometry, then corrects servo steering continuously. Small errors don't build up over multi-leg routes.
- **Gyro-based turns.** It turns to a target angle using integrated yaw rate, scales speed by the remaining angle and trims for overshoot. It uses different wheel speeds on the inside and outside of the turn to reduce slip.
- **Obstacle approach.** It uses median-filtered ultrasonic readings and IR distance sensors to drive up to an obstacle and stop at a target distance.
- **Fail-safes.** It reports timeouts, stalls, IMU faults and bad arguments back to the Pi as status codes instead of failing silently.
- **Gyro calibration.** It measures the gyro's zero-rate bias at startup and subtracts it from every reading.

## Hardware

| Part | Role |
|---|---|
| STM32F407 | Main motion controller |
| ICM-20948 IMU (I2C) | Gyroscope Z-axis for heading |
| 2 × DC motors + quadrature encoders | Drive and odometry (hardware timers in encoder mode) |
| Servo | Front-wheel steering |
| Ultrasonic + 2 × IR sensors (ADC) | Obstacle distance |
| OLED | On-board debug display |
| Raspberry Pi (UART3) | Sends commands, receives `DONE` / `ERR` replies |

## Software structure

The firmware runs on **FreeRTOS (CMSIS-RTOS v2)** with two tasks:

- **`StartDefaultTask`** reads commands from an interrupt-filled UART queue, parses them, runs the motion and replies.
- **`show`** is a low-priority task that refreshes the OLED with live sensor and status data.

Key functions in `firmware/main.c`:

| Function | Purpose |
|---|---|
| `Drive_Distance` | Encoder-based straight drive with gyro + odometry heading hold |
| `Car_Turn` | Gyro-integrated turn to a target angle |
| `Car_ApproachUltrasonic` | Drive toward an obstacle and stop at a target range |
| `Gyro_CalibrateZBias` | Measures and stores the gyro's zero-rate bias |
| `US_Median_mm`, `IR_Read_mm` | Filtered distance sensor readings |
| `HAL_UART_RxCpltCallback` | Byte-by-byte UART receive into a command queue |

## Command protocol (RPi → STM32)

| Command | Meaning |
|---|---|
| `FWxxx` / `BWxxx` | Drive forward / backward xxx cm |
| `LTxxx` / `RTxxx` | Turn left / right xxx degrees |
| `US`, `IR`, `GY`, `YW` | Query a sensor reading |

Every command gets a one-line reply such as `DONE FW080 A:… B:… HE:… OK` or `ERR BADARG [...]`. The Pi always knows whether a move succeeded.

## Notes

This is the application source (`main.c`) from an STM32CubeIDE project. Generated HAL/driver files and the OLED library are not included.
