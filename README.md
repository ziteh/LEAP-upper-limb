# LEAP Upper Limb

***LEAP-Up***, or "Lightweight Exoskeleton with Assistive Power, Upper Limb" is an upper limb powered exoskeleton based on the experience of develop [LEAP](https://github.com/ziteh/LEAP).  

- Firmware
  - [Master controller](/stm32/hi-level/): STM32Cube, STM32 HAL
  - [SIE joint code](/stm32/stm32_simple_foc/): PlatformIO, simpleFOC
  - [SFE joint code](/stm32/stm32_simple_foc/): PlatformIO, simopleFOC
  - [EFE joint code](/stm32/maxon_escon_pid_control/): PlatformIO, LibOpenCM3
- Hardware
  - PCB
    - [Master controller](/pcb/master-controller/)
    - [SIE/SFE joint](/pcb/motor-controller_simpleFOC/)
    - [EFE joint](/pcb/motor-controller_maxon/)
  - [Mechanical](/3d_model/)

# Part List

- MCU
  - STM32 NUCLEO-F446RE: Master controller and EFE joint
  - STM32 NUCLEO-L432KC: SIE and SFE joints
- Gear
  - SHD-25-100-2SH-SP: SIE joint
  - CSD-32-100-2A-GR: SFE joint
  - maxon GP 32 S: EFE joint
- Motor
  - T-Motor U8: SIE joint
  - T-Motor U10II: SFE joint
  - maxon EC-i30: EFE joint
- Motor Driver
  - ZONRI DRV8302: SIE and SFE joints
  - maxon ESCON 70/10: EFE joint
- Sensor
  - WACOH WEF-6A200-4-RCD: 6-Axis force sensor
  - ams-OSRAM AS5047P: Angle sensor/encoder
- Others
  - MAX490: RS-422 to UART

# Data Package (Legacy)

Data package used between ROS 2 and STM32.

Every data package contains several bytes, the first byte is "Header", the last byte is "EOT" symbol, other bytes are "Payload".

For example:
| Number | Byte           | Remark                     |
| :----: | -------------- | -------------------------- |
|   0    | Header         | The start of data package. |
|   1    | Payload byte-0 | The first byte of payload. |
|  ...   | Payload bytes  |
|  n-1   | Payload byte-n | The last byte of payload.  |
|   n    | EOT            | The end of data package.   |

> All the "X" in following table are meaning "Don't care".

## Header

### Structure

The MSB of header must be `1`.

| 7 (MSB) |   6   |   5   |   4   |   3   |   2   |   1   | 0 (LSB) |
| :-----: | :---: | :---: | :---: | :---: | :---: | :---: | :-----: |
|    1    |   0   |   0   |   0   |  T3   |  T2   |  T1   |   T0    |

- T*x*: Type of this package. 4-bit, 0~15.

###  Define

The following table show all the types of header:

| Header | Description                | Remark |
| :----: | -------------------------- | ------ |
| `0x80` | Motor basic control        | --     |
| `0x81` | Motor position control     | --     |
| `0x82` | *Reserve*                  | --     |
| `0x83` | *Reserve*                  | --     |
| `0x84` | *Reserve*                  | --     |
| `0x85` | *Reserve*                  | --     |
| `0x86` | Request force sensor value | --     |
| `0x87` | Request motor state        | --     |
| `0x88` | *Reserve*                  | --     |
| `0x89` | *Reserve*                  | --     |
| `0x8A` | *Reserve*                  | --     |
| `0x8B` | *Reserve*                  | --     |
| `0x8C` | Joint position state       | --     |
| `0x8D` | Force sensor value         | --     |
| `0x8E` | Motor state                | --     |
| `0x8F` | Do nothing                 | --     |

## Payload

### Structure

The MSB of payload must be `0`.

| 7 (MSB) |   6   |   5   |   4   |   3   |   2   |   1   | 0 (LSB) |
| :-----: | :---: | :---: | :---: | :---: | :---: | :---: | :-----: |
|    0    |  D6   |  D5   |  D4   |  D3   |  D2   |  D1   |   D0    |

- D*x*: Data. 7-bit, 0~127.

###  Define

#### Motor basic control

Number of bytes: 4

| Bytes | 7 (MSB) |   6   |   5   |   4   |   3   |   2   |   1   | 0 (LSB) | Remark            |
| :---: | :-----: | :---: | :---: | :---: | :---: | :---: | :---: | :-----: | ----------------- |
|  `0`  |    0    |   X   |  ID5  |  ID4  |  ID3  |  ID2  |  ID1  |   ID0   | Motor ID          |
|  `1`  |    0    |   X   |   X   |   X   |  D1   |  D0   |  E1   |   E0    | Direction, enable |
|  `2`  |    0    |   X   |  S5   |  S4   |  S3   |  S2   |  S1   |   S0    | Speed-0           |
|  `3`  |    0    |   X   |  S11  |  S10  |  S9   |  S8   |  S7   |   S6    | Speed-1           |

- ID*x*: The motor ID. 6-bit, 0~63.
- E*x*: Motor enable. 2-bit, 0~3.
  - `00(b)`：Disable.
  - `01(b)`：Enable.
  - `10(b)`：Toggle enable.
  - `11(b)`：Undefined.
- D*x*: Motor direction. 2-bit, 0~3.
  - `00(b)`：CW.
  - `01(b)`：CCW.
  - `10(b)`：Toggle direction.
  - `11(b)`：Undefined.
- S*x*: Speed. 12-bit, 0~4095.

#### Motor position control

Number of bytes: 3

| Bytes | 7 (MSB) |   6   |   5   |   4   |   3   |   2   |   1   | 0 (LSB) | Remark     |
| :---: | :-----: | :---: | :---: | :---: | :---: | :---: | :---: | :-----: | ---------- |
|  `0`  |    0    |   X   |  ID5  |  ID4  |  ID3  |  ID2  |  ID1  |   ID0   | Motor ID   |
|  `1`  |    0    |   X   |  P5   |  P4   |  P3   |  P2   |  P1   |   P0    | Position-0 |
|  `2`  |    0    |   X   |  P11  |  P10  |  P9   |  P8   |  P7   |   P6    | Position-1 |

- ID*x*: The motor ID. 6-bit, 0~63.
- P*x*: Position. 12-bit, 0~4095.

#### Request motor state

Number of bytes: 1

| Bytes | 7 (MSB) |   6   |   5   |   4   |   3   |   2   |   1   | 0 (LSB) | Remark   |
| :---: | :-----: | :---: | :---: | :---: | :---: | :---: | :---: | :-----: | -------- |
|  `0`  |    0    |   X   |  ID5  |  ID4  |  ID3  |  ID2  |  ID1  |   ID0   | Motor ID |

- ID*x*: The motor ID. 6-bit, 0~63.

#### Motor state

Number of bytes: 4

| Bytes | 7 (MSB) |   6   |   5   |   4   |   3   |   2   |   1   | 0 (LSB) | Remark                   |
| :---: | :-----: | :---: | :---: | :---: | :---: | :---: | :---: | :-----: | ------------------------ |
|  `0`  |    0    |   X   |  ID5  |  ID4  |  ID3  |  ID2  |  ID1  |   ID0   | Motor ID                 |
|  `1`  |    0    |   X   |   X   |   X   |   X   |  R0   |  D0   |   E0    | Direction, enable, state |
|  `2`  |    0    |   X   |  SP5  |  SP4  |  SP3  |  SP2  |  SP1  |   SP0   | Speed-0                  |
|  `3`  |    0    |   X   | SP11  | SP10  |  SP9  |  SP8  |  SP7  |   SP6   | Speed-1                  |

- ID*x*: The motor ID. 6-bit, 0~63.
- E*x*: Enable. 1-bit，0~1.
  - `0(b)`：Disable.
  - `1(b)`：Enable.
- D*x*: Direction. 1-bit, 0~1.
  - `0(b)`：CW.
  - `1(b)`：CCW.
- R*x*: Motor ready. 1-bit, 0~1.
  - `0(b)`：Fault.
  - `1(b)`：Ready.
- SP*x*: Motor speed. 12-bit, 0~4095.

#### Request force sensor value

Number of bytes: 1

| Bytes | 7 (MSB) |   6   |   5   |   4   |   3   |   2   |   1   | 0 (LSB) | Remark          |
| :---: | :-----: | :---: | :---: | :---: | :---: | :---: | :---: | :-----: | --------------- |
|  `0`  |    0    |   X   |   X   |   X   |   X   |  ID2  |  ID1  |   ID0   | Force sensor ID |

- ID*x*: Force sensor ID. 3-bit, 0~7.

#### Joint position state

Number of bytes: 5

| Bytes | 7 (MSB) |   6   |   5   |   4   |   3   |   2   |   1   | 0 (LSB) | Remark          |
| :---: | :-----: | :---: | :---: | :---: | :---: | :---: | :---: | :-----: | --------------- |
|  `0`  |    0    |   X   |  ID5  |  ID4  |  ID3  |  ID2  |  ID1  |   ID0   | Motor ID        |
|  `1`  |    0    |   X   |  N5   |  N4   |  N3   |  N2   |  N1   |   N0    | Now position-0  |
|  `2`  |    0    |   X   |  N11  |  N10  |  N9   |  N8   |  N7   |   N6    | Now position-1  |
|  `3`  |    0    |   X   |  G5   |  G4   |  G3   |  G2   |  G1   |   G0    | Goal position-0 |
|  `4`  |    0    |   X   |  G11  |  G10  |  G9   |  G8   |  G7   |   G6    | Goal position-1 |

- ID*x*: The motor ID. 6-bit, 0~63.
- N*x*: Now position. 12-bit, 0~4095.
- G*x*: Goal position. 12-bit, 0~4095.

#### Force sensor value

Number of bytes: 7

| Bytes | 7 (MSB) |   6   |   5   |   4   |   3   |   2   |   1   | 0 (LSB) | Remark          |
| :---: | :-----: | :---: | :---: | :---: | :---: | :---: | :---: | :-----: | --------------- |
|  `0`  |    0    |   X   |   X   |   X   |   X   |  ID2  |  ID1  |   ID0   | Force sensor ID |
|  `1`  |    0    |   X   |  VX5  |  VX4  |  VX3  |  VX2  |  VX1  |   VX0   | Value X-0       |
|  `2`  |    0    |   X   | VX11  | VX10  |  VX9  |  VX8  |  VX7  |   VX6   | Value X-1       |
|  `3`  |    0    |   X   |  VY5  |  VY4  |  VY3  |  VY2  |  VY1  |   VY0   | Value Y-0       |
|  `4`  |    0    |   X   | VY11  | VY10  |  VY9  |  VY8  |  VY7  |   VY6   | Value Y-1       |
|  `5`  |    0    |   X   |  VZ5  |  VZ4  |  VZ3  |  VZ2  |  VZ1  |   VZ0   | Value Z-0       |
|  `6`  |    0    |   X   | VZ11  | VZ10  |  VZ9  |  VZ8  |  VZ7  |   VZ6   | Value Z-1       |

- ID*x*: Force sensor ID. 3-bit, 0~7.
- VX*x*: Value X. 12-bit, 0~4095.
- VY*x*: Value Y. 12-bit, 0~4095.
- VZ*x*: Value Z. 12-bit, 0~4095.

## EOT

EOT (End of transmission) symbol = `0xFF`.

| 7 (MSB) |   6   |   5   |   4   |   3   |   2   |   1   | 0 (LSB) |
| :-----: | :---: | :---: | :---: | :---: | :---: | :---: | :-----: |
|    1    |   1   |   1   |   1   |   1   |   1   |   1   |    1    |

# Environment

- Ubuntu 20.04.3 'Focal Fossa' LTS (64-bit)
  - ROS 2 Foxy Fitzroy
- MATLAB R2020a
  - Robotics Toolbox for MATLAB `Ver. 10.4`
- STM32
  - PlatformIO IDE for VSCode `Ver. 3.4.4`
    - Core `Ver. 6.1.9`
    - Platform: ST STM32 `Ver. 15.6.0`
  - ~~Atollic TrueSTUDIO for STM32 `Ver. 9.3.0`~~
  - STM32Cube
    - MX: `Ver 6.7.0`
    - IDE: `Ver 1.12.0`
  - SimpleFOC: `Ver 2.2.3`
- Autodesk Inventor professional 2022

# Reference

- [SimpleFOC](https://simplefoc.com/)
- [stm32-examples](https://github.com/ziteh/stm32-examples)
  - [libopencm3](https://ziteh.github.io/series/%E7%B0%A1%E5%96%AE%E5%85%A5%E9%96%80-libopencm3-stm32-%E5%B5%8C%E5%85%A5%E5%BC%8F%E7%B3%BB%E7%B5%B1%E9%96%8B%E7%99%BC/)
- [libopencm3 Developer Documentation](https://libopencm3.org/docs/latest/html/index.html)
