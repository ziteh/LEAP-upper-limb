# LEAP Upper Limb

***LEAP-Up***, or "Lightweight Exoskeleton with Assistive Power, Upper Limb" is my personal research. It's an upper limb powered exoskeleton based on the experience of develop [LEAP](https://github.com/ziteh/LEAP).  

It is still at an early stage of development.

<br/>

***LEAP-Up***，或稱為「輔助性動力輕型外骨骼-上肢部分」是我的個人研究。它是以開發 [LEAP](https://github.com/ziteh/LEAP) 的經驗為基礎，製作的上肢動力外骨骼。  

目前還在初期開發階段。

---

# Data Package

Data package used between ROS 2 and STM32.

Ever data package contains several bytes, the first byte is 'header', other bytes are 'payload'.

## Structure

### Header

The MSB of header must be `1`.

| 7 (MSB) |   6   |   5   |   4   |   3   |   2   |   1   | 0 (LSB) |
| :-----: | :---: | :---: | :---: | :---: | :---: | :---: | :-----: |
|    1    |   X   |   X   |   X   |  T3   |  T2   |  T1   |   T0    |

- T*x*: Type of this package. 4-bit, 0~15.
- X: Don't care.

### Payload

The MSB of payload must be `0`.

| 7 (MSB) |   6   |   5   |   4   |   3   |   2   |   1   | 0 (LSB) |
| :-----: | :---: | :---: | :---: | :---: | :---: | :---: | :-----: |
|    0    |  D6   |  D5   |  D4   |  D3   |  D2   |  D1   |   D0    |

- D*x*: Data. 7-bit, 0~127.

##  Defined 

### Header

The following table show all the types of header:

| Header | Description            | Remark |
| :----: | ---------------------- | ------ |
| `0x80` | Motor basic control    | --     |
| `0x81` | Motor position control | --     |
| `0x82` | *Reserve*              | --     |
| `0x83` | *Reserve*              | --     |
| `0x84` | *Reserve*              | --     |
| `0x85` | *Reserve*              | --     |
| `0x86` | *Reserve*              | --     |
| `0x87` | Request motor state    | --     |
| `0x88` | *Reserve*              | --     |
| `0x89` | *Reserve*              | --     |
| `0x8A` | *Reserve*              | --     |
| `0x8B` | *Reserve*              | --     |
| `0x8C` | *Reserve*              | --     |
| `0x8D` | *Reserve*              | --     |
| `0x8E` | Motor state            | --     |
| `0x8F` | Do nothing             | --     |

### Payload

#### Motor basic control 

Number of bytes: 2.

| Bytes | 7 (MSB) |   6   |   5   |   4   |   3   |   2   |   1   | 0 (LSB) | Remark            |
| :---: | :-----: | :---: | :---: | :---: | :---: | :---: | :---: | :-----: | ----------------- |
|  `0`  |    0    |   X   |  ID5  |  ID4  |  ID3  |  ID2  |  ID1  |   ID0   | Motor ID          |
|  `1`  |    0    |   X   |   X   |   X   |  D1   |  D0   |  E1   |   E0    | Direction, enable |

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
- X: Don't care.

#### Motor position control 

Number of bytes: 3.

| Bytes | 7 (MSB) |   6   |   5   |   4   |   3   |   2   |   1   | 0 (LSB) | Remark     |
| :---: | :-----: | :---: | :---: | :---: | :---: | :---: | :---: | :-----: | ---------- |
|  `0`  |    0    |   X   |  ID5  |  ID4  |  ID3  |  ID2  |  ID1  |   ID0   | Motor ID   |
|  `1`  |    0    |   X   |  P5   |  P4   |  P3   |  P2   |  P1   |   P0    | Position-0 |
|  `2`  |    0    |   X   |  P11  |  P10  |  P9   |  P8   |  P7   |   P6    | Position-1 |

- ID*x*: The motor ID. 6-bit, 0~63.
- P*x*: Position. 12-bit, 0~4095.
- X: Don't care.

#### Request motor state

Number of bytes: 1.

| Bytes | 7 (MSB) |   6   |   5   |   4   |   3   |   2   |   1   | 0 (LSB) | Remark   |
| :---: | :-----: | :---: | :---: | :---: | :---: | :---: | :---: | :-----: | -------- |
|  `0`  |    0    |   X   |  ID5  |  ID4  |  ID3  |  ID2  |  ID1  |   ID0   | Motor ID |

- ID*x*: The motor ID. 6-bit, 0~63.

#### Motor state

Number of bytes: 4.

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
- X: Don't care.

# Environment
- Ubuntu 20.04.3 'Focal Fossa' LTS (64-bit)
  - ROS 2 Foxy Fitzroy
- MATLAB R2020a
  - Robotics Toolbox for MATLAB `Ver. 10.4`
- STM32
  - PlatformIO IDE for VSCode `Ver. 2.4.0`
    - Core `Ver. 5.2.3`
    - Platform: ST STM32 `Ver. 15.0.0`
  - ~~Atollic TrueSTUDIO for STM32 `Ver. 9.3.0`~~
