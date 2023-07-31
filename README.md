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
