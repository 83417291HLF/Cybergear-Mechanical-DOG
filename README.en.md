# Cybergear Mechanical DOG

An 8-DOF parallel-leg quadruped project for the author's 2026 ROBOCON preparation. The controller is a RoboMaster Type A board (STM32F427IIHx), driving eight Xiaomi CyberGear motors across two CAN buses.

See the [Chinese README](README.md) for hardware mappings, remote channels and parameters, the [code walkthrough](docs/CODE_WALKTHROUGH.md) for implementation details and known defects, and the [build guide](docs/BUILD_AND_BRINGUP.md) for bring-up steps.

## Current software

- Bare-metal loop targeting 10 ms, using STM32 HAL.
- SBUS remote input through USART1 DMA/IDLE.
- Diagonal gait with cycloidal swing trajectories, differential turning and stepping in place.
- MPU6500/IST8310 attitude estimation with pitch/roll leg-height compensation.
- Geometric inverse kinematics, joint spring/damping compensation and motor-side PD.
- CAN1 motor IDs 1–4; CAN2 IDs 5–8.

Experimental VMC and jump files are present but are not included in the current Keil build target. The active code does not start FreeRTOS.

## Build

Open [sizujixgou1123.uvprojx](software/sizujixgou1123/MDK-ARM/sizujixgou1123.uvprojx). The project records ARMCC 5.05 update 1 and Keil.STM32F4xx_DFP.2.17.0. HAL and CMSIS sources are included.

This is a development snapshot. Static review identified a DMA buffer length mismatch, incomplete remote-loss handling, feedback angle scale inconsistency and other issues documented in the walkthrough. This documentation update did not compile, flash or test the robot, and does not establish competition compliance.

Build products, local IDE state, duplicate archives and third-party desktop tool bundles are excluded from version control; local copies remain untouched. Existing third-party license files and attribution are retained. No new license has been assigned to the author's code.
