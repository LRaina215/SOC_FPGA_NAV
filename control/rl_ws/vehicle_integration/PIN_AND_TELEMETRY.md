# Sensor integration

This isolated SRAM diagnostic top contains the original complete full_center_v4 network and adds BMI088 plus both MT6701 SSI channels. All12 actuator outputs remain constant zero. No motor calibration or servo pulse command exists in this top.

Left MT6701: P11 35/37/39 (DO/CLK/CSN) = C13/A13/A15.
Right MT6701: P11 36/38/40 (DO/CLK/CSN) = B13/A14/A16.
The new right assignment was given by the user and checked against the local CB schematic sheet10 (B2_R17_N/B2_R15_N/B2_R20_N) and core schematic sheet1 (B13/A14/A16).

SERVO1 LeftBack, SERVO2 LeftFront, SERVO3 RightFront, SERVO4 RightBack, per user. Physical zero, direction, travel and pulse conversion remain unmeasured; this correspondence alone does not constitute servo calibration.

UART request03 has empty payload. Reply83 has64 frozen bytes, LE:

| Offset | Bytes | Meaning |
|---|---:|---|
| 0 | 4 | current50MHz ticks |
| 4 / 8 | 4each | IMU publication count / last publication ticks |
| 12 / 18 | 6each | XYZ signed16 accel / gyro |
| 24 / 28 | 4each | Left encoder CRC-valid count / last ticks |
| 32 / 34 / 35 | 2 / 1 / 1 | Left angle / status / error |
| 36 | 1 | bits0–3: IMU ready,error; left ready,error |
| 37 | 3 | Left full raw frame |
| 40 / 44 | 4each | Right encoder CRC-valid count / last ticks |
| 48 / 50 / 51 | 2 / 1 / 1 | Right angle / status / error |
| 52 / 53 | 1 / 3 | Right ready/error flags / full raw frame |
| 56 / 57 / 58 | 1each | Last BMI operation / expected / actual byte |
| 59 / 60 | 1each | Last accel/gyro chip IDs |
| 61 | 2 | reserved |
| 63 | 1 | live raw input levels: MISO, LeftDO, RightDO bits0–2 |

Sensor reads retain the existing100Hz BMI configuration for initial physical diagnosis. This configuration does not satisfy a requirement for a fresh sample at every125Hz policy tick. Timestamped400Hz reads and data-ready checks must be validated before closed-loop operation. SSI count includes CRC-valid samples even when magnetic status is bad; readiness is separate. A stuck-low DO can form a valid all-zero frame: real angle variation must also be measured.

The CPU runtime directory provides observation/history and five-bar geometry references, with8 independent tests. These are not currently wired into an autonomous FPGA vehicle controller. Wheel torque reference0.1Nm×action cannot be sent as uncalibrated PWM or speed.

User has authorized ignoring weak magnetic field only. Both SSI instances use ALLOW_WEAK_FIELD=1; raw status is retained, CRC and overspeed checks remain active.

BMI088 CS-line identification: normal documented mapping is tried first. Only an initial chip-ID failure permits one retry with the two documented CS lines exchanged. Both IDs (0x1E/0x0F) and every configuration readback must pass before publishing any sample. Byte63 bit3 reports the exchanged mapping. On an initialization failure, two additional read-only CHIP_ID transactions run on physical CS1 and CS2. Byte61 and byte62 hold their one-byte returned IDs; byte63 bit4 marks completion. These probes do not clear the error or publish sensor data. This is not evidence of valid hardware until an actual six-axis sample is observed. `tb_bmi088` tests normal and exchanged CS, disconnected MISO and incorrect configuration readback.
