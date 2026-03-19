# TVC Flight Computer

A Thrust Vector Control (TVC) flight computer for model rockets, built on the Teensy 4.1 platform.

## Overview

This project implements a complete flight computer for rocket TVC systems. It reads sensor data from an MPU9250 IMU, BMP085/BMP180 barometric pressure sensor, and Neo-6M GPS module. It uses PID control to stabilize the rocket's attitude using servo-controlled thrust vectoring, and logs all flight data to an SD card.

## Hardware

### Main Controller
- **Teensy 4.1** - ARM Cortex-M7 based microcontroller

### Sensors
| Sensor | Interface | Purpose |
|--------|-----------|---------|
| MPU9250 | I2C (0x68) | Accelerometer & Gyroscope (6-DOF) |
| BMP085/BMP180 | I2C | Barometric pressure & altitude |
| Neo-6M GPS | UART (Serial2 @ 9600 baud) | GPS location & speed |

### Communication
| Device | Interface | Purpose |
|--------|-----------|---------|
| ESP-01 | UART (Serial1 @ 115200 baud) | WiFi telemetry & command receive |

### Actuators
| Servo | Pin | Function |
|-------|-----|----------|
| Roll Servo | D4 | Roll thrust vector control |
| Pitch Servo | D5 | Pitch thrust vector control |

### Storage
- **SD Card** - Built-in SD card slot (BUILTIN_SDCARD)

## Flight Modes

| Mode | Description |
|------|-------------|
| `MODE_SAFE` | Safe mode - servos centered, no control |
| `MODE_STABILIZE` | Attitude stabilization using PID |
| `MODE_ASCENT` | Ascent phase control |
| `MODE_DESCENT` | Descent phase control |
| `MODE_LAND` | Landing phase |
| `MODE_SIM` | Simulation mode for testing |

## Control Algorithm

### Kalman Filter
A 1D Kalman filter is implemented for roll angle estimation, combining accelerometer and gyroscope data for optimal noise rejection.

### PID Controller
Proportional-Integral-Derivative controller for roll stabilization:
- **Kp** = 1.0 (Proportional gain)
- **Ki** = 0.02 (Integral gain)  
- **Kd** = 1.2 (Derivative gain)

## Telemetry

### Serial Output (JSON format)
The flight computer outputs telemetry at 10 Hz to the ESP-01 in JSON format:

```json
{"t":12345,"mode":"STABILIZE","roll":1.23,"pitch":0.45,"yaw":0.0,"alt":125.5,"lat":40.7128,"lng":-74.0060,"servo":95}
```

### Data Fields
| Field | Type | Description |
|-------|------|-------------|
| `t` | unsigned long | Milliseconds since boot |
| `mode` | string | Current flight mode |
| `roll` | float | Roll angle (degrees) |
| `pitch` | float | Pitch angle (degrees) |
| `yaw` | float | Yaw angle (degrees) |
| `alt` | float | Altitude relative to ground (meters) |
| `lat` | double | GPS latitude |
| `lng` | double | GPS longitude |
| `servo` | int | Servo angle |

### Command Protocol
Commands are received from ESP-01 in format:
```
MODE:<mode_number>
```

## Data Logging

Flight data is logged to `rocket.csv` on the SD card with the following columns:
```
time,mode,roll,pitch,yaw,alt,lat,lng,rollErr,pid,servo
```

## Dependencies

### PlatformIO Libraries
- [Adafruit BMP085 Library](https://github.com/adafruit/Adafruit_BMP085_Library) v1.2.4
- [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus) v1.1.0

### Built-in Libraries
- Wire.h (I2C communication)
- Servo.h (Servo control)
- SD.h (SD card access)
- SPI.h (SPI communication)

## Building

1. Open project in PlatformIO
2. Build: `pio run`
3. Upload: `pio run --target upload`

### PlatformIO Configuration

```ini
[env:teensy41]
platform = teensy
board = teensy41
framework = arduino

lib_deps =
    adafruit/Adafruit BMP085 Library@1.2.4
    mikalhart/TinyGPSPlus@1.1.0
```

## Pin Connections

```
Teensy 4.1     Component
-----------    ---------
D4             Roll Servo signal
D5             Pitch Servo signal
D0 (RX1)       ESP-01 TX
D1 (TX1)       ESP-01 RX
D9 (RX2)       Neo-6M GPS TX
D10 (TX2)      Neo-6M GPS RX
D18 (SDA)      MPU9250 SDA, BMP085 SDA
D19 (SCL)      MPU9250 SCL, BMP085 SCL
BUILTIN_SDCARD SD Card (built-in)
```

## ESP-01 WiFi Firmware

The project includes ESP-01 firmware for WiFi telemetry. The firmware is located in `src/WIFI_firmware/` and provides:
- Web interface for real-time telemetry viewing
- Command sending capability
- WiFi connectivity for ground station communication

## License

MIT License

## Author

Created for model rocket TVC flight control applications.
