# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

PicoWSpider is an Arduino project for the **PicoWRobotV1.2** - a robotics prototyping board for Raspberry Pi Pico W. The robot features 4 DC motors with mecanum wheels, servos, sensors (DHT22, ultrasonic, MPU6050 gyro), OLED display, RGB LED, and a web-based control interface.

## Build & Upload

### Arduino IDE
Open `PicoWSpider.ino` in Arduino IDE. Select Board: **Raspberry Pi Pico W**

### Arduino CLI (command line)
```bash
# Install Pico W support (one-time)
arduino-cli config add board_manager.additional_urls https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
arduino-cli core update-index
arduino-cli core install rp2040:rp2040

# Install libraries (one-time)
arduino-cli lib install "Adafruit GFX Library" "Adafruit SSD1306" "Adafruit MPU6050" "Adafruit PWM Servo Driver Library" "DHT sensor library"

# Compile
arduino-cli compile --fqbn rp2040:rp2040:rpipicow .

# Upload (replace /dev/cu.usbmodem* with your port)
arduino-cli upload --fqbn rp2040:rp2040:rpipicow -p /dev/cu.usbmodem* .
```

## Project Structure

```
PicoWSpider/
├── PicoWSpider.ino      # Main sketch (setup, loop, motor functions)
├── MotorController.*    # TB6612FNG dual motor driver
├── ServoController.*    # PCA9685 PWM servo driver (I2C 0x60)
├── DHTSensor.*          # DHT22 temperature/humidity
├── Ultrasonic.*         # HC-SR04 distance sensor
├── RobotWebServer.*     # WiFi AP web server
├── html.*, css.*        # Embedded web interface
└── docs/
    ├── images/          # Pin diagrams and schematics
    └── PicoWv1.stl      # 3D printable chassis
```

## Architecture

### Constants (PicoWSpider.ino)
All configurable values are at the top: `MOTOR_SPEED_*`, `*_INTERVAL`, `OBSTACLE_DISTANCE_CM`, pin definitions.

### Web Server Pattern
RobotWebServer uses callbacks for motor control:
```cpp
robotServer.setMoveForwardCallback(moveForward);
robotServer.setStopCallback(stopMotors);
```

### Pin Assignments
- Motor AB: PWM(22,26), Control(0,1,3,6), Standby(2)
- Motor CD: PWM(27,28), Control(7,8,10,11), Standby(9)
- Ultrasonic: Trig(14), Echo(13)
- DHT22: Pin 12
- Buttons: 15,16,17,18
- RGB LED: R(20), G(21), B(19) - common anode (inverted)
- I2C: OLED(0x3C), Servo(0x60)

### WiFi
Default AP mode: SSID "PicoW", password "12345678". IP shown on OLED.
