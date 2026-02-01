# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

PicoWSpider is an Arduino project for the **PicoWRobotV1.2** - a robotics prototyping board for Raspberry Pi Pico W. The robot features 4 DC motors with mecanum wheels, servos, sensors (DHT22, ultrasonic, MPU6050 gyro), OLED display, RGB LED, and a web-based control interface.

## Build & Upload

This is an Arduino IDE project. Open `PicoWSpider.ino` in Arduino IDE.

**Required Board**: Raspberry Pi Pico W (select in Arduino IDE: Tools > Board > Raspberry Pi Pico W)

**Required Libraries** (install via Arduino Library Manager):
- Adafruit GFX Library
- Adafruit SSD1306
- Adafruit MPU6050
- Adafruit PWM Servo Driver Library
- DHT sensor library

## Architecture

### Main Entry Point
`PicoWSpider.ino` - Contains setup(), loop(), and motor movement functions (moveForward, moveBackward, spin, strafe, etc.)

### Hardware Controllers (Header + Implementation pairs)
- `MotorController.h/.cpp` - TB6612FNG dual motor driver control (supports 2 motors per instance, soft start/stop)
- `ServoController.h/.cpp` - PCA9685 PWM servo driver via I2C (address 0x60)
- `DHTSensor.h/.cpp` - DHT22 temperature/humidity sensor wrapper
- `Ultrasonic.h/.cpp` - HC-SR04 ultrasonic distance sensor

### Web Server
- `RobotWebServer.h/.cpp` - WiFi AP mode web server for remote control
- `html.h/.cpp` - Web interface HTML (stored as raw string literal)
- `css.h/.cpp` - Web interface CSS (stored as raw string literal)

The web server uses callback pattern for motor control - set callbacks via `setMoveForwardCallback()`, etc.

### Pin Assignments (defined in PicoWSpider.ino)
- Motor AB: PWM(22,26), Control(0,1,3,6), Standby(2)
- Motor CD: PWM(27,28), Control(7,8,10,11), Standby(9)
- Ultrasonic: Trig(14), Echo(13)
- DHT22: Pin 12
- Buttons: 15,16,17,18
- RGB LED: R(20), G(21), B(19) - common anode (inverted PWM)
- I2C devices: OLED(0x3C), Servo Driver(0x60)

### WiFi Configuration
Default AP mode creates network "PicoW" with password "12345678". IP displayed on OLED after boot.
