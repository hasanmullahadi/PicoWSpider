#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>

#include "RobotWebServer.h"
#include "MotorController.h"
#include "Ultrasonic.h"
#include "ServoController.h"
#include "DHTSensor.h"

// ============================================================================
// Configuration Constants
// ============================================================================

// WiFi
const char *WIFI_SSID = "PicoW";
const char *WIFI_PASSWORD = "12345678";

// Motor speeds
const uint8_t MOTOR_SPEED_NORMAL = 150;
const uint8_t MOTOR_SPEED_SLOW = 100;
const uint8_t MOTOR_SPEED_FAST = 200;

// Timing intervals (ms)
const unsigned long DISPLAY_UPDATE_INTERVAL = 250;
const unsigned long SERVO_UPDATE_INTERVAL = 1000;
const unsigned long OBSTACLE_CLEAR_DELAY = 1000;
const unsigned long TURN_360_DURATION = 3000;

// Obstacle detection threshold (cm)
const float OBSTACLE_DISTANCE_CM = 30.0;

// OLED display
const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 64;
const int SCREEN_ADDRESS = 0x3C;

// ============================================================================
// Pin Definitions
// ============================================================================

// Motor Controller AB
#define PWMA_PIN 22
#define AIN1_PIN 1
#define AIN2_PIN 0
#define PWMB_PIN 26
#define BIN1_PIN 3
#define BIN2_PIN 6
#define STBY_PIN_AB 2

// Motor Controller CD
#define PWMC_PIN 27
#define CIN1_PIN 8
#define CIN2_PIN 7
#define PWMD_PIN 28
#define DIN1_PIN 10
#define DIN2_PIN 11
#define STBY_PIN_CD 9

// Sensors
#define TRIG_PIN 14
#define ECHO_PIN 13
#define DHT_PIN 12
#define DHT_TYPE DHT22

// User interface
#define BUTTON_0 15
#define BUTTON_1 16
#define BUTTON_2 17
#define BUTTON_3 18
#define LED_RED 20
#define LED_GREEN 21
#define LED_BLUE 19

// Servo channels
#define SRV0 0
#define SRV15 15

// ============================================================================
// Global Objects
// ============================================================================

RobotWebServer robotServer(WIFI_SSID, WIFI_PASSWORD, 80, true);
MotorController motorControllerAB(PWMA_PIN, AIN2_PIN, AIN1_PIN, PWMB_PIN, BIN2_PIN, BIN1_PIN, STBY_PIN_AB);
MotorController motorControllerCD(PWMC_PIN, CIN1_PIN, CIN2_PIN, PWMD_PIN, DIN1_PIN, DIN2_PIN, STBY_PIN_CD);
Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN);
ServoController servoController(0x60);
DHTSensor dhtSensor(DHT_PIN, DHT_TYPE);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_MPU6050 mpu;

// State variables
unsigned long previousDisplayMillis = 0;
unsigned long previousServoMillis = 0;
unsigned long obstacleClearMillis = 0;
unsigned long turnStartMillis = 0;
float distance = 0;
float humidity = 0;
float temperature = 0;
int brightness = 255;
bool isSpinning = false;
bool spinDirection = false;
bool isTurning = false;
bool servoMoveRight = false;

// ============================================================================
// Setup
// ============================================================================

void setup() {
    Serial.begin(115200);

    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) delay(10);
    }
    Serial.println("MPU6050 Found!");

    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);

    // Initialize sensors
    ultrasonic.begin();
    dhtSensor.begin();

    // Initialize buttons
    pinMode(BUTTON_0, INPUT_PULLUP);
    pinMode(BUTTON_1, INPUT_PULLUP);
    pinMode(BUTTON_2, INPUT_PULLUP);
    pinMode(BUTTON_3, INPUT_PULLUP);

    // Initialize RGB LED (common anode - inverted)
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    // Initialize I2C and OLED
    Wire.begin();
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        for (;;);
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Mini Robot"));
    display.display();
    delay(2000);

    // Initialize servo controller
    servoController.begin();
    servoController.moveServo(SRV0, 90);
    servoController.moveServo(SRV15, 90);

    // Initialize motor controllers
    motorControllerAB.begin();
    motorControllerCD.begin();
    delay(1000);

    // Start web server
    robotServer.begin();

    String ipAddress = WiFi.localIP().toString();
    Serial.print("IP Address: ");
    Serial.println(ipAddress);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("Connected to WiFi"));
    display.setCursor(0, 20);
    display.println(F("IP Address:"));
    display.setCursor(0, 35);
    display.println(ipAddress);
    display.display();

    // Set web server callbacks
    robotServer.setMoveForwardCallback(moveForward);
    robotServer.setMoveBackwardCallback(moveBackward);
    robotServer.setStopCallback(stopMotors);
    robotServer.setSpinCallback(spin);
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
    unsigned long currentMillis = millis();

    // Update display and sensors
    if (currentMillis - previousDisplayMillis >= DISPLAY_UPDATE_INTERVAL) {
        previousDisplayMillis = currentMillis;
        updateDisplayAndSensors();
    }

    // Update servo positions
    if (currentMillis - previousServoMillis >= SERVO_UPDATE_INTERVAL) {
        previousServoMillis = currentMillis;
        updateServos();
    }

    // Update IMU data
    if (mpu.getMotionInterruptStatus()) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        robotServer.setAcceleration(a.acceleration.x, a.acceleration.y, a.acceleration.z);
        robotServer.setGyroscope(g.gyro.x, g.gyro.y, g.gyro.z);
    }

    // Update web server data
    updateLEDsFromServer();
    robotServer.setDistance(distance);
    robotServer.setDHTValues(temperature, humidity);
    robotServer.handleClient();
}

// ============================================================================
// Display & Sensor Updates
// ============================================================================

void updateDisplayAndSensors() {
    display.clearDisplay();

    // Read sensors
    temperature = dhtSensor.readTemperature();
    humidity = dhtSensor.readHumidity();
    distance = ultrasonic.getDistance();

    // Display temperature and humidity
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(F("T: "));
    display.print(temperature);
    display.print(F("C, H: "));
    display.print(humidity);
    display.println(F("%"));

    // Handle button presses
    handleButtons();

    // Display distance
    display.setCursor(0, 32);
    display.print(F("Distance: "));
    display.print(distance);
    display.println(F(" cm"));

    // Display IP
    display.setCursor(0, 48);
    display.print(F("IP: "));
    display.println(WiFi.localIP().toString());

    display.display();
}

void handleButtons() {
    if (digitalRead(BUTTON_0) == LOW) {
        displayButton(0);
        setRGB(LOW, HIGH, HIGH);  // Red
    } else if (digitalRead(BUTTON_1) == LOW) {
        displayButton(1);
        setRGB(HIGH, LOW, HIGH);  // Green
    } else if (digitalRead(BUTTON_2) == LOW) {
        displayButton(2);
        setRGB(HIGH, HIGH, LOW);  // Blue
    } else if (digitalRead(BUTTON_3) == LOW) {
        displayButton(3);
        toggleBrightness();
    }
}

void displayButton(int buttonNumber) {
    display.setCursor(0, 16);
    display.print(F("Button "));
    display.print(buttonNumber);
    display.println(F(" pressed"));
}

// ============================================================================
// LED Control
// ============================================================================

void setRGB(int red, int green, int blue) {
    // Common anode: invert values
    analogWrite(LED_RED, 255 - red);
    analogWrite(LED_GREEN, 255 - green);
    analogWrite(LED_BLUE, 255 - blue);
}

void toggleBrightness() {
    brightness = (brightness == 255) ? 128 : 255;
    analogWrite(LED_RED, brightness);
    analogWrite(LED_GREEN, brightness);
    analogWrite(LED_BLUE, brightness);
}

void updateLEDsFromServer() {
    uint8_t r, g, b;
    robotServer.getRGBValue(0, r, g, b);
    setRGB(r, g, b);
}

// ============================================================================
// Servo Control
// ============================================================================

void updateServos() {
    if (servoMoveRight) {
        servoController.moveRight(SRV0);
        servoController.moveRight(SRV15);
    } else {
        servoController.moveLeft(SRV0);
        servoController.moveLeft(SRV15);
    }
    servoMoveRight = !servoMoveRight;
}

// ============================================================================
// Motor Control - Basic Movements
// ============================================================================

void moveForward() {
    motorControllerAB.motorAForward(MOTOR_SPEED_NORMAL);
    motorControllerAB.motorBForward(MOTOR_SPEED_NORMAL);
    motorControllerCD.motorAForward(MOTOR_SPEED_NORMAL);
    motorControllerCD.motorBForward(MOTOR_SPEED_NORMAL);
}

void moveBackward() {
    motorControllerAB.motorABackward(MOTOR_SPEED_NORMAL);
    motorControllerAB.motorBBackward(MOTOR_SPEED_NORMAL);
    motorControllerCD.motorABackward(MOTOR_SPEED_NORMAL);
    motorControllerCD.motorBBackward(MOTOR_SPEED_NORMAL);
}

void strafeRight() {
    motorControllerAB.motorAForward(MOTOR_SPEED_NORMAL);
    motorControllerAB.motorBBackward(MOTOR_SPEED_NORMAL);
    motorControllerCD.motorABackward(MOTOR_SPEED_NORMAL);
    motorControllerCD.motorBForward(MOTOR_SPEED_NORMAL);
}

void strafeLeft() {
    motorControllerAB.motorABackward(MOTOR_SPEED_NORMAL);
    motorControllerAB.motorBForward(MOTOR_SPEED_NORMAL);
    motorControllerCD.motorAForward(MOTOR_SPEED_NORMAL);
    motorControllerCD.motorBBackward(MOTOR_SPEED_NORMAL);
}

void stopMotors() {
    motorControllerAB.motorAStop();
    motorControllerAB.motorBStop();
    motorControllerCD.motorAStop();
    motorControllerCD.motorBStop();
}

// ============================================================================
// Motor Control - Spinning
// ============================================================================

void spin() {
    spinLeftSlow();
}

void spinLeftSlow() {
    motorControllerAB.motorAForward(MOTOR_SPEED_SLOW);
    motorControllerAB.motorBForward(MOTOR_SPEED_SLOW);
    motorControllerCD.motorABackward(MOTOR_SPEED_SLOW);
    motorControllerCD.motorBBackward(MOTOR_SPEED_SLOW);
}

void spinRightSlow() {
    motorControllerAB.motorABackward(MOTOR_SPEED_SLOW);
    motorControllerAB.motorBBackward(MOTOR_SPEED_SLOW);
    motorControllerCD.motorAForward(MOTOR_SPEED_SLOW);
    motorControllerCD.motorBForward(MOTOR_SPEED_SLOW);
}

void performSlowSpin(bool direction) {
    if (direction) {
        spinRightSlow();
    } else {
        spinLeftSlow();
    }
}

// ============================================================================
// Autonomous Navigation
// ============================================================================

void movingAround() {
    unsigned long currentMillis = millis();

    if (ultrasonic.getDistance() < OBSTACLE_DISTANCE_CM) {
        if (!isSpinning) {
            isSpinning = true;
            spinDirection = random(0, 2);
        }
        performSlowSpin(spinDirection);
        obstacleClearMillis = currentMillis;
    } else {
        if (isSpinning && (currentMillis - obstacleClearMillis >= OBSTACLE_CLEAR_DELAY)) {
            isSpinning = false;
        }
        if (!isSpinning) {
            moveForward();
        }
    }
}

void perform360Turn() {
    unsigned long currentMillis = millis();

    if (!isTurning) {
        motorControllerAB.motorAForward(MOTOR_SPEED_NORMAL);
        motorControllerAB.motorBBackward(MOTOR_SPEED_NORMAL);
        motorControllerCD.motorABackward(MOTOR_SPEED_NORMAL);
        motorControllerCD.motorBForward(MOTOR_SPEED_NORMAL);
        turnStartMillis = currentMillis;
        isTurning = true;
    }

    if (isTurning && (currentMillis - turnStartMillis >= TURN_360_DURATION)) {
        stopMotors();
        isTurning = false;
    }
}

// ============================================================================
// Testing & Diagnostics
// ============================================================================

void displayStatus(const char *status) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Status:"));
    display.println(status);
    display.display();
}

void testMotors() {
    displayStatus("Testing Motor A");
    motorControllerAB.softStartA(128);
    delay(1000);
    motorControllerAB.softStopA();
    delay(1000);

    displayStatus("Testing Motor B");
    motorControllerAB.softStartB(128);
    delay(1000);
    motorControllerAB.softStopB();
    delay(1000);

    displayStatus("Testing Motor C");
    motorControllerCD.softStartA(128);
    delay(1000);
    motorControllerCD.softStopA();
    delay(1000);

    displayStatus("Testing Motor D");
    motorControllerCD.softStartB(128);
    delay(1000);
    motorControllerCD.softStopB();
    delay(1000);
}

void forwardTest() {
    const char* motorNames[] = {"Front Left (A)", "Back Left (B)", "Back Right (C)", "Front Right (D)"};

    displayStatus(motorNames[0]);
    motorControllerAB.motorAForward(MOTOR_SPEED_FAST);
    delay(2000);
    motorControllerAB.motorAStop();
    delay(1000);

    displayStatus(motorNames[1]);
    motorControllerAB.motorBForward(MOTOR_SPEED_FAST);
    delay(2000);
    motorControllerAB.motorBStop();
    delay(1000);

    displayStatus(motorNames[2]);
    motorControllerCD.motorAForward(MOTOR_SPEED_FAST);
    delay(2000);
    motorControllerCD.motorAStop();
    delay(1000);

    displayStatus(motorNames[3]);
    motorControllerCD.motorBForward(MOTOR_SPEED_FAST);
    delay(2000);
    motorControllerCD.motorBStop();
    delay(1000);

    displayStatus("Forward Test Complete");
    delay(2000);
}
