#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Adafruit_MPU6050.h>

#include "RobotWebServer.h"


// Wi-Fi credentials

const char *ssid = "YourWifi";     
const char *password = "YourWifiPass";  
RobotWebServer robotServer(ssid, password);


#include "MotorController.h"

// Define pins for Motor Controller AB (MOTORAB)
#define PWMA_PIN 22
#define AIN1_PIN 1
#define AIN2_PIN 0
#define PWMB_PIN 26
#define BIN1_PIN 3
#define BIN2_PIN 6
#define STBY_PIN_AB 2  // Standby pin for MOTORAB

// Define pins for Motor Controller CD (MOTORCD)
#define PWMC_PIN 27
#define CIN1_PIN 8
#define CIN2_PIN 7
#define PWMD_PIN 28
#define DIN1_PIN 10
#define DIN2_PIN 11
#define STBY_PIN_CD 9  // Standby pin for MOTORCD

// Create instances for both motor controllers
MotorController motorControllerAB(PWMA_PIN, AIN2_PIN, AIN1_PIN, PWMB_PIN, BIN2_PIN, BIN1_PIN, STBY_PIN_AB);
MotorController motorControllerCD(PWMC_PIN, CIN1_PIN, CIN2_PIN, PWMD_PIN, DIN1_PIN, DIN2_PIN, STBY_PIN_CD);



#include "Ultrasonic.h"

// Define the pins for the ultrasonic sensor
#define TRIG_PIN 14
#define ECHO_PIN 13

// Create an instance of the Ultrasonic class
Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN);


#include "ServoController.h"

#define SRV0 0    // Define the channel number for SRV0
#define SRV15 15  // Define the channel number for SRV0


// In your setup function or wherever you instantiate the ServoController:
ServoController servoController(0x60);  // Using the address 0x60





#include "DHTSensor.h"

#define DHT_PIN 12      // Define the pin where the DHT sensor is connected
#define DHT_TYPE DHT22  // Define the DHT sensor type (e.g., DHT11, DHT22)

#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< I2C address for the OLED

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Instantiate the DHTSensor object
DHTSensor dhtSensor(DHT_PIN, DHT_TYPE);

// Button pins
#define BUTTON_0 15
#define BUTTON_1 16
#define BUTTON_2 17
#define BUTTON_3 18

// RGB LED pins
#define LED_RED 20
#define LED_GREEN 21
#define LED_BLUE 19

int brightness = 255;  // Initial brightness level


Adafruit_MPU6050 mpu;


void setup() {
  // Initialize the DHT sensor

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // MPU setups
  // setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);  // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);



  ultrasonic.begin();

  dhtSensor.begin();

  // Initialize buttons
  pinMode(BUTTON_0, INPUT_PULLUP);
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);
  pinMode(BUTTON_3, INPUT_PULLUP);

  // Initialize RGB LED
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  // Initialize the I2C communication
  Wire.begin();

  // Initialize the display with the I2C address 0x3C
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    for (;;)
      ;  // If allocation fails, enter an infinite loop
  }

  // Clear the buffer
  display.clearDisplay();

  // Display initial message
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Mini Robot"));
  display.display();  // Display the message on the screen
  delay(2000);        // Wait for 2 seconds

  // Initialize the Servo Controller
  servoController.begin();

  // Move the servo to the center (90 degrees) initially
  servoController.moveServo(SRV0, 90);
  servoController.moveServo(SRV15, 90);

  // Initialize both motor controllers
  motorControllerAB.begin();
  motorControllerCD.begin();

  delay(1000);

  // Server Code
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Get IP address
  String ipAddress = WiFi.localIP().toString();
  Serial.print("IP Address: ");
  Serial.println(ipAddress);

  // Display IP address on OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Connected to WiFi"));
  display.setCursor(0, 20);
  display.println(F("IP Address:"));
  display.setCursor(0, 35);
  display.println(ipAddress);
  display.display();

  // Set up the web server and control functions
  robotServer.setMoveForwardCallback(moveForward);
  robotServer.setMoveBackwardCallback(moveBackward);
  robotServer.setStopCallback(stopMotors);
  robotServer.setSpinCallback(spin);

  // Start the web server
  robotServer.begin();
}

// Unique timing variables for different tasks in the main loop
unsigned long previousDisplayMillis = 0;
const long displayUpdateInterval = 250;  // Update display and sensors every 250ms

unsigned long previousServoMillis = 0;
const long servoUpdateInterval = 1000;  // Update servo positions every 1 second
float distance;
float humidity;
float temperature;
void loop() {
  unsigned long currentMillis = millis();


  // Non-blocking display and sensor updates
  if (currentMillis - previousDisplayMillis >= displayUpdateInterval) {
    previousDisplayMillis = currentMillis;

    // Clear the display buffer
    display.clearDisplay();

    // Read temperature and humidity
    temperature = dhtSensor.readTemperature();
    humidity = dhtSensor.readHumidity();

    // Display temperature
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(F("T: "));
    display.print(temperature);
    display.print(F("C"));

    // Display humidity
    display.print(F(", H: "));
    display.print(humidity);
    display.println(F("%"));

    // Check button states and display button number when pressed
    if (digitalRead(BUTTON_0) == LOW) {
      displayButton(0);
      controlRGB(LOW, HIGH, HIGH);  // Red ON
    } else if (digitalRead(BUTTON_1) == LOW) {
      displayButton(1);
      controlRGB(HIGH, LOW, HIGH);  // Green ON
    } else if (digitalRead(BUTTON_2) == LOW) {
      displayButton(2);
      controlRGB(HIGH, HIGH, LOW);  // Blue ON
    } else if (digitalRead(BUTTON_3) == LOW) {
      displayButton(3);
      adjustBrightness();  // Adjust Brightness
    }

    // Read distance from ultrasonic sensor and display it
    distance = ultrasonic.getDistance();
    displayDistance(distance);
    String ipAddress = WiFi.localIP().toString();
    display.setCursor(0, 32 + 16);
    display.print(F("IP: "));
    display.println(ipAddress);
    // Update the display with new information
    display.display();
  }

  // Non-blocking servo control
  if (currentMillis - previousServoMillis >= servoUpdateInterval) {
    previousServoMillis = currentMillis;

    // Move the servo left (0 degrees)
    servoController.moveLeft(SRV0);
    servoController.moveLeft(SRV15);

    // Move the servo right (180 degrees)
    static bool moveRight = false;
    if (moveRight) {
      servoController.moveRight(SRV0);
      servoController.moveRight(SRV15);
    }

    moveRight = !moveRight;  // Toggle between left and right movements
  }

  // Call non-blocking movement function
  if (mpu.getMotionInterruptStatus()) {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    robotServer.setAcceleration(a.acceleration.x, a.acceleration.y, a.acceleration.z);
    robotServer.setGyroscope(g.gyro.x, g.gyro.y, g.gyro.z);
  }
  updateLEDsFromServer();
  robotServer.setDistance(distance);
  robotServer.setDHTValues(temperature, humidity);  // Set the values in the server
  robotServer.handleClient();

  // movingAround();
}

void updateLEDsFromServer() {
  // Loop through each RGB LED and update its color based on server values

  uint8_t r, g, b;
  robotServer.getRGBValue(0, r, g, b);  // Get the RGB value for the current LED
  // Serial.print("R: ");
  controlRGB(r, g, b);  //ledController.setRGB(i, r, g, b);    // Apply the RGB value to the LED

  // Serial.print("Original RGB: (");
  // Serial.print(r);
  // Serial.print(", ");
  // Serial.print(g);
  // Serial.print(", ");
  // Serial.print(b);
  // Serial.println(")");
}




void displayButton(int buttonNumber) {
  display.setCursor(0, 16);  // Adjust to display below temperature and humidity
  display.print("Button ");
  display.print(buttonNumber);
  display.println(" pressed");
}

void controlRGB(int red, int green, int blue) {
  analogWrite(LED_RED, 255-red);
  analogWrite(LED_GREEN, 255-green);
  analogWrite(LED_BLUE, 255-blue);
}

void adjustBrightness() {
  brightness = brightness == 255 ? 128 : 255;  // Toggle between full and half brightness
  analogWrite(LED_RED, brightness);
  analogWrite(LED_GREEN, brightness);
  analogWrite(LED_BLUE, brightness);
}

void displayDistance(float distance) {
  // display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 32);
  display.print("Distance: ");
  display.print(distance);
  display.println(" cm");
  // display.display();
}


void testMotors() {
  // Test Motor A (Controller AB)
  display.setCursor(0, 46);
  display.print("Motor A");
  display.display();
  motorControllerAB.softStartA(128);  // Soft start Motor A to half speed
  delay(1000);                        // Run for 2 seconds
  motorControllerAB.softStopA();      // Soft stop Motor A
  delay(1000);                        // Pause for 1 second
                                      // display.setCursor(0, 64);
  display.print("B");
  display.display();
  // Test Motor B (Controller AB)
  motorControllerAB.softStartB(128);  // Soft start Motor B to half speed
  delay(1000);                        // Run for 2 seconds
  motorControllerAB.softStopB();      // Soft stop Motor B
  delay(1000);                        // Pause for 1 second
  //display.setCursor(0, 64);
  display.print("C");
  display.display();
  // Test Motor C (Controller CD)
  motorControllerCD.softStartA(128);  // Soft start Motor C to half speed
  delay(1000);                        // Run for 2 seconds
  motorControllerCD.softStopA();      // Soft stop Motor C
  delay(1000);                        // Pause for 1 second
                                      // display.setCursor(0, 64);
  display.print("D");
  display.display();
  // Test Motor D (Controller CD)
  motorControllerCD.softStartB(128);  // Soft start Motor D to half speed
  delay(1000);                        // Run for 2 seconds
  motorControllerCD.softStopB();      // Soft stop Motor D
  delay(1000);                        // Pause for 1 second
}
// Global variables for non-blocking movement and spinning
unsigned long previousMillis = 0;
unsigned long obstacleClearMillis = 0;  // Timer to track how long the obstacle has been cleared
const long interval = 2000;             // Time between forward movements
const long clearDelay = 1000;           // Time to wait after the obstacle is cleared before moving forward
bool isSpinning = false;                // Track whether the robot is spinning
bool spinDirection = false;             // Spin direction: false = left, true = right

void movingAround() {
  unsigned long currentMillis = millis();

  // Check for obstacle
  if (ultrasonic.getDistance() < 30) {
    // Start spinning if an obstacle is detected
    if (!isSpinning) {
      isSpinning = true;             // Set spinning flag
      spinDirection = random(0, 2);  // Randomize spin direction (0 for left, 1 for right)
    }
    // Perform slow spin until the obstacle is cleared
    performSlowSpin(spinDirection);
    // Reset the obstacle clear timer
    obstacleClearMillis = currentMillis;
  } else {
    // Only stop spinning if the obstacle has been cleared for a sustained time (debouncing)
    if (isSpinning && (currentMillis - obstacleClearMillis >= clearDelay)) {
      isSpinning = false;  // Stop spinning
    }

    // Move forward (non-blocking) after the clear delay
    if (!isSpinning && currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;  // Reset the time
      moveForward();                   // Move forward continuously
    }
  }
}

// Function to perform a slow right spin
void spinLeftSlow() {
  motorControllerAB.motorAForward(100);   // Front Left (Motor A) forward at slow speed
  motorControllerAB.motorBForward(100);   // Back Left (Motor B) backward at slow speed
  motorControllerCD.motorBBackward(100);  // Back Right (Motor C) forward at slow speed
  motorControllerCD.motorABackward(100);  // Front Right (Motor D) backward at slow speed
}

// Function to perform a slow left spin
void spinRightSlow() {
  motorControllerAB.motorABackward(100);  // Front Left (Motor A) backward at slow speed
  motorControllerAB.motorBBackward(100);  // Back Left (Motor B) forward at slow speed
  motorControllerCD.motorBForward(100);   // Back Right (Motor C) backward at slow speed
  motorControllerCD.motorAForward(100);   // Front Right (Motor D) forward at slow speed
}

// Function to perform a slow spin in a specified direction
void performSlowSpin(bool direction) {
  if (direction) {
    spinRightSlow();
  } else {
    spinLeftSlow();
  }
}




// Function to move the robot forward
void moveForward() {
  motorControllerAB.motorAForward(150);  // Front Left (Motor A)
  motorControllerAB.motorBForward(150);  // Back Left (Motor B)
  motorControllerCD.motorBForward(150);  // Back Right (Motor C)
  motorControllerCD.motorAForward(150);  // Front Right (Motor D)
}

void moveBackward() {
  motorControllerAB.motorABackward(150);  // Front Left (Motor A)
  motorControllerAB.motorBBackward(150);  // Back Left (Motor B)
  motorControllerCD.motorBBackward(150);  // Back Right (Motor C)
  motorControllerCD.motorABackward(150);  // Front Right (Motor D)
}


// Function to strafe right (shift right)
void strafeRight() {
  motorControllerAB.motorAForward(150);   // Front Left (Motor A) forward
  motorControllerAB.motorBBackward(150);  // Back Left (Motor B) backward
  motorControllerCD.motorBForward(150);   // Back Right (Motor C) forward
  motorControllerCD.motorABackward(150);  // Front Right (Motor D) backward
}

// Function to strafe left (shift left)
void strafeLeft() {
  motorControllerAB.motorABackward(150);  // Front Left (Motor A) backward
  motorControllerAB.motorBForward(150);   // Back Left (Motor B) forward
  motorControllerCD.motorBBackward(150);  // Back Right (Motor C) backward
  motorControllerCD.motorAForward(150);   // Front Right (Motor D) forward
}

void stopMotors() {
  motorControllerAB.motorAStop();
  motorControllerAB.motorBStop();
  motorControllerCD.motorAStop();
  motorControllerCD.motorBStop();
}

void spin() {
  spinLeftSlow();
}
// Global variables for non-blocking 360-degree turn
unsigned long turnStartMillis = 0;
const long turnDuration = 3000;  // Duration for a full 360-degree turn (adjust based on your robot's speed)
bool isTurning = false;

void perform360Turn() {
  unsigned long currentMillis = millis();

  if (!isTurning) {
    // Start the 360-degree turn
    motorControllerAB.motorAForward(150);   // Front Left (Motor A) forward
    motorControllerAB.motorBBackward(150);  // Back Left (Motor B) backward
    motorControllerCD.motorBForward(150);   // Back Right (Motor C) forward
    motorControllerCD.motorABackward(150);  // Front Right (Motor D) backward
    turnStartMillis = currentMillis;        // Store the start time
    isTurning = true;                       // Set the turning flag
  }

  // Check if the turn duration has passed
  if (isTurning && currentMillis - turnStartMillis >= turnDuration) {
    // Stop all motors after the turn is completed
    stopAllMotors();
    isTurning = false;  // Reset the turning flag
                        // displayStatus("360 Turn Completed");
  }
}

// Function to stop all motors
void stopAllMotors() {
  motorControllerAB.motorAStop();
  motorControllerAB.motorBStop();
  motorControllerCD.motorAStop();
  motorControllerCD.motorBStop();
}

// Function to display the current status on the OLED
void displayStatus(const char *status) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Status:");
  display.println(status);
  display.display();
}


void forwardTest() {
  // Test Motor A (Front Left)
  displayStatus("Front Left (A) Moving");
  motorControllerAB.motorAForward(200);
  delay(2000);  // Move forward for 2 seconds
  motorControllerAB.motorAStop();
  delay(1000);  // Pause for 1 second

  // Test Motor B (Back Left)
  displayStatus("Back Left (B) Moving");
  motorControllerAB.motorBForward(200);
  delay(2000);  // Move forward for 2 seconds
  motorControllerAB.motorBStop();
  delay(1000);  // Pause for 1 second

  // Test Motor C (Back Right)
  displayStatus("Back Right (C) Moving");
  motorControllerCD.motorAForward(200);
  delay(2000);  // Move forward for 2 seconds
  motorControllerCD.motorAStop();
  delay(1000);  // Pause for 1 second

  // Test Motor D (Front Right)
  displayStatus("Front Right (D) Moving");
  motorControllerCD.motorBForward(200);
  delay(2000);  // Move forward for 2 seconds
  motorControllerCD.motorBStop();
  delay(1000);  // Pause for 1 second

  // Display that all tests are complete
  displayStatus("Forward Test Complete");
  delay(2000);
}
