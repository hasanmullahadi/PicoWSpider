#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


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
MotorController motorControllerAB(PWMA_PIN, AIN1_PIN, AIN2_PIN, PWMB_PIN, BIN1_PIN, BIN2_PIN, STBY_PIN_AB);
MotorController motorControllerCD(PWMC_PIN, CIN1_PIN, CIN2_PIN, PWMD_PIN, DIN1_PIN, DIN2_PIN, STBY_PIN_CD);



#include "Ultrasonic.h"

// Define the pins for the ultrasonic sensor
#define TRIG_PIN 14
#define ECHO_PIN 13

// Create an instance of the Ultrasonic class
Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN);


#include "ServoController.h"

#define SRV0 0  // Define the channel number for SRV0
#define SRV15 15  // Define the channel number for SRV0


// In your setup function or wherever you instantiate the ServoController:
ServoController servoController(0x60);  // Using the address 0x60





#include "DHTSensor.h"

#define DHT_PIN 12        // Define the pin where the DHT sensor is connected
#define DHT_TYPE DHT22    // Define the DHT sensor type (e.g., DHT11, DHT22)

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
#define LED_RED 19
#define LED_GREEN 20
#define LED_BLUE 21

int brightness = 255;  // Initial brightness level

void setup() {
  // Initialize the DHT sensor
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
    for (;;);  // If allocation fails, enter an infinite loop
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

}

void loop() {
  // Clear the display buffer
  display.clearDisplay();

  // Read temperature and humidity
  float temperature = dhtSensor.readTemperature();
  float humidity = dhtSensor.readHumidity();

  // Display temperature
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("T: "));
  display.print(temperature);
  display.print(F("C"));

  // Display humidity
 // display.setCursor(0, 0);
  display.print(F(",H: "));
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
  float distance = ultrasonic.getDistance();
  displayDistance(distance);
  // // Display the readings on the screen
 
 movingAround();
 //forwardTest();

  //  testMotors();
  // display.display();
  // // Wait before the next update

  // //Move the servo left (0 degrees)
  // servoController.moveLeft(SRV0);
  // servoController.moveLeft(SRV15);
  // delay(1000);  // Wait for 1 second

  // // Move the servo right (180 degrees)
  // servoController.moveRight(SRV0);
  // servoController.moveRight(SRV15);
  // delay(1000);  // Wait for 1 second

  // delay(2000);
}

void displayButton(int buttonNumber) {
  display.setCursor(0, 16);  // Adjust to display below temperature and humidity
  display.print("Button ");
  display.print(buttonNumber);
  display.println(" pressed");
}

void controlRGB(int red, int green, int blue) {
  analogWrite(LED_RED, red ? brightness : 0);
  analogWrite(LED_GREEN, green ? brightness : 0);
  analogWrite(LED_BLUE, blue ? brightness : 0);
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


void testMotors()
{
 // Test Motor A (Controller AB)
    display.setCursor(0, 46);
    display.print("Motor A");
    display.display();
    motorControllerAB.softStartA(128);  // Soft start Motor A to half speed
    delay(1000);  // Run for 2 seconds
    motorControllerAB.softStopA();  // Soft stop Motor A
    delay(1000);  // Pause for 1 second
   // display.setCursor(0, 64);
    display.print("B");
    display.display();
    // Test Motor B (Controller AB)
    motorControllerAB.softStartB(128);  // Soft start Motor B to half speed
    delay(1000);  // Run for 2 seconds
    motorControllerAB.softStopB();  // Soft stop Motor B
    delay(1000);  // Pause for 1 second
    //display.setCursor(0, 64);
    display.print("C");
    display.display();
    // Test Motor C (Controller CD)
    motorControllerCD.softStartA(128);  // Soft start Motor C to half speed
    delay(1000);  // Run for 2 seconds
    motorControllerCD.softStopA();  // Soft stop Motor C
    delay(1000);  // Pause for 1 second
   // display.setCursor(0, 64);
    display.print("D");
    display.display();
    // Test Motor D (Controller CD)
    motorControllerCD.softStartB(128);  // Soft start Motor D to half speed
    delay(1000);  // Run for 2 seconds
    motorControllerCD.softStopB();  // Soft stop Motor D
    delay(1000);  // Pause for 1 second
}


void movingAround()
{
  // Check for obstacle
    if (ultrasonic.getDistance() < 30) {
        displayStatus("Obstacle Detected");
        perform360Turn();
    } else {
        displayStatus("Moving Forward");
        moveForward();
        delay(2000); // Move forward for 2 seconds
        
        displayStatus("Strafing Right");
        strafeRight();
        delay(2000); // Strafe right for 2 seconds

        displayStatus("Strafing Left");
        strafeLeft();
        delay(2000); // Strafe left for 2 seconds
    }
}

// Function to move the robot forward
void moveForward() {
    motorControllerAB.motorAForward(200);  // Front Left (Motor A)
    motorControllerAB.motorBForward(200);  // Back Left (Motor B)
    motorControllerCD.motorBForward(200);  // Back Right (Motor C)
    motorControllerCD.motorAForward(200);  // Front Right (Motor D)
}

// Function to strafe right (shift right)
void strafeRight() {
    motorControllerAB.motorAForward(200);  // Front Left (Motor A) forward
    motorControllerAB.motorBBackward(200);  // Back Left (Motor B) backward
    motorControllerCD.motorBForward(200);  // Back Right (Motor C) forward
    motorControllerCD.motorABackward(200);  // Front Right (Motor D) backward
}

// Function to strafe left (shift left)
void strafeLeft() {
    motorControllerAB.motorABackward(200);  // Front Left (Motor A) backward
    motorControllerAB.motorBForward(200);  // Back Left (Motor B) forward
    motorControllerCD.motorBBackward(200);  // Back Right (Motor C) backward
    motorControllerCD.motorAForward(200);  // Front Right (Motor D) forward
}

// Function to perform a 360-degree turn
void perform360Turn() {
    motorControllerAB.motorAForward(200);  // Front Left (Motor A) forward
    motorControllerAB.motorBBackward(200);  // Back Left (Motor B) backward
    motorControllerCD.motorBForward(200);  // Back Right (Motor C) forward
    motorControllerCD.motorABackward(200);  // Front Right (Motor D) backward
    delay(3000); // Delay for a full 360-degree turn (adjust based on your robot's speed)
    stopAllMotors();  // Stop all motors after the turn
    displayStatus("360 Turn Completed");
}

// Function to stop all motors
void stopAllMotors() {
    motorControllerAB.motorAStop();
    motorControllerAB.motorBStop();
    motorControllerCD.motorAStop();
    motorControllerCD.motorBStop();
}

// Function to display the current status on the OLED
void displayStatus(const char* status) {
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
    delay(2000); // Move forward for 2 seconds
    motorControllerAB.motorAStop();
    delay(1000); // Pause for 1 second

    // Test Motor B (Back Left)
    displayStatus("Back Left (B) Moving");
    motorControllerAB.motorBForward(200);
    delay(2000); // Move forward for 2 seconds
    motorControllerAB.motorBStop();
    delay(1000); // Pause for 1 second

    // Test Motor C (Back Right)
    displayStatus("Back Right (C) Moving");
    motorControllerCD.motorAForward(200);
    delay(2000); // Move forward for 2 seconds
    motorControllerCD.motorAStop();
    delay(1000); // Pause for 1 second

    // Test Motor D (Front Right)
    displayStatus("Front Right (D) Moving");
    motorControllerCD.motorBForward(200);
    delay(2000); // Move forward for 2 seconds
    motorControllerCD.motorBStop();
    delay(1000); // Pause for 1 second

    // Display that all tests are complete
    displayStatus("Forward Test Complete");
    delay(2000);
}



