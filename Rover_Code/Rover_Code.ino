//Motor Imports & Variables
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <SoftwareSerial.h>
#include <Servo.h>
// #include "utility/Adafruit_MS_PWMServoDriver.h"

#define SERIAL_DEBUG 1
// Pin definitions
#define SPRAYCAN_PIN 7
#define ZERO_BTN_OUTPUT 13
#define ARRAY_LENGTH 2

// Initialize motor shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* right_motor = AFMS.getMotor(1);
Adafruit_DCMotor* left_motor = AFMS.getMotor(2);

// Initialize arrays for controlling motors
bool firstTime = true;
int motorValues[ARRAY_LENGTH];
int initialMotorValues[ARRAY_LENGTH];
float deadZoneOffset[ARRAY_LENGTH];
float deadZoneRange = 0.06;

Servo spraycan;
int zero_value = 0;
int sensitivity; // TODO: default value for sensitivity?
int found = 0;
SoftwareSerial xbee(10, 11); // RX, TX
SoftwareSerial detector(8, 9); // RX, TX

void setup() {
  pinMode(SPRAYCAN_PIN, OUTPUT);
  pinMode(ZERO_BTN_OUTPUT, OUTPUT);

  AFMS.begin();  // create with the default frequency 1.6KHz
  right_motor->run(RELEASE);
  left_motor->run(RELEASE);
  // spraycan.attach(SPRAYCAN_PIN);

  #if SERIAL_DEBUG
  Serial.begin(57600);
  // Wait for serial port to connect. Needed for native USB port only
  while (!Serial) ;
  Serial.println("Serial connected");
  #endif

  // Set the data rate for the SoftwareSerial ports
  xbee.begin(9600);
  //detector.begin(9600);
  #if SERIAL_DEBUG
  Serial.println("Software serial ports connected");
  #endif
}


void loop() {
  if (xbee.available()) {
    // Read from serial and parse incoming data
    // TODO: Add synchronization
    motorValues[0] = xbee.parseInt();
    motorValues[1] = xbee.parseInt();
    zero_value = xbee.parseInt();
    sensitivity = xbee.parseInt();

    // Initialize zero position and deadzone for motor control
    if (firstTime) {
      initialMotorValues[0] = motorValues[0];
      initialMotorValues[1] = motorValues[1];
      deadZoneOffset[0] = motorValues[0] * deadZoneRange;
      deadZoneOffset[1] = motorValues[1] * deadZoneRange;
      firstTime = false;
    }

    // Debugging
    #if SERIAL_DEBUG
    Serial.println(String(motorValues[0]) + ", " + motorValues[1] + ", " + motorValues[2]);
    Serial.print("Calulated Offsets: ");
    Serial.println(String(deadZoneOffset[0]) + ", " + deadZoneOffset[1] + ", " + deadZoneOffset[2]);
    #endif

    // Adjust the speed and direction of the right and left motors
    controlMotor();
  }

  // Send sensitivity and zeroing instructions to the metal detector Uno
  char buffer[20];
  sprintf(buffer, "%d,%d", sensitivity, zero_value);
  detector.println(buffer);

  // Read from metal detector Uno
  if (detector.available()) {
    found = detector.parseInt();
    sprintf(buffer, "%d", found);
    xbee.println(buffer);
  }
}


void controlMotor() {
  for (int i = 0; i < ARRAY_LENGTH; ++i) {
    // Move backwards
    if ((motorValues[i] > 0) && (motorValues[i] <= initialMotorValues[i] - deadZoneOffset[i])) {
      int motor_speed = map(motorValues[i], 0, initialMotorValues[i] - deadZoneOffset[i], 150, 0);
      if (i == 0) {
        // Left motor
        left_motor->run(BACKWARD);
        left_motor->setSpeed(motor_speed);
      } else if (i == 1) {
        // Right motor
        right_motor->run(BACKWARD);
        right_motor->setSpeed(motor_speed);
      }
    } else if ((motorValues[i] > initialMotorValues[i] - deadZoneOffset[i]) && (motorValues[i] < initialMotorValues[i] + deadZoneOffset[i])) {
      //Neutral
      if(i == 0) {
        left_motor->run(RELEASE);
      } else if (i == 1) {
        right_motor->run(RELEASE);
      }
    } else if ((motorValues[i] >= initialMotorValues[i] + deadZoneOffset[i])) {
      // Forward
      int motor_speed = map(motorValues[i], initialMotorValues[i] + deadZoneOffset[i], 1023, 0, 150);
      if (i == 0) {
        // Left motor
        left_motor->run(FORWARD);
        left_motor->setSpeed(motor_speed);
      } else if (i == 1) {
        // Right motor
        right_motor->run(FORWARD);
        right_motor->setSpeed(motor_speed);
      }
    }
  }
}
