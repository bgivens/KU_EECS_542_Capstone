//Motor Imports & Variables
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <SoftwareSerial.h>
#include <Servo.h>
// #include "utility/Adafruit_MS_PWMServoDriver.h"

#define SERIAL_DEBUG FALSE

// Number of cycles from external counter needed to generate a signal event
#define CYCLES_PER_SIGNAL 5000

// Frequency delta threshold for triggering
#define MARKING_THRESHOLD 600

// Pin definitions
#define SENSITIVITY_POT_APIN 1
#define MARKING_PIN 8
#define TRIGGER_BTN_PIN 11
#define RESET_BTN_PIN 12
#define RESET_BTN_OUTPUT 13

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *right_motor = AFMS.getMotor(1);
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);

byte commandByte;
SoftwareSerial xbee(10, 11); // RX, TX
Servo spraycan;

int zero_value = 1;

void setup() {
  pinMode(MARKING_PIN, OUTPUT);
  pinMode(TRIGGER_BTN_PIN, INPUT_PULLUP);
  pinMode(RESET_BTN_PIN, INPUT_PULLUP);
  pinMode(RESET_BTN_OUTPUT, OUTPUT);

  // markingServo.attach(9);

  Serial.begin(57600);
  AFMS.begin();  // create with the default frequency 1.6KHz
  // wait for serial port to connect. Needed for native USB port only
  while (!Serial) ;

  // set the data rate for the SoftwareSerial port
  xbee.begin(9600);

  //spraycan.attach(13);
  //spraycan.write(15);
  right_motor->run(RELEASE);
  left_motor->run(RELEASE);
}

void loop() {
  //Xbee Serial Read
  // Read left stick value
  if (xbee.available() > 0) {
    commandByte = xbee.read();
    #if SERIAL_DEBUG
    Serial.println("Left:");
    Serial.println(commandByte);
    #endif
  }
  int left_motor_value = (int) commandByte;

  //Read right stick value
  if (xbee.available() > 0) {
    // Read our command byte
    commandByte = xbee.read();
  }
  int right_motor_value = (int) commandByte;

  // Read zeroing command
  if (xbee.available() > 0) {
    // Read our command byte
    commandByte = xbee.read();
  }
  int zero_value = (int) commandByte;


  if (zero_value == 0) {
    storedTimeDelta = 0;
    digitalWrite(RESET_BTN_OUTPUT, HIGH);
  }

  digitalWrite(RESET_BTN_OUTPUT, LOW);

  // Adjust the speed and direction of the right and left motors
  controlMotor(right_motor_value, 1);
  controlMotor(leftt_motor_value, 2);
}

/**
* Control the speed of a motor on the rover.
* @param motor_value a [0, 255] value that determines the speed and direction of the motor
* @param motor the motor to apply changes to based on the following map:
* 1 = Right track
* 2 = Left track
*/
void controlMotor(int motor_value, int motor) {
  if ((motor_value > 0) && (motor_value <= 110)) {
    int motor_speed = map(motor_value, 0, 110, 150, 0);
    if (motor == 1) {
      right_motor->run(BACKWARD);
      right_motor->setSpeed(motor_speed);
    } else if (motor == 2) {
      left_motor->run(BACKWARD);
      left_motor->setSpeed(motor_speed);
    }
  } else if ((motor_value > 110) && (motor_value < 140) {
    if (motor == 1) {
      right_motor->run(RELEASE);
    } else if (motor == 2) {
      left_motor->run(RELEASE);
    }
  } else if (motor_value >= 140) {
    int motor_speed = map(motor_value, 140, 255, 0, 150);
    if (motor == 1) {
      right_motor->run(FORWARD);
      right_motor->setSpeed(motor_speed);
    } else if (motor == 2) {
      left_motor->run(FORWARD);
      left_motor->setSpeed(motor_speed);
    }
  }
}

/**
* Map an integer from range [inMin, inMax] to a floating point range [outMin, outMax]
* @param input the integer to be mapped
* @param inMin minimum of input range
* @param inMax maximum of input range
* @param outMin minimum of output range
* @param outMax maximum of output range
* @return mapped floating point value
*/
float mapFloat(int input, int inMin, int inMax, float outMin, float outMax) {
  float scale = (float)(input - inMin) / (inMax - inMin);
  return ((outMax - outMin) * scale) + outMin;
}
