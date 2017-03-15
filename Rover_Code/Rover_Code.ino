//Motor Imports & Variables
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <SoftwareSerial.h>
#include <Servo.h>
// #include "utility/Adafruit_MS_PWMServoDriver.h"

#define SERIAL_DEBUG 1

// Pin definitions
#define SENSITIVITY_POT_APIN 1
#define SPRAYCAN_PIN 8
#define ZERO_BTN_OUTPUT 13

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *right_motor = AFMS.getMotor(1);
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);

SoftwareSerial xbee(10, 11); // RX, TX
Servo spraycan;

int left_motor_value = 540;
int right_motor_value = 540;
int zero_value = 0;
String COMMA = ", ";

void setup() {
  // Configure I/O pins
  pinMode(SPRAYCAN_PIN, OUTPUT);
  pinMode(ZERO_BTN_OUTPUT, OUTPUT);

  // Configure the servo
  // spraycan.attach(SPRAYCAN_PIN);

  // Configure motor shield with the default frequency 1.6KHz
  AFMS.begin();

  // Configure hardware serial for debugging
  Serial.begin(57600);
  // wait for serial port to connect. Needed for native USB port only
  while (!Serial) ;

  #if SERIAL_DEBUG
  Serial.println("Serial connected");
  #endif

  // set the data rate for the SoftwareSerial port
  xbee.begin(9600);
  #if SERIAL_DEBUG
  Serial.println("Xbee connected");
  Serial.println("Left, Right, Zero");
  #endif

  //spraycan.attach(13);
  //spraycan.write(15);
  right_motor->run(RELEASE);
  left_motor->run(RELEASE);
}

void loop() {
  if (xbee.available()) {
    // read from serial and parse incoming data

    left_motor_value = xbee.parseInt();
    right_motor_value = xbee.parseInt();
    zero_value = xbee.parseInt();

    // Debugging
    #if SERIAL_DEBUG
    Serial.println(left_motor_value + COMMA + right_motor_value + COMMA + zero_value);
    // Serial.println(left_motor_value + String(", ") + right_motor_value + String(", ") + zero_value);
    #endif
  }

  // Pass the value of the zero button, which is itself read xbee, to the metal detector Uno
  // digitalWrite(ZERO_BTN_OUTPUT, zero_value);

  // Adjust the speed and direction of the right and left motors
  controlMotor(right_motor_value, 1);
  controlMotor(left_motor_value, 2);
}

/**
* Control the speed of a motor on the rover.
* @param motor_value a [0, 1023] value that determines the speed and direction of the motor
* @param motor the motor to apply changes to based on the following map:
* 1 = Right track
* 2 = Left track
*/
void controlMotor(int motor_value, int motor) {
  if ((motor_value > 0) && (motor_value <= 500)) {
    // move backwards
    int motor_speed = map(motor_value, 0, 500, 150, 0);
    if (motor == 1) {
      right_motor->run(BACKWARD);
      right_motor->setSpeed(motor_speed);
    } else if (motor == 2) {
      left_motor->run(BACKWARD);
      left_motor->setSpeed(motor_speed);
    }
  } else if ((motor_value > 500) && (motor_value < 580)) {
    // neutral position
    if (motor == 1) {
      right_motor->run(RELEASE);
    } else if (motor == 2) {
      left_motor->run(RELEASE);
    }
  } else if (motor_value >= 580) {
    // move forward
    int motor_speed = map(motor_value, 580, 1023, 0, 150);
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
