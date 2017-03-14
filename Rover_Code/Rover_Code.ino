//Motor Imports & Variables
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <SoftwareSerial.h>
#include <Servo.h>
// #include "utility/Adafruit_MS_PWMServoDriver.h"

#define SERIAL_DEBUG 1

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

byte leftCommandByte;
byte rightCommandByte;
byte zeroCommandByte;


SoftwareSerial xbee(10, 11); // RX, TX
Servo spraycan;

int left_motor_value = 125;
int right_motor_value = 125;
int zero_value = 0;
String COMMA = ", ";

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

  #if SERIAL_DEBUG
  Serial.println("Serial connected");
  #endif

  // set the data rate for the SoftwareSerial port
  xbee.begin(9600);
  #if SERIAL_DEBUG
  Serial.println("Xbee connected");
  #endif

  //spraycan.attach(13);
  //spraycan.write(15);
  right_motor->run(RELEASE);
  left_motor->run(RELEASE);
}

void loop() {
  //Xbee Serial Read
  // Read left stick value
  // if (xbee.available() > 0) {
  //   leftCommandByte = xbee.read();
  //   #if SERIAL_DEBUG
  //   Serial.println("Left:");
  //   Serial.println(leftCommandByte);
  //   #endif
  // }
  // int left_motor_value = (int) leftCommandByte;
  //
  // //Read right stick value
  // if (xbee.available() > 0) {
  //   // Read our command byte
  //   rightCommandByte = xbee.read();
  // }
  // int right_motor_value = (int) rightCommandByte;
  //
  // // Read zeroing command
  // if (xbee.available() > 0) {
  //   // Read our command byte
  //   zeroCommandByte = xbee.read();
  // }
  // int zero_value = (int) zeroCommandByte;


  if (xbee.available()) {
    // read from serial and parse incoming data
 
    left_motor_value = xbee.parseInt();
    right_motor_value = xbee.parseInt();
    zero_value = xbee.parseInt();
    
    // Alternately, use xbee.readBytesUntil('\n', buf, 30);
    // then use strtok to split the strings
    // and atoi to pare them into ints
    
    // Debugging
    Serial.println(left_motor_value + COMMA + right_motor_value + COMMA + zero_value);
//    Serial.print(left_motor_value);
//    Serial.print(", ");
//    Serial.print(right_motor_value);
//    Serial.print(", ");
//    Serial.print(zero_value);
//    Serial.println();
  }

//  digitalWrite(RESET_BTN_OUTPUT, LOW);

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
