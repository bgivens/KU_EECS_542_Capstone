//Motor Imports & Variables
#include <Wire.h>
#include <Adafruit_MotorShield.h>
//#include <SoftwareSerial.h>
#include <Servo.h>
// #include "utility/Adafruit_MS_PWMServoDriver.h"

#define BAUDRATE 9600

// Pin definitions
#define SPRAYCAN_PIN 7
#define FOUND_PIN 9
#define ZERO_PIN 8
#define ERROR_HIGH 4
#define ERROR_LOW 3


// Initialize motor shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* right_motor = AFMS.getMotor(1);
Adafruit_DCMotor* left_motor = AFMS.getMotor(2);

// Variables
Servo spraycan;
int zero_value = 0;
int found = 0;
int left_motor_val = 255;
int right_motor_val = 255;
int new_left_motor = 255;
int new_right_motor = 255;
char buffer[20]; // Buffer for sending/receiving strings


void setup() {
  pinMode(SPRAYCAN_PIN, OUTPUT);

  AFMS.begin();  // create with the default frequency 1.6KHz
  right_motor->run(RELEASE);
  left_motor->run(RELEASE);
  // spraycan.attach(SPRAYCAN_PIN);

  // Initialize hardware serial
  Serial.begin(BAUDRATE);
  // Wait for serial port to connect. Needed for hardware serial
  while (!Serial) ;
  
  pinMode(FOUND_PIN, INPUT_PULLUP);
  pinMode(ZERO_PIN, OUTPUT);
  
  pinMode(ERROR_HIGH, OUTPUT);
  pinMode(ERROR_LOW, OUTPUT);
  digitalWrite(ERROR_HIGH, LOW);
  digitalWrite(ERROR_LOW, LOW);  
}


void loop() {
  // If data has been received on the xbee
  if (Serial.available()) {
    // Read from serial and parse incoming data
    new_left_motor = Serial.parseInt();
    new_right_motor = Serial.parseInt();
    zero_value = Serial.parseInt();
    
    // Error check input
    if (new_left_motor != 0 && new_right_motor != 0 && (zero_value == 0 || zero_value == 1)) {
      left_motor_val = new_left_motor;
      right_motor_val = new_right_motor;
      digitalWrite(ERROR_HIGH, LOW);
    } else {
      // Bad input from xbee (you should reset rover and controller Unos)
      digitalWrite(ERROR_HIGH, HIGH);
      // Stop rover in event of an error
      left_motor_val = 255;
      right_motor_val = 255;
    }
    
    // Adjust the speed and direction of the right and left motors
    controlMotor(right_motor_val, 1);
    controlMotor(left_motor_val, 2);

    // Pass zeroing instructions to the metal detector Uno
    // but only send instructions when a value is received from the controller
    digitalWrite(ZERO_PIN, !zero_value);
    
    // Send metal detector found status to controller
    Serial.println(!digitalRead(FOUND_PIN));
  }
}


// Control the speed and direction of a motor
// \param motor_val the raw motor value received from controller
// \param motor which motor to control
// 1 corresponds to the right motor, 2 corresponds to the left motor
void controlMotor(int motor_val, int motor) {
  // Motor values are received as [0, 510 values]
  motor_val = motor_val - 255;

  if (motor_val > 20) {
    if (motor == 1) {
      right_motor->run(FORWARD);
      right_motor->setSpeed(motor_val);
    } else {
      left_motor->run(FORWARD);
      left_motor->setSpeed(motor_val);
    }
  } else if (motor_val < -20) {
    if (motor == 1) {
      right_motor->run(BACKWARD);
      right_motor->setSpeed((-1)*motor_val);
    } else {
      left_motor->run(BACKWARD);
      left_motor->setSpeed((-1)*motor_val);
    }
  } else {
    if (motor == 1) {
      right_motor->run(RELEASE);
    } else {
      left_motor->run(RELEASE);
    }
  }
}
