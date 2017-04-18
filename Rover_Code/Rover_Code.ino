//Motor Imports & Variables
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <SoftwareSerial.h>
#include <Servo.h>
// #include "utility/Adafruit_MS_PWMServoDriver.h"

#define SERIAL_DEBUG 1

#define BAUDRATE 9600

// Pin definitions
#define SPRAYCAN_PIN 7
#define FOUND_PIN 9
#define ZERO_PIN 8

// Serial definitions
SoftwareSerial   xbee(10, 11); // RX, TX
//SoftwareSerial detector(8, 9); // RX, TX

// Initialize motor shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* right_motor = AFMS.getMotor(1);
Adafruit_DCMotor* left_motor = AFMS.getMotor(2);

// Variables
Servo spraycan;
int zero_value = 0;
int sensitivity; // TODO: default value for sensitivity?
int found = 0;
int left_motor_val;
int right_motor_val;
char buffer[20]; // Buffer for sending/receiving strings


void setup() {
  pinMode(SPRAYCAN_PIN, OUTPUT);

  AFMS.begin();  // create with the default frequency 1.6KHz
  right_motor->run(RELEASE);
  left_motor->run(RELEASE);
  // spraycan.attach(SPRAYCAN_PIN);

  #if SERIAL_DEBUG
  // Initialize hardware serial for serial monitor debugging
  Serial.begin(BAUDRATE);
  // Wait for serial port to connect. Needed for native USB port only
  while (!Serial) ;
  Serial.println("Rover debug connected");
  #endif

  // Set the data rate for the SoftwareSerial ports
  xbee.begin(BAUDRATE);
  //detector.begin(BAUDRATE);
  
  pinMode(FOUND_PIN, INPUT_PULLUP);
  pinMode(ZERO_PIN, OUTPUT);
  
  #if SERIAL_DEBUG
  Serial.println("Software serial ports connected");
  Serial.println("(left, right, zero, sens)");
  #endif
}


void loop() {
  // If data has been received on the xbee
  if (xbee.available()) {
    // Read from serial and parse incoming data
    left_motor_val  = xbee.parseInt();
    right_motor_val = xbee.parseInt();
    zero_value = xbee.parseInt();
    sensitivity = xbee.parseInt();
    
    // Print values for debugging
    #if SERIAL_DEBUG
//    Serial.print("(");
//    Serial.print(left_motor_val);
//    Serial.print(", ");
//    Serial.print(right_motor_val);
//    Serial.print(", ");
//    Serial.print(zero_value);
//    Serial.print(", ");
//    Serial.print(sensitivity);
//    Serial.print(")\n");
    #endif

    // Adjust the speed and direction of the right and left motors
    controlMotor(right_motor_val, 1);
    controlMotor(left_motor_val, 2);


    // Send sensitivity and zeroing instructions to the metal detector Uno
    // but only send instructions after another value is received from the controller
    if (zero_value == 1) { 
      digitalWrite(ZERO_PIN, LOW);
    } else {
      digitalWrite(ZERO_PIN, HIGH);
    }
  }

  // Read from metal detector Uno
  found = !digitalRead(FOUND_PIN);
  Serial.println(found);
  xbee.println(found);
}


// Control the speed and direction of a motor
// \param motor_val the raw motor value received from controller
// \param motor which motor to control
// 1 corresponds to the right motor, 2 corresponds to the left motor
void controlMotor(int motor_val, int motor) {
  // Motor values are received as [0, 510 values]
  motor_val = motor_val  - 255;

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
