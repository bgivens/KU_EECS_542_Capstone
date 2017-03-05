//Motor Imports & Variables
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <SoftwareSerial.h>
//#include <Servo.h>

#include "utility/Adafruit_MS_PWMServoDriver.h"

#define SERIAL_DEBUG FALSE

// Number of cycles from external counter needed to generate a signal event
#define CYCLES_PER_SIGNAL 5000

// Frequency delta threshold for triggering
#define MARKING_THRESHOLD 600

// Pin definitions
#define SENSITIVITY_POT_APIN 1
#define SPEAKER_PIN 2
#define MARKING_PIN 8
#define TRIGGER_BTN_PIN 11
#define RESET_BTN_PIN 12
#define RESET_BTN_OUTPUT 13

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *right_motor = AFMS.getMotor(1);
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);

byte commandByte1;
byte commandByte2;
byte commandByte3;
SoftwareSerial xbee(10, 11); // RX, TX
//Servo spraycan;

unsigned long lastSignalTime = 0;
unsigned long signalTimeDelta = 0;

boolean firstSignal = true;
unsigned long storedTimeDelta = 0;
int zero_value = 1;

// This signal is called whenever OCR1A reaches 0
// (Note: OCR1A is decremented on every external clock cycle)
SIGNAL(TIMER1_COMPA_vect)
{
  unsigned long currentTime = micros();
  signalTimeDelta = currentTime - lastSignalTime;
  lastSignalTime = currentTime;

  if (firstSignal) {
    firstSignal = false;
  }
  else if (storedTimeDelta == 0) {
    storedTimeDelta = signalTimeDelta;
  }

  // Reset OCR1A
  OCR1A += CYCLES_PER_SIGNAL;
}


void setup() {

  // Set WGM(Waveform Generation Mode) to 0 (Normal)
  TCCR1A = 0b00000000;

  // Set CSS(Clock Speed Selection) to 0b111 (External clock source on T0 pin
  // (ie, pin 5 on UNO). Clock on rising edge.)
  TCCR1B = 0b00000111;

  // Enable timer compare interrupt A (ie, SIGNAL(TIMER1_COMPA_VECT))
  TIMSK1 |= (1 << OCIE1A);

  // Set OCR1A (timer A counter) to 1 to trigger interrupt on next cycle
  OCR1A = 1;

  pinMode(SPEAKER_PIN, OUTPUT);
  pinMode(MARKING_PIN, OUTPUT);
  pinMode(TRIGGER_BTN_PIN, INPUT_PULLUP);
  pinMode(RESET_BTN_PIN, INPUT_PULLUP);
  pinMode(RESET_BTN_OUTPUT, OUTPUT);

  Serial.begin(57600);
  AFMS.begin();  // create with the default frequency 1.6KHz

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  // set the data rate for the SoftwareSerial port
  xbee.begin(9600);

  //spraycan.attach(13);
  //spraycan.write(15);
  right_motor->run(RELEASE);
  left_motor->run(RELEASE);
}

void loop() {
  //Xbee Serial Read
  if (xbee.available() > 0) {
    commandByte1 = xbee.read();

    #if SERIAL_DEBUG
    Serial.println("Left:");
    Serial.println(commandByte1);
    #endif
  }
  int left_motor_value = (int) commandByte1;

  //Read right stick value
  if (xbee.available() > 0) {
    // Read our command byte
    commandByte2 = xbee.read();
  }
  int right_motor_value = (int) commandByte2;

  if (xbee.available() > 0) {
    // Read our command byte
    commandByte3 = xbee.read();
  }
  int zero_value = (int) commandByte3;

  //Metal Detector
  float sensitivity = mapFloat(analogRead(SENSITIVITY_POT_APIN), 0, 1023, 0.5, 10.0);
  int storedTimeDeltaDifference = (storedTimeDelta - signalTimeDelta) * sensitivity;

  #if SERIAL_DEBUG
  Serial.print("Signal Time Delta: ");
  Serial.println(storedTimeDeltaDifference);
  #endif

  if (storedTimeDeltaDifference > 1000) {
    Serial.print("Stored Time Delta: ");
    Serial.println(storedTimeDeltaDifference);

    #if SERIAL_DEBUG
    Serial.print("Signal Time Delta: ");
    Serial.println(signalTimeDelta);
    #endif
  }

  //if(storedTimeDeltaDifference > MARKING_THRESHOLD && storedTimeDeltaDifference < 6000)
  if (storedTimeDeltaDifference > MARKING_THRESHOLD) {
    digitalWrite(MARKING_PIN, HIGH);
  } else {
    digitalWrite(MARKING_PIN, LOW);
  }

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
