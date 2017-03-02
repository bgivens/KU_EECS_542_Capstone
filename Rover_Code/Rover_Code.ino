
//Motor Imports & Variables
#include <Wire.h>
#include <Adafruit_MotorShield.h>

#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *right_motor = AFMS.getMotor(1);
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);

byte commandByte1; 
byte commandByte2; 
#include <SoftwareSerial.h>
//#include <Servo.h>
SoftwareSerial mySerial(10, 11); // RX, TX
//Servo scorpion;
int val;
byte rawData[2];

//Metal Detector Imports & Variables


// Number of cycles from external counter needed to generate a signal event
#define CYCLES_PER_SIGNAL 5000

// Base tone frequency (speaker)
#define BASE_TONE_FREQUENCY 280

// Frequency delta threshold for fancy spinner to trigger
#define MARKING_THRESHOLD 600

// Pin definitions
#define SENSITIVITY_POT_APIN 1
#define SPEAKER_PIN 2
#define MARKING_PIN 8
#define TRIGGER_BTN_PIN 11
#define RESET_BTN_PIN 12
#define RESET_BTN_OUTPUT 13

unsigned long lastSignalTime = 0;
unsigned long signalTimeDelta = 0;

boolean firstSignal = true;
unsigned long storedTimeDelta = 0;

// This signal is called whenever OCR1A reaches 0
// (Note: OCR1A is decremented on every external clock cycle)
SIGNAL(TIMER1_COMPA_vect)
{
  unsigned long currentTime = micros();
  signalTimeDelta =  currentTime - lastSignalTime;
  lastSignalTime = currentTime;

  if (firstSignal)
  {
    firstSignal = false;
  }
  else if (storedTimeDelta == 0)
  {
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
  mySerial.begin(9600);
  
  //scorpion.attach(13);
  
  //scorpion.write(15);
  right_motor->run(RELEASE);
  left_motor->run(RELEASE);
}

void loop() {

      //************************
      // XBee Communication
      //************************
      
      //Read left stick value
      if (mySerial.available() > 0) 
      {
        commandByte1 = mySerial.read();
        //Serial.println("Left:");
        //Serial.println(commandByte1);
      }
      int left_motor_value = (int) commandByte1;
      //Read right stick value
      if (mySerial.available() > 0) 
      {
        // Read our command byte
        commandByte2 = mySerial.read();
      }
      int right_motor_value = (int) commandByte2;
      
      //If both values are zero, release motors
      //&& (right_motor_value < 150 && right_motor_value > 100  right_motor->run(RELEASE);
      if(left_motor_value > 110 && left_motor_value < 150) 
      {
        left_motor->run(RELEASE); 
      }
      if(right_motor_value > 110 && right_motor_value < 150)
      {
        right_motor->run(RELEASE); 
      }
     
      //******************
      //Left motor control
      //******************
      
      //If value is less than zero, move backwards
      if((left_motor_value < 110) && (left_motor_value > 0))
      {
        //Since motor speed is a value ranged from 0-255, map the range 1-127 to 1-255 to get variable speed 
        int motor_speed = left_motor_value;
        motor_speed = map(motor_speed, 0, 110, 150, 0);
        left_motor->run(BACKWARD);
        left_motor->setSpeed(motor_speed);
      }
      else if(left_motor_value > 140)
      {
        int motor_speed = left_motor_value;
        motor_speed = map(motor_speed, 140, 255, 0, 150);
        left_motor->run(FORWARD);
        left_motor->setSpeed(motor_speed);
      }   
  
      //******************
      //Right motor control
      //******************
      if((right_motor_value < 110) && (right_motor_value > 0))
      {
        int motor_speed = abs(right_motor_value);
        motor_speed = map(motor_speed, 0, 110, 150, 0);
        right_motor->run(BACKWARD);
        right_motor->setSpeed(motor_speed);
      }
      else if(right_motor_value > 140)
      {
        int motor_speed = abs(right_motor_value);
        motor_speed = map(motor_speed, 140, 255, 0, 150);
        right_motor->run(FORWARD);
        right_motor->setSpeed(motor_speed);
      }
      //if (digitalRead(TRIGGER_BTN_PIN) == LOW)
      //{
        //float sensitivity = mapFloat(analogRead(SENSITIVITY_POT_APIN), 0, 1023, 0.5, 10.0);
        float sensitivity = 8.0;
        int storedTimeDeltaDifference = (storedTimeDelta - signalTimeDelta) * sensitivity;
        tone(SPEAKER_PIN, BASE_TONE_FREQUENCY + storedTimeDeltaDifference);
        
        Serial.println(storedTimeDeltaDifference);
        
        if (storedTimeDeltaDifference > MARKING_THRESHOLD && storedTimeDeltaDifference < 1000)
        {
          digitalWrite(MARKING_PIN, HIGH);
        }
        else
        {
          digitalWrite(MARKING_PIN, LOW);
        }
      //}
      //else
      //{
        //noTone(SPEAKER_PIN);
        //digitalWrite(MARKING_PIN, LOW);
      //}
      if (digitalRead(RESET_BTN_PIN) == LOW)
      {
        storedTimeDelta = 0;
        digitalWrite(RESET_BTN_OUTPUT, HIGH);
      }
      
      digitalWrite(RESET_BTN_OUTPUT, LOW);
      
}


float mapFloat(int input, int inMin, int inMax, float outMin, float outMax)
{
  float scale = (float)(input - inMin) / (inMax - inMin);
  return ((outMax - outMin) * scale) + outMin;
}
