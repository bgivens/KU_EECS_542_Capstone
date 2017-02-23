#include <Wire.h>
#include <Adafruit_MotorShield.h>

#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *right_motor = AFMS.getMotor(1);
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);

byte commandByte1; 
byte commandByte2; 
#include <SoftwareSerial.h>
#include <Servo.h>
SoftwareSerial mySerial(10, 11); // RX, TX
Servo scorpion;
int val;
byte rawData[2];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);           // set up Serial library at 9600 bps
  AFMS.begin();  // create with the default frequency 1.6KHz
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  mySerial.println("Bluetooth Test");
  // Set the speed to start, from 0 (off) to 255 (max speed)
  /*
  right_motor->setSpeed(150);
  right_motor->run(FORWARD);
  
  left_motor->setSpeed(150);
  left_motor->run(FORWARD);
  // turn on motor

  */
  
  scorpion.attach(13);
  
  scorpion.write(15);
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
      Serial.println("Left:");
      Serial.println(left_motor_value);
      Serial.println("Right:");
      Serial.println(right_motor_value);
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
        map(motor_speed, 0, 110, 150, 0);
        left_motor->run(BACKWARD);
        left_motor->setSpeed(motor_speed);
      }
      else if((left_motor_value > 140)
      {
        int motor_speed = left_motor_value;
        map(motor_speed, 140, 255, 0, 150);
        left_motor->run(FORWARD);
        left_motor->setSpeed(motor_speed);
      }   
       
//      if(left_motor_value < 20)
//      {
//        //Since motor speed is a value ranged from 0-255, map the range 1-127 to 1-255 to get variable speed 
//        int motor_speed = abs(left_motor_value);
//        map(motor_speed, 1, 127, 1, 255); 
//        left_motor->run(BACKWARD);
//        left_motor->setSpeed(motor_speed);
//      }
//      if(left_motor_value < 110 && left_motor_value > 60)
//      {
//        int motor_speed = 75;
//        left_motor->run(BACKWARD);
//        left_motor->setSpeed(motor_speed);
//      }
//      if(left_motor_value < 60 && left_motor_value > 20)
//      {
//        int motor_speed = 100;
//        left_motor->run(BACKWARD);
//        left_motor->setSpeed(motor_speed);
//      }
//      if(left_motor_value > 230)
//      {
//        int motor_speed = 150;
//        left_motor->run(FORWARD);
//        left_motor->setSpeed(motor_speed);
//      }
//      if(left_motor_value > 150 && left_motor_value < 200)
//      {
//        int motor_speed = 75;
//        left_motor->run(FORWARD);
//        left_motor->setSpeed(motor_speed);
//      }
//      if(left_motor_value > 200 && left_motor_value < 230)
//      {
//        int motor_speed = 100;
//        left_motor->run(FORWARD);
//        left_motor->setSpeed(motor_speed);
//      }
//      
      
      //******************
      //Right motor control
      //******************
      if((right_motor_value < 110) && (right_motor_value > 0))
      {
        int motor_speed = abs(right_motor_value);
        map(motor_speed, 0, 110, 150, 0);
        right_motor->run(BACKWARD);
        right_motor->setSpeed(motor_speed);
      }
      else if((right_motor_value > 140)
      {
        int motor_speed = abs(right_motor_value);
        map(motor_speed, 140, 255, 0, 150);
        right_motor->run(FORWARD);
        right_motor->setSpeed(motor_speed);
      }
      
//      if(right_motor_value < 20)
//      {
//        //Since motor speed is a value ranged from 0-255, map the range 1-127 to 1-255 to get variable speed 
//        int motor_speed = 150;
//        right_motor->run(BACKWARD);
//        right_motor->setSpeed(motor_speed);
//      }
//      if(right_motor_value < 110 && right_motor_value > 60)
//      {
//        int motor_speed = 75;
//        right_motor->run(BACKWARD);
//        right_motor->setSpeed(motor_speed);
//      }
//      if(right_motor_value < 60 && right_motor_value > 20)
//      {
//        int motor_speed = 100;
//        right_motor->run(BACKWARD);
//        right_motor->setSpeed(motor_speed);
//      }
//      if(right_motor_value > 230)
//      {
//        int motor_speed = 150;
//        right_motor->run(FORWARD);
//        right_motor->setSpeed(motor_speed);
//      }
//      if(right_motor_value > 150 && right_motor_value < 200)
//      {
//        int motor_speed = 75;
//        right_motor->run(FORWARD);
//        right_motor->setSpeed(motor_speed);
//      }
//      if(right_motor_value > 200 && right_motor_value < 230)
//      {
//        int motor_speed = 100;
//        right_motor->run(FORWARD);
//        right_motor->setSpeed(motor_speed);
//      }
        
      
}
