
#include <Servo.h>


Servo spraycan;

void setup() {

  spraycan.attach(9);
  spraycan.write(180);
  delay(400);
  spraycan.write(100);  
  delay(250);
  spraycan.write(180);
}

void loop() {
  
}

