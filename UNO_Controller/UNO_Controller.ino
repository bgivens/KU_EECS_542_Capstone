//Arduino uno transmit
#include <SoftwareSerial.h>

//Declaring software serial pins for XBee transmission
SoftwareSerial mySerial(10,11);

//Defining and initializing button variables and pins

int zero_button = 4;
int zero_button_val = 1;

//Defining and initializing variables for obtaining joystick data
int joystick_right_vertical_pin = A1;
int joystick_left_vertical_pin = A0;

//int joystick_right_horizontal_data = 0;
int joystick_right_vertical_data = 0;

//int joystick_left_horizontal_data = 0;
int joystick_left_vertical_data = 0;


void setup() {
  // put your setup code here, to run once:
  
  //Starting Software serial for XBee transmission
  mySerial.begin(9600);
  
  //Initializing hardware serial for serial monitor debugging
  Serial.begin(9600);
  
  //Configuring pins used for button input as inputs with pull-up resistors
  // Value of 1 -> Button is not pressed, Value of 0 -> Button is pressed
  pinMode(zero_button, INPUT_PULLUP);
}

void loop() {
  //
  //Obtaining button values
  //
  //zero_button_val = digitalRead(button1_pin);
  
  //
  //  Obtaining analog joystick data
  //
  joystick_left_vertical_data = analogRead(joystick_left_vertical_pin);
  joystick_right_vertical_data = analogRead(joystick_right_vertical_pin);
  
  //
  // Debugging prints to verify component functionality
  //
  
  //Printing joystick values for debugging
  //TODO: Current issues with negative values being received as negative on the rover side likely result from send issues on controller side
  //      Need to verify that the correct data is being sent to begin with, and if not, it's likey an issue with how the controller is wired up
  //      If the data is being sent fine on this side, then the rover side is likely treating the received bytes as signed values, when they should be unsigned.
  Serial.print("Left Joystick Vertical Axis Data: ");
  Serial.println(joystick_left_vertical_data);
  Serial.print("Right Joystick Vertical Axis Data: ");
  Serial.println(joystick_right_vertical_data);
  
  //Sending data to receiver through XBee every 100ms
  mySerial.write(map(joystick_right_vertical_data, 0, 1023, 0, 255));
  mySerial.write(map(joystick_left_vertical_data, 0, 1023, 0, 255));
  //TODO: Send button data
  
  delay(100);
}
