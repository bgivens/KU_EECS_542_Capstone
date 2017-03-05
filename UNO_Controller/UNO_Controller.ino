//Arduino uno transmit
#include <SoftwareSerial.h>

#define ZERO_PIN 12
#define SERIAL_DEBUG TRUE
#define BAUDRATE 9600

//Declaring software serial pins for XBee transmission
SoftwareSerial xbee(10, 11); // TX RX

//Variable for storing state of button that zeros the metal detector
int zero_button_val = 1;

//Defining and initializing variables for obtaining joystick data
int joystick_right_vertical_pin = A1;
int joystick_left_vertical_pin = A0;

//int joystick_right_horizontal_data = 0;
int joystick_right_vertical_data = 0;

//int joystick_left_horizontal_data = 0;
int joystick_left_vertical_data = 0;


void setup() {
  //Starting Software serial for XBee transmission
  xbee.begin(BAUDRATE);

  #if SERIAL_DEBUG
  //Initializing hardware serial for serial monitor debugging
  Serial.begin(BAUDRATE);
  #endif

  //Configuring pins used for button input as inputs with pull-up resistors
  // Value of 1 -> Button is not pressed, Value of 0 -> Button is pressed
  pinMode(ZERO_PIN, INPUT_PULLUP);
}


void loop() {
  //Read analog joystick data
  joystick_left_vertical_data = analogRead(joystick_left_vertical_pin);
  joystick_right_vertical_data = analogRead(joystick_right_vertical_pin);

  //Read zero button
  zero_button_val = digitalRead(ZERO_PIN);

  //Printing joystick values for debugging
  //TODO: Current issues with negative values being received as negative on the rover side likely result from send issues on controller side
  //      Need to verify that the correct data is being sent to begin with, and if not, it's likey an issue with how the controller is wired up
  //      If the data is being sent fine on this side, then the rover side is likely treating the received bytes as signed values, when they should be unsigned.
  #if SERIAL_DEBUG
  Serial.print("Left Joystick Vertical Axis Data: ");
  Serial.println(joystick_left_vertical_data);
  Serial.print("Right Joystick Vertical Axis Data: ");
  Serial.println(joystick_right_vertical_data);
  Serial.print("Reset Button Data: ");
  Serial.println(zero_button_val);
  #endif

  //Sending data to receiver through XBee every 100ms
  xbee.write(map(joystick_right_vertical_data, 0, 1023, 0, 255));
  xbee.write(map(joystick_left_vertical_data, 0, 1023, 0, 255));
  //TODO: Send button data
  xbee.write(zero_button_val);

  delay(100);
}
