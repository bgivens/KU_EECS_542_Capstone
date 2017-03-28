//Arduino uno transmit
#include <SoftwareSerial.h>

#define SERIAL_DEBUG 0
#define BAUDRATE 9600
#define ZERO_PIN 12
#define JS_R_PIN A1
#define JS_L_PIN A0
// TODO: determine the following pins
#define SENS_PIN 10
#define LED_PIN 9


// Declaring software serial pins for XBee transmission
SoftwareSerial xbee(10, 11); // TX RX

int zero_value = 1; // Used to zero the metal detector, active low
int sensitivity; // Sensitivity of metal detector
int joystick_right_vertical = 0;
int joystick_left_vertical = 0;
int found = 0;


void setup() {
  // Starting Software serial for XBee transmission
  xbee.begin(BAUDRATE);

  #if SERIAL_DEBUG
  // Initializing hardware serial for serial monitor debugging
  Serial.begin(BAUDRATE);
  while (!Serial) ;
  #endif

  // Configure pins used for button input as inputs with pull-up resistors
  // Value of 1 -> Button is not pressed, Value of 0 -> Button is pressed
  pinMode(ZERO_PIN, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);
}


void loop() {
  // Read inputs
  zero_value = !digitalRead(ZERO_PIN);
  sensitivity = analogRead(SENS_PIN);
  joystick_left_vertical = analogRead(JS_L_PIN);
  joystick_right_vertical = analogRead(JS_R_PIN);

  // Print values for debugging
  #if SERIAL_DEBUG
  Serial.print("Left Joystick Vertical Axis Data: ");
  Serial.println(joystick_left_vertical);
  Serial.print("Right Joystick Vertical Axis Data: ");
  Serial.println(joystick_right_vertical);
  Serial.print("Zero Button: ");
  Serial.println(zero_value);
  Serial.print("Sensitivity: ");
  Serial.println(sensitivity);
  #endif

  // Send data to receiver through XBee every 100ms
  char buffer[30];
  sprintf(buffer, "%d,%d,%d,%d", joystick_left_vertical, joystick_right_vertical, zero_value, sensitivity);
  xbee.println(buffer);

  // Read from rover Uno
  if (xbee.available()) {
    found = xbee.parseInt();
    digitalWrite(LED_PIN, found);
  }
  delay(100);
}
