//Arduino uno transmit
#include <SoftwareSerial.h>

#define SERIAL_DEBUG 0
#define BAUDRATE 9600
#define ZERO_PIN 12
#define JS_R_PIN A1
#define JS_L_PIN A0
#define LED_PIN 9


// Declaring software serial pins for XBee transmission
SoftwareSerial xbee(10, 11); // TX RX

int joystick_left_vertical = 0;
int joystick_right_vertical = 0;
int zero_value = 0; // Used to zero the metal detector
int found = 0;


void setup() {
  // Starting Software serial for XBee transmission
  xbee.begin(BAUDRATE);

  #if SERIAL_DEBUG
  // Initialize hardware serial for serial monitor debugging
  Serial.begin(BAUDRATE);
  // Wait for serial port to connect. Needed for native USB port only
  while (!Serial) ;
  Serial.println("Controller debug connected");
  Serial.println("(left, right, zero, sens)");
  #endif

  // Configure pins used for button input as inputs with pull-up resistors
  // Value of 1 -> Button is not pressed, Value of 0 -> Button is pressed
  pinMode(ZERO_PIN, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);
}


void loop() {
  // Read joysticks and convert [0, 1023] values to [0, 510]
  joystick_left_vertical = map(analogRead(JS_L_PIN), 0, 1023, 0, 510);
  joystick_right_vertical = map(analogRead(JS_R_PIN), 0, 1023, 0, 510);
  // Read inputs
  zero_value = !digitalRead(ZERO_PIN);

  // Print values for debugging
  #if SERIAL_DEBUG
  Serial.print("(");
  Serial.print(joystick_left_vertical);
  Serial.print(", ");
  Serial.print(joystick_right_vertical);
  Serial.print(", ");
  Serial.print(zero_value);
  Serial.print(")\n");
  #endif

  // Send data to receiver through XBee every 100ms
  char buffer[20];
  sprintf(buffer, "%3d,%3d,%d", joystick_left_vertical, joystick_right_vertical, zero_value);
  xbee.println(buffer);

  // Read from rover Uno
  if (xbee.available()) {
    found = xbee.parseInt();
    digitalWrite(LED_PIN, found);
  }
  delay(100);
}
