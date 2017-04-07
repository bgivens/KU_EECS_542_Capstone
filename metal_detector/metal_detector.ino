/*
 * Copyright (c) 2016 Evan Kale
 * Media: @EvanKale91
 * Email: EvanKale91@gmail.com
 * Website: www.ISeeDeadPixel.com
 *          www.evankale.blogspot.ca
 *          www.youtube.com/EvanKale91
 *
 * This file is part of ArduinoMetalDetector.
 *
 * ArduinoMetalDetector is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 /*
 * Modified by: Daniel Norman
 */

#define SERIAL_DEBUG 1

// Number of cycles from external counter needed to generate a signal event
#define CYCLES_PER_SIGNAL 5000

// Base tone frequency (speaker)
#define BASE_TONE_FREQUENCY 280

// Frequency delta threshold for fancy spinner to trigger
#define MARKING_THRESHOLD 400

// Pin definitions
#define SENSITIVITY_POT_APIN 1
#define SPEAKER_PIN 2
#define MARKING_PIN 8
#define TRIGGER_BTN_PIN 11
#define RESET_BTN_PIN 12
#define RESET_BTN_OUTPUT 13
#define BAUDRATE 9600

//TODO: determine serial pins
SoftwareSerial rover(9, 10); // RX, TX
int found = 0; //boolean, whether a mine is found
int zero_value = 0; //boolean, zero detector when 1
int i_sens;
float sensitivity = 10.0;

unsigned long lastSignalTime = 0;
unsigned long signalTimeDelta = 0;

boolean firstSignal = true;
unsigned long storedTimeDelta = 0;

// This signal is called whenever OCR1A reaches 0
// (Note: OCR1A is decremented on every external clock cycle)
SIGNAL(TIMER1_COMPA_vect) {
  unsigned long currentTime = micros();
  signalTimeDelta =  currentTime - lastSignalTime;
  lastSignalTime = currentTime;

  if (firstSignal) {
    firstSignal = false;
  } else if (storedTimeDelta == 0) {
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

  rover.begin(BAUDRATE);

  #if SERIAL_DEBUG
  Serial.begin(9600);
  Serial.println("Beginning");
  #endif
}

void loop() {
  if (rover.available()) {
    // Parse values from rover
    zero_value = xbee.parseInt();
    i_sens = rover.parseInt();

    if (zero_value) {
      // Reset metal detector when instructed
      storedTimeDelta = 0;
    }
    // Convert [0, 1023] sensitivity to [0.5, 10.0] for calculations
    mapFloat(i_sens, 0, 1023, 0.5, 10.0);
  }

  //if (digitalRead(TRIGGER_BTN_PIN) == LOW)
  //{
    //float sensitivity = mapFloat(analogRead(SENSITIVITY_POT_APIN), 0, 1023, 0.5, 10.0);

    int storedTimeDeltaDifference = (storedTimeDelta - signalTimeDelta) * sensitivity;
    // tone(SPEAKER_PIN, BASE_TONE_FREQUENCY + storedTimeDeltaDifference);

    Serial.println(storedTimeDeltaDifference);

    if (storedTimeDeltaDifference > MARKING_THRESHOLD) {
      // Currently detecting a mine
      found = 1;
    } else {
      // Not detecting a mine
      found = 0;
    }

  // TODO: Send found status back to rover

  //}
  //else
  //{
    //noTone(SPEAKER_PIN);
    //digitalWrite(MARKING_PIN, LOW);
  //}

  // if (digitalRead(RESET_BTN_PIN) == LOW) {
  //   storedTimeDelta = 0;
  //   digitalWrite(RESET_BTN_OUTPUT, HIGH);
  // }
  // digitalWrite(RESET_BTN_OUTPUT, LOW);
}

float mapFloat(int input, int inMin, int inMax, float outMin, float outMax) {
  float scale = (float)(input - inMin) / (inMax - inMin);
  return ((outMax - outMin) * scale) + outMin;
}
