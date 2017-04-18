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
#define FOUND_PIN 9
#define ZERO_PIN 8
#define BAUDRATE 9600


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

  pinMode(FOUND_PIN, OUTPUT);
  pinMode(ZERO_PIN, INPUT_PULLUP);


  #if SERIAL_DEBUG
  Serial.begin(9600);
  Serial.println("Beginning Metal Detector");
  #endif
}

void loop() {
  //float sensitivity = mapFloat(analogRead(SENSITIVITY_POT_APIN), 0, 1023, 0.5, 10.0);
  
  int storedTimeDeltaDifference = (storedTimeDelta - signalTimeDelta) * sensitivity;
  // tone(SPEAKER_PIN, BASE_TONE_FREQUENCY + storedTimeDeltaDifference);

  #if SERIAL_DEBUG
  //Serial.println(storedTimeDeltaDifference);
  #endif
  
  // Determine if a mine is detected and send found status back to rover
  if (storedTimeDeltaDifference > MARKING_THRESHOLD) {
    // Currently detecting a mine
//    Serial.println(0);
    digitalWrite(FOUND_PIN, LOW);
  } else {
    // Not detecting a mine
//    Serial.println(1);
    digitalWrite(FOUND_PIN, HIGH);
  }

  // Reset metal detector when instructed
  if (digitalRead(ZERO_PIN) == LOW) {
    storedTimeDelta = 0;
//    Serial.println("zero");
    // digitalWrite(RESET_BTN_OUTPUT, HIGH);
  }
  // digitalWrite(RESET_BTN_OUTPUT, LOW);
}

float mapFloat(int input, int inMin, int inMax, float outMin, float outMax) {
  float scale = (float)(input - inMin) / (inMax - inMin);
  return ((outMax - outMin) * scale) + outMin;
}
