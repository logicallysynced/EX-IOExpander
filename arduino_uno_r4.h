/*
 *  © 2026, DCC-EX contributors.
 *
 *  This file is part of EX-IOExpander.
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 */

#ifndef ARDUINO_UNO_R4_H
#define ARDUINO_UNO_R4_H

#include <Arduino.h>
#include "globals.h"

// Match the existing “Uno-style” mapping used by EX-IOExpander:
// - Digital: D2..D13 (12)
// - Analogue: A0..A3 (4)
// Total = 16 pins
//
// Notes for UNO R4:
// - A4/A5 are typically used as I2C SDA/SCL like classic Uno.
// - Using A0..A3 avoids collisions with I2C when in I2C mode.
// - For PWM capabilities, the core determines whether a given pin supports analogWrite().

pinDefinition pinMap[TOTAL_PINS] = {
  {2, DIO}, {3, DIOP}, {4, DIO}, {5, DIOP}, {6, DIOP}, {7, DIO},
  {8, DIO}, {9, DIOP}, {10, DIOP}, {11, DIOP}, {12, DIO}, {13, DIO},
  {A0, AIDIO}, {A1, AIDIO}, {A2, AIDIO}, {A3, AIDIO},
};

// I2C pins on UNO-form-factor boards
#ifndef I2C_SDA
  #define I2C_SDA A4
#endif
#ifndef I2C_SCL
  #define I2C_SCL A5
#endif

pinName pinNameMap[TOTAL_PINS] = {
  {2, "D2"}, {3, "D3"}, {4, "D4"}, {5, "D5"}, {6, "D6"}, {7, "D7"},
  {8, "D8"}, {9, "D9"}, {10, "D10"}, {11, "D11"}, {12, "D12"}, {13, "D13"},
  {A0, "A0"}, {A1, "A1"}, {A2, "A2"}, {A3, "A3"},
};

// Servo support (Arduino Servo library should work on UNO R4)
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;
Servo servo8;
Servo servo9;
Servo servo10;
Servo servo11;
Servo servo12;

Servo servoMap[MAX_SERVOS] = {
  servo1, servo2, servo3, servo4, servo5, servo6,
  servo7, servo8, servo9, servo10, servo11, servo12,
};

#endif
