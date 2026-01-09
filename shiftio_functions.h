#pragma once
#include <Arduino.h>

void exioShiftInBytes(uint8_t clk, uint8_t latch, uint8_t data, uint8_t* out, uint8_t nBytes);
void exioShiftOutBytes(uint8_t clk, uint8_t latch, uint8_t data, const uint8_t* in, uint8_t nBytes);