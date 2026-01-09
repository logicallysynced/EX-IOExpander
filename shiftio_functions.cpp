#include <Arduino.h>
#include "shiftio_functions.h"
#include "globals.h"   // pinMap[], pinNameMap[], numPins

static bool mapPins(uint8_t clk, uint8_t latch, uint8_t data,
                    uint8_t &pClk, uint8_t &pLatch, uint8_t &pData) {
  if (clk >= numPins || latch >= numPins || data >= numPins) return false;
  pClk   = pinMap[clk].physicalPin;
  pLatch = pinMap[latch].physicalPin;
  pData  = pinMap[data].physicalPin;
  return true;
}

static uint8_t shiftInByte_phys(uint8_t pClk, uint8_t pLatch, uint8_t pData) {
  uint8_t value = 0;

  digitalWrite(pLatch, HIGH);
  delayMicroseconds(1);
  digitalWrite(pLatch, LOW);

  for (int i = 0; i < 8; i++) {
    digitalWrite(pClk, LOW);
    delayMicroseconds(1);

    if (digitalRead(pData)) value |= (1 << (7 - i));

    digitalWrite(pClk, HIGH);
    delayMicroseconds(1);
  }
  return value;
}

static void shiftOutByte_phys(uint8_t pClk, uint8_t pData, uint8_t value) {
  for (int i = 7; i >= 0; i--) {
    digitalWrite(pData, (value >> i) & 0x01);
    digitalWrite(pClk, HIGH);
    delayMicroseconds(1);
    digitalWrite(pClk, LOW);
  }
}

void exioShiftInBytes(uint8_t clk, uint8_t latch, uint8_t data, uint8_t* out, uint8_t nBytes) {
  uint8_t pClk, pLatch, pData;
  if (!mapPins(clk, latch, data, pClk, pLatch, pData)) {
    // fail-safe: return all 0xFF so you can spot it (or all 0x00, your choice)
    for (uint8_t i = 0; i < nBytes; i++) out[i] = 0xFF;
    return;
  }

  pinMode(pClk, OUTPUT);
  pinMode(pLatch, OUTPUT);
  pinMode(pData, INPUT_PULLUP);
  digitalWrite(pClk, LOW);
  digitalWrite(pLatch, LOW);

  USB_SERIAL.print(F("SHIFTIN idx "));
  USB_SERIAL.print(clk); USB_SERIAL.print("/");
  USB_SERIAL.print(latch); USB_SERIAL.print("/");
  USB_SERIAL.print(data); USB_SERIAL.print(F(" => phys "));
  USB_SERIAL.print(pClk); USB_SERIAL.print("/");
  USB_SERIAL.print(pLatch); USB_SERIAL.print("/");
  USB_SERIAL.println(pData);

  for (uint8_t i = 0; i < nBytes; i++) {
    out[i] = shiftInByte_phys(pClk, pLatch, pData);
  }
}

void exioShiftOutBytes(uint8_t clk, uint8_t latch, uint8_t data, const uint8_t* in, uint8_t nBytes) {
  uint8_t pClk, pLatch, pData;
  if (!mapPins(clk, latch, data, pClk, pLatch, pData)) return;

  pinMode(pClk, OUTPUT);
  pinMode(pLatch, OUTPUT);
  pinMode(pData, OUTPUT);
  digitalWrite(pClk, LOW);
  digitalWrite(pLatch, LOW);

  for (int i = (int)nBytes - 1; i >= 0; i--) {
    shiftOutByte_phys(pClk, pData, in[i]);
  }

  digitalWrite(pLatch, HIGH);
}
