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

#include <Arduino.h>

#include "globals.h"
#include "tcp_functions.h"

#include "display_functions.h"
#include "pin_io_functions.h"
#include "shiftio_functions.h"

// Some Arduino toolchains don't define this; make __has_include safe.
#ifndef __has_include
  #define __has_include(x) 0
#endif

// ------------------------- Transport selection ------------------------------
//
// Priority order:
//   1) Ethernet
//   2) ESP8266 native WiFi (ESP-13)
//   3) WiFiS3 (UNO R4 WiFi, Portenta C33)
//   4) Generic WiFi.h
//

#ifndef __has_include
  #define __has_include(x) 0
#endif

// ---------- Ethernet FIRST ----------
#if defined(ENABLE_ETHERNET) && (ENABLE_ETHERNET == true)
  #if __has_include(<Ethernet.h>)
    #include <Ethernet.h>
    #define EXIO_TCP_ETH 1
  #endif
#endif

// ---------- WiFi fallback ----------
#if !defined(EXIO_TCP_ETH) && defined(ENABLE_WIFI) && (ENABLE_WIFI == true)

  // ESP8266 (ESP-13)
  #if defined(ARDUINO_ARCH_ESP8266)
    #include <ESP8266WiFi.h>
    #define EXIO_TCP_WIFI 1

  // WiFiS3 (UNO R4 WiFi, Portenta C33)
  #elif __has_include(<WiFiS3.h>)
    #include <WiFiS3.h>
    #define EXIO_TCP_WIFI 1

  // Classic WiFi (ESP32, SAMD, etc.)
  #elif __has_include(<WiFi.h>)
    #include <WiFi.h>
    #define EXIO_TCP_WIFI 1
  #endif

#endif

// ------------------------- Server/client objects ----------------------------

#if defined(EXIO_TCP_WIFI)
  static WiFiServer  _server(IP_PORT);
  static WiFiClient  _client;
#elif defined(EXIO_TCP_ETH)
  static EthernetServer _server(IP_PORT);
  static EthernetClient _client;
#endif

// ------------------------- State mirroring I2C ------------------------------

static uint8_t outboundFlag = 0;
static byte commandBuffer[3];

static const uint8_t EXIO_SHIFT_MAX_BYTES = 16;
static byte responseBuffer[1 + EXIO_SHIFT_MAX_BYTES]; // [status][payload...]
static uint8_t responseLength = 0;

// ------------------------- Small portability yield --------------------------

static inline void _tinyYield() {
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  yield();
#else
  // AVR etc: keep it minimal
  delay(0);
#endif
}

// ------------------------- Framing helpers ----------------------------------
// Request:  [CMD][LEN][PAYLOAD...]
// Response: [RCMD][RLEN][PAYLOAD...]

static bool _readExact(Stream& s, uint8_t* buf, size_t len, uint32_t timeoutMs) {
  uint32_t start = millis();
  size_t got = 0;

  while (got < len) {
    if ((millis() - start) > timeoutMs) return false;

    int avail = s.available();
    if (avail <= 0) { _tinyYield(); continue; }

    int c = s.read();
    if (c < 0) { _tinyYield(); continue; }

    buf[got++] = (uint8_t)c;
  }

  return true;
}

static void _writeFrame(Stream& s, uint8_t cmd, const uint8_t* payload, uint8_t len) {
  uint8_t hdr[2] = { cmd, len };
  s.write(hdr, 2);
  if (len && payload) s.write(payload, len);
}

static void _writeAck(Stream& s, uint8_t cmd, uint8_t statusByte) {
  _writeFrame(s, cmd, &statusByte, 1);
}

// ------------------------- Network bring-up ---------------------------------

bool tcpEnabled() {
#if defined(EXIO_TCP_WIFI) || defined(EXIO_TCP_ETH)
  return true;
#else
  return false;
#endif
}

#if defined(EXIO_TCP_WIFI)

static bool _wifiEnsureConnected() {
#ifdef DONT_TOUCH_WIFI_CONF
  // External system handles WiFi. We just assume it is connected.
  return (WiFi.status() == WL_CONNECTED);
#else
  if (WiFi.status() == WL_CONNECTED) return true;

  // ESP32 / ESP8266 only
  #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
    WiFi.mode(WIFI_STA);
  #endif

  // Hostname (best-effort; not all cores support this)
  #if defined(ARDUINO_ARCH_ESP32)
    if (strlen(WIFI_HOSTNAME) > 0) WiFi.setHostname(WIFI_HOSTNAME);
  #elif defined(ARDUINO_ARCH_ESP8266)
    if (strlen(WIFI_HOSTNAME) > 0) WiFi.hostname(WIFI_HOSTNAME);
  #endif
  // WiFiS3: no hostname API currently; ignore.

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if ((millis() - start) > WIFI_CONNECT_TIMEOUT) return false;
    delay(50);
  }

  return true;
#endif
}

#endif // EXIO_TCP_WIFI

#if defined(EXIO_TCP_ETH)

static void _ethBegin() {
  byte mac[] = MAC_ADDRESS;

  // If IP_ADDRESS is defined as an initializer (e.g. {192,168,1,50}) or IPAddress(...)
  // this will work as long as globals.h defines it appropriately.
  IPAddress ip = IP_ADDRESS;

  // Prefer DHCP, fallback to static
  int ok = Ethernet.begin(mac);
  if (ok == 0) {
    Ethernet.begin(mac, ip);
  }
  delay(250);
}

#endif // EXIO_TCP_ETH

// ------------------------- Protocol handlers --------------------------------

static void _handleCommand(Stream& s, uint8_t cmd, const uint8_t* payload, uint8_t len) {
  switch (cmd) {

    // EXIOINIT (TCP)
    // Request payload: [nPins][firstVpin_L][firstVpin_H]  len=3
    // Response: RCMD=EXIOPINS, payload=[EXIOPINS][numDigitalPins][numAnaloguePins]
    case EXIOINIT: {
      if (len != 3) {
        displayEventFlag = 2;
        uint8_t resp[3] = { 0, 0, 0 };
        _writeFrame(s, EXIOPINS, resp, 3);
        return;
      }

      initialisePins();

      numReceivedPins = payload[0];
      firstVpin = (uint16_t)((payload[2] << 8) | payload[1]);

      if (numReceivedPins == numPins) {
        displayEventFlag = 0;
        setupComplete = true;
      } else {
        displayEventFlag = 1;
        setupComplete = false;
      }

      outboundFlag = EXIOINIT;
      displayEvent = EXIOINIT;

      uint8_t resp[3] = { EXIOPINS, numDigitalPins, numAnaloguePins };
      _writeFrame(s, EXIOPINS, resp, 3);
      return;
    }

    case EXIOINITA: {
      outboundFlag = EXIOINITA;
      if (!setupComplete) {
        _writeFrame(s, EXIOINITA, nullptr, 0);
        return;
      }
      _writeFrame(s, EXIOINITA, analoguePinMap, (uint8_t)numAnaloguePins);
      return;
    }

    case EXIORDAN: {
      outboundFlag = EXIORDAN;
      if (!setupComplete) {
        _writeFrame(s, EXIORDAN, nullptr, 0);
        return;
      }
      _writeFrame(s, EXIORDAN, analoguePinStates, (uint8_t)analoguePinBytes);
      return;
    }

    case EXIORDD: {
      outboundFlag = EXIORDD;
      if (!setupComplete) {
        _writeFrame(s, EXIORDD, nullptr, 0);
        return;
      }
      _writeFrame(s, EXIORDD, digitalPinStates, (uint8_t)digitalPinBytes);
      return;
    }

    case EXIOVER: {
      outboundFlag = EXIOVER;
      _writeFrame(s, EXIOVER, versionBuffer, 3);
      return;
    }

    case EXIODPUP: {
      outboundFlag = EXIODPUP;
      if (len != 2) {
        displayEvent = EXIODPUP;
        _writeAck(s, EXIODPUP, EXIOERR);
        return;
      }
      uint8_t pin = payload[0];
      bool pullup = payload[1] ? true : false;
      bool ok = enableDigitalInput(pin, pullup);
      _writeAck(s, EXIODPUP, ok ? EXIORDY : EXIOERR);
      return;
    }

    case EXIOWRD: {
      outboundFlag = EXIOWRD;
      if (len != 2) {
        displayEvent = EXIOWRD;
        _writeAck(s, EXIOWRD, EXIOERR);
        return;
      }
      uint8_t pin = payload[0];
      bool state = payload[1] ? true : false;
      bool ok = writeDigitalOutput(pin, state);
      _writeAck(s, EXIOWRD, ok ? EXIORDY : EXIOERR);
      return;
    }

    case EXIOENAN: {
      outboundFlag = EXIOENAN;
      if (len != 1) {
        _writeAck(s, EXIOENAN, EXIOERR);
        return;
      }
      uint8_t pin = payload[0];
      bool ok = enableAnalogue(pin);
      _writeAck(s, EXIOENAN, ok ? EXIORDY : EXIOERR);
      return;
    }

    case EXIOWRAN: {
      outboundFlag = EXIOWRAN;
      if (len != 6) {
        displayEvent = EXIOWRAN;
        _writeAck(s, EXIOWRAN, EXIOERR);
        return;
      }
      uint8_t pin = payload[0];
      uint16_t value = (uint16_t)((payload[2] << 8) | payload[1]);
      uint8_t profile = payload[3];
      uint16_t duration = (uint16_t)((payload[5] << 8) | payload[4]);
      bool ok = writeAnalogue(pin, value, profile, duration);
      _writeAck(s, EXIOWRAN, ok ? EXIORDY : EXIOERR);
      return;
    }

    // SHIFTIN: [clk][latch][data][nBytes] -> response [EXIORDY][bytes...]
    case EXIOSHIFTIN: {
      outboundFlag = EXIOSHIFTIN;

      if (len != 4) {
        responseBuffer[0] = EXIOERR;
        responseLength = 1;
        _writeFrame(s, EXIOSHIFTIN, responseBuffer, responseLength);
        return;
      }

      uint8_t clk   = payload[0];
      uint8_t latch = payload[1];
      uint8_t data  = payload[2];
      uint8_t n     = payload[3];

      USB_SERIAL.print(F("TCP SHIFTIN clk="));
      USB_SERIAL.print(clk);
      USB_SERIAL.print(F(" latch="));
      USB_SERIAL.print(latch);
      USB_SERIAL.print(F(" data="));
      USB_SERIAL.print(data);
      USB_SERIAL.print(F(" n="));
      USB_SERIAL.println(n);

      if (n == 0 || n > EXIO_SHIFT_MAX_BYTES) {
        responseBuffer[0] = EXIOERR;
        responseLength = 1;
        _writeFrame(s, EXIOSHIFTIN, responseBuffer, responseLength);
        return;
      }

      responseBuffer[0] = EXIORDY;
      exioShiftInBytes(clk, latch, data, &responseBuffer[1], n);
      responseLength = (uint8_t)(1 + n);

      _writeFrame(s, EXIOSHIFTIN, responseBuffer, responseLength);
      return;
    }

    // SHIFTOUT: [clk][latch][data][nBytes][bytes...]
    case EXIOSHIFTOUT: {
      outboundFlag = EXIOSHIFTOUT;

      if (len < 4) {
        _writeAck(s, EXIOSHIFTOUT, EXIOERR);
        return;
      }

      uint8_t clk   = payload[0];
      uint8_t latch = payload[1];
      uint8_t data  = payload[2];
      uint8_t n     = payload[3];

      if (n == 0 || n > EXIO_SHIFT_MAX_BYTES) {
        _writeAck(s, EXIOSHIFTOUT, EXIOERR);
        return;
      }
      if (len != (uint8_t)(4 + n)) {
        _writeAck(s, EXIOSHIFTOUT, EXIOERR);
        return;
      }

      exioShiftOutBytes(clk, latch, data, &payload[4], n);

      _writeAck(s, EXIOSHIFTOUT, EXIORDY);
      return;
    }

    default:
      return;
  }
}

// ------------------------- Public API ---------------------------------------

void tcpBegin() {
#if defined(EXIO_TCP_WIFI)
  bool ok = _wifiEnsureConnected();
  if (!ok) {
    USB_SERIAL.println(F("TCP: WiFi connect failed"));
    return;
  }

  USB_SERIAL.print(F("TCP: WiFi connected, IP="));
  USB_SERIAL.println(WiFi.localIP());

  _server.begin();
  USB_SERIAL.print(F("TCP: Server listening on port "));
  USB_SERIAL.println(IP_PORT);

#elif defined(EXIO_TCP_ETH)
  _ethBegin();

  USB_SERIAL.print(F("TCP: Ethernet started, IP="));
  USB_SERIAL.println(Ethernet.localIP());

  _server.begin();
  USB_SERIAL.print(F("TCP: Server listening on port "));
  USB_SERIAL.println(IP_PORT);

#else
  USB_SERIAL.println(F("TCP: not enabled/available in this build"));
#endif
}

void tcpLoop() {
#if defined(EXIO_TCP_WIFI) || defined(EXIO_TCP_ETH)

  if (!_client || !_client.connected()) {
    _client = _server.available();
    if (_client && _client.connected()) {
      USB_SERIAL.println(F("TCP: client connected"));
    }
    return;
  }

  // Frame header: 2 bytes
  while (_client.available() >= 2) {
    uint8_t hdr[2];
    if (!_readExact(_client, hdr, 2, 25)) {
      _client.stop();
      USB_SERIAL.println(F("TCP: header timeout -> disconnect"));
      return;
    }

    uint8_t cmd = hdr[0];
    uint8_t len = hdr[1];

    // Keep small + stack-safe (matches your I2C 32-byte cap)
    uint8_t payload[32];
    if (len > sizeof(payload)) {
      // Drain and drop (protocol violation / attack / bug)
      for (uint8_t i = 0; i < len; i++) {
        if (_client.available()) (void)_client.read();
        else _tinyYield();
      }
      _client.stop();
      USB_SERIAL.println(F("TCP: frame too large -> disconnect"));
      return;
    }

    if (len > 0) {
      if (!_readExact(_client, payload, len, 100)) {
        _client.stop();
        USB_SERIAL.println(F("TCP: payload timeout -> disconnect"));
        return;
      }
      _handleCommand(_client, cmd, payload, len);
    } else {
      _handleCommand(_client, cmd, nullptr, 0);
    }

    _tinyYield();
  }

#endif
}

void tcpEnd() {
#if defined(EXIO_TCP_WIFI) || defined(EXIO_TCP_ETH)
  if (_client) _client.stop();
#endif
}
