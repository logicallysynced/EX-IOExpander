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
// Priority order:
//   1) Ethernet
//   2) WiFi (board-specific first, then generic)
//

// ---------- Ethernet FIRST ----------
#if defined(ENABLE_ETHERNET) && (ENABLE_ETHERNET == true)
  #if __has_include(<Ethernet.h>)
    #include <Ethernet.h>
    #define EXIO_TCP_ETH 1
  #endif
#endif

// ---------- WiFi fallback ----------
#if !defined(EXIO_TCP_ETH) && defined(ENABLE_WIFI) && (ENABLE_WIFI == true)

  // UNO R4 WiFi / Portenta C33 / other WiFiS3 boards
  #if defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_PORTENTA_C33)
    #include <WiFiS3.h>
    #define EXIO_TCP_WIFI 1

  // ESP8266
  #elif defined(ARDUINO_ARCH_ESP8266)
    #include <ESP8266WiFi.h>
    #define EXIO_TCP_WIFI 1

  // ESP32 / classic WiFi.h
  #elif defined(ARDUINO_ARCH_ESP32)
    #include <WiFi.h>
    #define EXIO_TCP_WIFI 1

  // “Best effort” generic fallback if a core provides one of these
  #elif __has_include(<WiFiS3.h>)
    #include <WiFiS3.h>
    #define EXIO_TCP_WIFI 1
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

// ------------------------- Latency stats ------------------------------------

static uint32_t _tcpFrameCount = 0;
static uint32_t _tcpLastLatencyUs = 0;
static uint32_t _tcpMaxLatencyUs = 0;
static uint64_t _tcpTotalLatencyUs = 0;

// ------------------------- State mirroring I2C ------------------------------

static uint8_t outboundFlag = 0;

// IMPORTANT: do NOT define numReceivedPins here.
// It is already defined in i2c_functions.cpp, and should be declared extern in globals.h.
// extern uint8_t numReceivedPins;  <-- in globals.h

static const uint8_t EXIO_SHIFT_MAX_BYTES = 16;
static byte responseBuffer[1 + EXIO_SHIFT_MAX_BYTES]; // [status][payload...]
static uint8_t responseLength = 0;

// ------------------------- Small portability yield --------------------------

static inline void _tinyYield() {
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  yield();
#else
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
  // If user says "don't touch", only honor that on ESP cores where an external
  // sketch/framework might already be managing WiFi. On WiFiS3 boards, you
  // generally need to call WiFi.begin() yourself.
#if defined(DONT_TOUCH_WIFI_CONF) && (defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266))
  return (WiFi.status() == WL_CONNECTED);
#else
  if (WiFi.status() == WL_CONNECTED) return true;

  // STA mode where supported
  #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
    WiFi.mode(WIFI_STA);
  #endif

  // Hostname (only where supported)
  #if defined(ARDUINO_ARCH_ESP32)
    if (strlen(WIFI_HOSTNAME) > 0) WiFi.setHostname(WIFI_HOSTNAME);
  #elif defined(ARDUINO_ARCH_ESP8266)
    if (strlen(WIFI_HOSTNAME) > 0) WiFi.hostname(WIFI_HOSTNAME);
  #endif
  // WiFiS3: no hostname API (ignore)

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if ((millis() - start) > WIFI_CONNECT_TIMEOUT) return false;
    delay(100);
  }

  // Give DHCP a moment to populate localIP() on some stacks
  for (uint8_t i = 0; i < 20; i++) {
    IPAddress ip = WiFi.localIP();
    if (ip[0] != 0 || ip[1] != 0 || ip[2] != 0 || ip[3] != 0) return true;
    delay(100);
  }

  return true;
#endif
}

#endif  // EXIO_TCP_WIFI

#if defined(EXIO_TCP_ETH)

/*************  ✨ Windsurf Command ⭐  *************/
/**
 * @brief Initialises the Ethernet interface.
 *
 * This function is called when the TCP server is started. It sets up the
 * Ethernet interface using the MAC address and IP address defined in
 * the globals.h file.
 *
 * @note Ethernet.begin() is called with the MAC address only, and if this
 * call fails, the function will call Ethernet.begin() again with both the MAC
 * address and IP address.
 *
 * @see Ethernet.begin()
 */
/*******  afc1f365-e1a5-40a8-82ec-42a435222ec7  *******/
static void _ethBegin() {
  byte mac[] = MAC_ADDRESS;

  // Globals should define IP_ADDRESS as either IPAddress(...) or {a,b,c,d} style.
  IPAddress ip = IP_ADDRESS;

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
      if (!setupComplete) { _writeFrame(s, EXIOINITA, nullptr, 0); return; }
      _writeFrame(s, EXIOINITA, analoguePinMap, (uint8_t)numAnaloguePins);
      return;
    }

    case EXIORDAN: {
      outboundFlag = EXIORDAN;
      if (!setupComplete) { _writeFrame(s, EXIORDAN, nullptr, 0); return; }
      _writeFrame(s, EXIORDAN, analoguePinStates, (uint8_t)analoguePinBytes);
      return;
    }

    case EXIORDD: {
      outboundFlag = EXIORDD;
      if (!setupComplete) { _writeFrame(s, EXIORDD, nullptr, 0); return; }
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
      if (len != 2) { displayEvent = EXIODPUP; _writeAck(s, EXIODPUP, EXIOERR); return; }
      uint8_t pin = payload[0];
      bool pullup = payload[1] ? true : false;
      bool ok = enableDigitalInput(pin, pullup);
      _writeAck(s, EXIODPUP, ok ? EXIORDY : EXIOERR);
      return;
    }

    case EXIOWRD: {
      outboundFlag = EXIOWRD;
      if (len != 2) { displayEvent = EXIOWRD; _writeAck(s, EXIOWRD, EXIOERR); return; }
      uint8_t pin = payload[0];
      bool state = payload[1] ? true : false;
      bool ok = writeDigitalOutput(pin, state);
      _writeAck(s, EXIOWRD, ok ? EXIORDY : EXIOERR);
      return;
    }

    case EXIOENAN: {
      outboundFlag = EXIOENAN;
      if (len != 1) { _writeAck(s, EXIOENAN, EXIOERR); return; }
      uint8_t pin = payload[0];
      bool ok = enableAnalogue(pin);
      _writeAck(s, EXIOENAN, ok ? EXIORDY : EXIOERR);
      return;
    }

    case EXIOWRAN: {
      outboundFlag = EXIOWRAN;
      if (len != 6) { displayEvent = EXIOWRAN; _writeAck(s, EXIOWRAN, EXIOERR); return; }
      uint8_t pin = payload[0];
      uint16_t value = (uint16_t)((payload[2] << 8) | payload[1]);
      uint8_t profile = payload[3];
      uint16_t duration = (uint16_t)((payload[5] << 8) | payload[4]);
      bool ok = writeAnalogue(pin, value, profile, duration);
      _writeAck(s, EXIOWRAN, ok ? EXIORDY : EXIOERR);
      return;
    }

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

    case EXIOSHIFTOUT: {
      outboundFlag = EXIOSHIFTOUT;

      if (len < 4) { _writeAck(s, EXIOSHIFTOUT, EXIOERR); return; }

      uint8_t clk   = payload[0];
      uint8_t latch = payload[1];
      uint8_t data  = payload[2];
      uint8_t n     = payload[3];

      if (n == 0 || n > EXIO_SHIFT_MAX_BYTES) { _writeAck(s, EXIOSHIFTOUT, EXIOERR); return; }
      if (len != (uint8_t)(4 + n)) { _writeAck(s, EXIOSHIFTOUT, EXIOERR); return; }

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

  while (_client.available() >= 2) {
    uint8_t hdr[2];

    if (!_readExact(_client, hdr, 2, 25)) {
      _client.stop();
      USB_SERIAL.println(F("TCP: header timeout -> disconnect"));
      return;
    }

    uint8_t cmd = hdr[0];
    uint8_t len = hdr[1];

    uint8_t payload[32];
    if (len > sizeof(payload)) {
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

      uint32_t t0 = micros();
      _handleCommand(_client, cmd, payload, len);
      uint32_t dt = micros() - t0;

      _tcpFrameCount++;
      _tcpLastLatencyUs = dt;
      _tcpTotalLatencyUs += dt;
      if (dt > _tcpMaxLatencyUs) _tcpMaxLatencyUs = dt;

    } else {
      uint32_t t0 = micros();
      _handleCommand(_client, cmd, nullptr, 0);
      uint32_t dt = micros() - t0;

      _tcpFrameCount++;
      _tcpLastLatencyUs = dt;
      _tcpTotalLatencyUs += dt;
      if (dt > _tcpMaxLatencyUs) _tcpMaxLatencyUs = dt;
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

// ------------------------- Serial helper implementations --------------------

void tcpPrintNetworkStatus() {
  USB_SERIAL.println(F("=== Network Status ==="));

  if (!tcpEnabled()) {
    USB_SERIAL.println(F("TCP: disabled/not compiled"));
    return;
  }

  USB_SERIAL.print(F("Port: "));
  USB_SERIAL.println(IP_PORT);

#if defined(EXIO_TCP_ETH)
  USB_SERIAL.println(F("Transport: Ethernet (Active)"));
  USB_SERIAL.print(F("Local IP: "));
  USB_SERIAL.println(Ethernet.localIP());

  // Link status is not universally supported across all Ethernet libs.
  #if defined(ETHERNET_LINK_STATUS)
    USB_SERIAL.print(F("Link: "));
    auto ls = Ethernet.linkStatus();
    if (ls == LinkON) USB_SERIAL.println(F("ON"));
    else if (ls == LinkOFF) USB_SERIAL.println(F("OFF"));
    else USB_SERIAL.println(F("UNKNOWN"));
  #endif

#elif defined(EXIO_TCP_WIFI)
  USB_SERIAL.println(F("Transport: WiFi (Active)"));
  USB_SERIAL.print(F("WiFi status: "));
  USB_SERIAL.println((WiFi.status() == WL_CONNECTED) ? F("CONNECTED") : F("NOT CONNECTED"));

  if (WiFi.status() == WL_CONNECTED) {
    USB_SERIAL.print(F("Local IP: "));
    USB_SERIAL.println(WiFi.localIP());

    USB_SERIAL.print(F("SSID: "));
    USB_SERIAL.println(WiFi.SSID());

    USB_SERIAL.print(F("RSSI: "));
    USB_SERIAL.print(WiFi.RSSI());
    USB_SERIAL.println(F(" dBm"));
  }
#endif

#if defined(EXIO_TCP_WIFI) || defined(EXIO_TCP_ETH)
  USB_SERIAL.print(F("Client connected: "));
  USB_SERIAL.println((_client && _client.connected()) ? F("YES") : F("NO"));
#endif
}

// ------------------------- Latency getters ----------------------------------

uint32_t tcpGetFrameCount() { return _tcpFrameCount; }
uint32_t tcpGetLastLatencyUs() { return _tcpLastLatencyUs; }

uint32_t tcpGetAvgLatencyUs() {
  if (_tcpFrameCount == 0) return 0;
  return (uint32_t)(_tcpTotalLatencyUs / (uint64_t)_tcpFrameCount);
}

uint32_t tcpGetMaxLatencyUs() { return _tcpMaxLatencyUs; }

// ------------------------- WiFi reconnect -----------------------------------

bool tcpWifiReconnect(const char* ssid, const char* pass) {
#if defined(EXIO_TCP_WIFI)
  #ifdef DONT_TOUCH_WIFI_CONF
    (void)ssid; (void)pass;
    USB_SERIAL.println(F("WiFi reconnect refused: DONT_TOUCH_WIFI_CONF is defined"));
    return false;
  #else
    if (!ssid || ssid[0] == '\0') {
      USB_SERIAL.println(F("WiFi reconnect: SSID missing"));
      return false;
    }

    // Best-effort disconnect
    #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
      WiFi.disconnect(true);
      delay(100);
    #else
      // WiFiS3 has disconnect() but signature differs by core versions; keep it minimal
      WiFi.disconnect();
      delay(100);
    #endif

    if (pass && pass[0] != '\0') {
      WiFi.begin(ssid, pass);
    } else {
      WiFi.begin(ssid);
    }

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED) {
      if ((millis() - start) > WIFI_CONNECT_TIMEOUT) {
        USB_SERIAL.println(F("WiFi reconnect: timeout"));
        return false;
      }
      delay(50);
    }

    USB_SERIAL.print(F("WiFi reconnect: connected, IP="));
    USB_SERIAL.println(WiFi.localIP());
    return true;
  #endif
#else
  (void)ssid; (void)pass;
  USB_SERIAL.println(F("WiFi reconnect: WiFi not compiled/enabled"));
  return false;
#endif
}
