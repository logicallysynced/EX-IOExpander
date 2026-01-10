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

//#define DIAG_IO

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

// ------------------------- State mirroring I2C ------------------------------

static uint8_t outboundFlag = 0;
static byte commandBuffer[3];

// IMPORTANT: must NOT be defined in multiple .cpp files.
// If you already define this in i2c_functions.cpp, remove it here and make it extern in globals.h.
extern uint8_t numReceivedPins;

static const uint8_t EXIO_SHIFT_MAX_BYTES = 16;
static byte responseBuffer[1 + EXIO_SHIFT_MAX_BYTES]; // [status][payload...]
static uint8_t responseLength = 0;

// ------------------------- Latency stats ------------------------------------

static uint32_t _tcpFrameCount = 0;
static uint32_t _tcpLastUs = 0;
static uint32_t _tcpMaxUs = 0;
static uint64_t _tcpTotalUs = 0;

// ------------------------- Network config storage ---------------------------

static uint8_t _ipBytes[4]     = IP_ADDRESS;
static uint8_t _subnetBytes[4] = SUBNET_MASK;
static uint8_t _gwBytes[4]     = GATEWAY_ADDRESS;
static uint8_t _dnsBytes[4]    = DNS_ADDRESS;

// ------------------------- Small portability yield --------------------------

static inline void _tinyYield() {
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  yield();
#else
  delay(0);
#endif
}

static inline IPAddress _ipFromBytes(const uint8_t b[4]) {
  return IPAddress(b[0], b[1], b[2], b[3]);
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

static bool _writeAll(Stream& s, const uint8_t* data, size_t len, uint32_t timeoutMs) {
  uint32_t start = millis();
  size_t sent = 0;

  while (sent < len) {
    if ((millis() - start) > timeoutMs) return false;

    // Stream::write can be short; handle it.
    size_t n = s.write(data + sent, len - sent);

    if (n == 0) {
      #ifdef DIAG_IO
        USB_SERIAL.println(F("TCP: write returned 0"));
      #endif
      _tinyYield();
      continue;
    }
    sent += n;
  }
  return true;
}

static bool _writeFrame(Stream& s, uint8_t cmd, const uint8_t* payload, uint8_t len) {
  // Write header first (2 bytes)
  uint8_t hdr[2] = { cmd, len };

  if (!_writeAll(s, hdr, 2, 2000)) return false;

  // Write payload as-is (do NOT copy into a stack frame)
  if (len && payload) {
    if (!_writeAll(s, payload, len, 2000)) return false;
  }

  // Give WiFi stack time
  _tinyYield();
  delay(2);       // I'd start with 2ms on WiFiS3; bump to 5ms if needed
  _tinyYield();

  return true;
}

static bool _writeAck(Stream& s, uint8_t cmd, uint8_t statusByte) {
  return _writeFrame(s, cmd, &statusByte, 1);
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

static bool _wifiApplyStaticConfig() {
  // Best-effort: WiFi.config(ip, dns, gateway, subnet) is common across WiFi libs.
  // If not supported by a particular core, it will fail to compile; in that case,
  // comment this out for that core and rely on DHCP.
  IPAddress ip     = _ipFromBytes(_ipBytes);
  IPAddress dns    = _ipFromBytes(_dnsBytes);
  IPAddress gw     = _ipFromBytes(_gwBytes);
  IPAddress subnet = _ipFromBytes(_subnetBytes);

  // Many WiFi libraries return bool; some return void. Handle both.
  #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
    return WiFi.config(ip, gw, subnet, dns);
  #else
    // WiFiS3 / others often use: config(ip, dns, gateway, subnet)
    // If your core expects (ip, dns, gateway, subnet), this matches.
    WiFi.config(ip, dns, gw, subnet);
    return true;
  #endif
}

static bool _wifiEnsureConnected() {
  // Honor "do not touch" only on ESP cores. On WiFiS3 boards you typically must begin().
#if defined(DONT_TOUCH_WIFI_CONF) && (defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266))
  return (WiFi.status() == WL_CONNECTED);
#else
  if (WiFi.status() == WL_CONNECTED) return true;

  #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
    WiFi.mode(WIFI_STA);
  #endif

  // Hostname (best-effort; not all cores support this)
  #if defined(ARDUINO_ARCH_ESP32)
    if (strlen(WIFI_HOSTNAME) > 0) WiFi.setHostname(WIFI_HOSTNAME);
  #elif defined(ARDUINO_ARCH_ESP8266)
    if (strlen(WIFI_HOSTNAME) > 0) WiFi.hostname(WIFI_HOSTNAME);
  #endif

  // Apply static config if USE_DHCP=false
  #if defined(USE_DHCP) && (USE_DHCP == false)
    (void)_wifiApplyStaticConfig();
  #endif

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if ((millis() - start) > WIFI_CONNECT_TIMEOUT) return false;
    delay(100);
  }

  // Give DHCP/static a moment to populate localIP()
  for (uint8_t i = 0; i < 20; i++) {
    IPAddress ip = WiFi.localIP();
    if (ip[0] || ip[1] || ip[2] || ip[3]) return true;
    delay(100);
  }

  return true;
#endif
}

#endif // EXIO_TCP_WIFI

#if defined(EXIO_TCP_ETH)

static void _ethBegin() {
  byte mac[] = MAC_ADDRESS;

  IPAddress ip     = _ipFromBytes(_ipBytes);
  IPAddress dns    = _ipFromBytes(_dnsBytes);
  IPAddress gw     = _ipFromBytes(_gwBytes);
  IPAddress subnet = _ipFromBytes(_subnetBytes);

  bool ok = false;

  #if defined(USE_DHCP) && (USE_DHCP == true)
    int dhcp = Ethernet.begin(mac);
    ok = (dhcp != 0);
  #endif

  if (!ok) {
    // Static fallback or forced static
    Ethernet.begin(mac, ip, dns, gw, subnet);
  }

  delay(250);
}

#endif // EXIO_TCP_ETH

// ------------------------- Protocol handlers --------------------------------

static void _handleCommand(Stream& s, uint8_t cmd, const uint8_t* payload, uint8_t len) {

  #ifdef DIAG_IO
    USB_SERIAL.print(F("TCP: rx cmd=0x"));
    USB_SERIAL.print(cmd, HEX);
    USB_SERIAL.print(F(" len="));
    USB_SERIAL.println(len);
  #endif


  switch (cmd) {

    case EXIOINIT: {
      if (len != 3) {
        displayEventFlag = 2;
        uint8_t resp[3] = { EXIOPINS, numDigitalPins, numAnaloguePins };
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

      //uint8_t resp[3] = { EXIOPINS, numDigitalPins, 0 };
      uint8_t resp[3] = { EXIOPINS, numDigitalPins, numAnaloguePins };
      //_writeFrame(s, EXIOPINS, resp, 3);

      if (!_writeFrame(s, EXIOPINS, resp, 3)) {
        #ifdef DIAG_IO
          USB_SERIAL.println(F("TCP: tx failed, dropping client [EXIOINIT]"));
        #endif
        _client.stop();
        return;
      }

      return;
    }

    case EXIOINITA: {
      outboundFlag = EXIOINITA;

      // Protocol guarantee:
      // If numAnaloguePins > 0, we MUST return that many bytes
      // even if setup is not complete yet.
      static uint8_t emptyMap[32] = {0}; // max analogue pins safeguard

      const uint8_t n = (uint8_t)numAnaloguePins;

      // Always send exactly n bytes. If setup isn't complete OR map isn't ready,
      // send an all-zero placeholder map.
      const uint8_t* map = analoguePinMap;
      if (!setupComplete || map == nullptr) {
        map = emptyMap;
      }

      
      if (!_writeFrame(s, EXIOINITA, map, n)) {
        #ifdef DIAG_IO
          USB_SERIAL.println(F("TCP: tx failed, dropping client [EXIOINITA]"));
        #endif
        _client.stop();
        return;
      }
      
      
      /*
      #ifdef DIAG_IO
      USB_SERIAL.print(F("TCP: INITA map bytes: "));
      for (uint8_t i = 0; i < n; i++) {
        USB_SERIAL.print(map[i], HEX);
        USB_SERIAL.print(' ');
      }
      USB_SERIAL.println();
      #endif

      bool ok = _writeFrame(s, EXIOINITA, map, n);
      #ifdef DIAG_IO
      USB_SERIAL.print(F("TCP: EXIOINITA tx n="));
      USB_SERIAL.print(n);
      USB_SERIAL.print(F(" ok="));
      USB_SERIAL.println(ok ? 1 : 0);
      #endif
      if (!ok) { _client.stop(); return; }
      */
      

      return;
    }

    case EXIORDAN: {
      outboundFlag = EXIORDAN;

      // CommandStation expects exactly numAnaloguePins*2 bytes every time.
      const uint8_t need = (uint8_t)(numAnaloguePins * 2);

      // Provide a safe zero-filled fallback.
      static uint8_t emptyStates[64] = {0}; // supports up to 32 analogue pins (32*2=64)

      const uint8_t* buf = analoguePinStates;

      // If we're not ready or the buffer/bytecount isn't sane, send zeros of the expected size.
      if (!setupComplete || buf == nullptr || analoguePinBytes < need) {
        buf = emptyStates;
      }

      #ifdef DIAG_IO

        USB_SERIAL.print(F("TCP: EXIORDAN tx need="));
        USB_SERIAL.print(need);
        USB_SERIAL.print(F(" analoguePinBytes="));
        USB_SERIAL.print((int)analoguePinBytes);
        USB_SERIAL.print(F(" setupComplete="));
        USB_SERIAL.println(setupComplete ? 1 : 0);

      #endif

      //_writeFrame(s, EXIORDAN, buf, need);

      if (!_writeFrame(s, EXIORDAN, buf, need)) {
        #ifdef DIAG_IO
          USB_SERIAL.println(F("TCP: tx failed, dropping client [EXIORDAN]"));
        #endif
        _client.stop();
        return;
      }

      return;
    }


    case EXIORDD: {
      outboundFlag = EXIORDD;

      const uint8_t need = (uint8_t)((numDigitalPins + 7) / 8);
      static uint8_t emptyDig[32] = {0}; // supports up to 256 digital pins (256/8=32)

      const uint8_t* buf = digitalPinStates;
      if (!setupComplete || buf == nullptr || digitalPinBytes < need) {
        buf = emptyDig;
      }

      if (!_writeFrame(s, EXIORDD, buf, need)) {
        #ifdef DIAG_IO
          USB_SERIAL.println(F("TCP: tx failed, dropping client [EXIORDD]"));
        #endif
        _client.stop();
        return;
      }

      //_writeFrame(s, EXIORDD, buf, need);
      return;
    }

    case EXIOVER: {
      outboundFlag = EXIOVER;
      //_writeFrame(s, EXIOVER, versionBuffer, 3);

      if (!_writeFrame(s, EXIOVER, versionBuffer, 3)) {
        #ifdef DIAG_IO
          USB_SERIAL.println(F("TCP: tx failed, dropping client [EXIOVER]"));
        #endif
        _client.stop();
        return;
      }

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

// ------------------------- Rebind helpers -----------------------------------

static void _serverRebind() {
#if defined(EXIO_TCP_WIFI) || defined(EXIO_TCP_ETH)
  if (_client) _client.stop();
  // On many cores, calling begin() again is fine to rebind.
  _server.begin();
#endif
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

  if (_client && !_client.connected()) {
    USB_SERIAL.println(F("TCP: client disconnected"));
    _client.stop();
    _client = WiFiClient();   // <--- hard reset the underlying handle
  }

  if (!_client || !_client.connected()) {
    WiFiClient incoming = _server.available();
    if (incoming) {
      _client.stop();
      _client = incoming;
      USB_SERIAL.println(F("TCP: client connected"));
    } else {
      return;
    }
  }


  while (_client.available() >= 2) {
    const uint32_t t0 = micros();

    uint8_t hdr[2];
    if (!_readExact(_client, hdr, 2, 250)) {
      _client.stop();
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
      return;
    }

    if (len > 0) {
      if (!_readExact(_client, payload, len, 250)) {
        _client.stop();
        return;
      }
      _handleCommand(_client, cmd, payload, len);
    } else {
      _handleCommand(_client, cmd, nullptr, 0);
    }

    const uint32_t t1 = micros();
    const uint32_t dt = (uint32_t)(t1 - t0);

    _tcpFrameCount++;
    _tcpLastUs = dt;
    _tcpTotalUs += dt;
    if (dt > _tcpMaxUs) _tcpMaxUs = dt;

    _tinyYield();
  }

#endif
}

void tcpEnd() {
#if defined(EXIO_TCP_WIFI) || defined(EXIO_TCP_ETH)
  if (_client) _client.stop();
#endif
}

// ------------------------- Diagnostics APIs ---------------------------------

void tcpPrintNetworkStatus() {
#if defined(EXIO_TCP_WIFI) || defined(EXIO_TCP_ETH)

  USB_SERIAL.println(F("=== TCP Network Status ==="));

  // Transport
#if defined(EXIO_TCP_ETH)
  USB_SERIAL.println(F("Transport: Ethernet"));
#elif defined(EXIO_TCP_WIFI)
  USB_SERIAL.println(F("Transport: WiFi"));
#else
  USB_SERIAL.println(F("Transport: (unknown)"));
#endif

  // IP details + port
#if defined(EXIO_TCP_ETH)
  IPAddress ip  = Ethernet.localIP();
  IPAddress sn  = Ethernet.subnetMask();
  IPAddress gw  = Ethernet.gatewayIP();
  IPAddress dns = Ethernet.dnsServerIP();
#elif defined(EXIO_TCP_WIFI)
  IPAddress ip  = WiFi.localIP();
  // Some cores provide these; others don't. Guard with fallbacks.
  IPAddress sn(0,0,0,0);
  IPAddress gw(0,0,0,0);
  IPAddress dns(0,0,0,0);

  // Arduino WiFi API commonly provides these on many cores:
  // (If your core doesn't, you'll get a compile error here—see notes below.)
  sn  = WiFi.subnetMask();
  gw  = WiFi.gatewayIP();
  dns = WiFi.dnsIP();
#endif

  USB_SERIAL.print(F("Local IP:  ")); USB_SERIAL.println(ip);
  USB_SERIAL.print(F("Subnet:    ")); USB_SERIAL.println(sn);
  USB_SERIAL.print(F("Gateway:   ")); USB_SERIAL.println(gw);
  USB_SERIAL.print(F("DNS:       ")); USB_SERIAL.println(dns);
  USB_SERIAL.print(F("Port:      ")); USB_SERIAL.println(IP_PORT);

  // Link / WiFi details (best-effort)
#if defined(EXIO_TCP_ETH)
  #ifdef ETHERNET_LINK_STATUS
    USB_SERIAL.print(F("Link:      "));
    auto ls = Ethernet.linkStatus();
    if (ls == LinkON) USB_SERIAL.println(F("up"));
    else if (ls == LinkOFF) USB_SERIAL.println(F("down"));
    else USB_SERIAL.println(F("unknown"));
  #endif
#elif defined(EXIO_TCP_WIFI)
  USB_SERIAL.print(F("WiFi:      "));
  if (WiFi.status() == WL_CONNECTED) {
    USB_SERIAL.print(F("connected, SSID="));
    USB_SERIAL.print(WiFi.SSID());
    USB_SERIAL.print(F(", RSSI="));
    USB_SERIAL.println(WiFi.RSSI());
  } else {
    USB_SERIAL.println(F("not connected"));
  }
#endif

  // Client status
  USB_SERIAL.print(F("Client:    "));
  USB_SERIAL.println(_client.connected() ? F("connected") : F("none"));

  USB_SERIAL.println(F("=========================="));

#else
  USB_SERIAL.println(F("TCP not enabled/available in this build."));
#endif
}


uint32_t tcpGetFrameCount() { return _tcpFrameCount; }
uint32_t tcpGetLastLatencyUs() { return _tcpLastUs; }
uint32_t tcpGetMaxLatencyUs() { return _tcpMaxUs; }
uint32_t tcpGetAvgLatencyUs() {
  if (_tcpFrameCount == 0) return 0;
  return (uint32_t)(_tcpTotalUs / _tcpFrameCount);
}

// ------------------------- WiFi runtime reconnect ---------------------------

bool tcpWifiReconnect(const char* ssid, const char* pass) {
#if defined(EXIO_TCP_WIFI)
#if defined(DONT_TOUCH_WIFI_CONF)
  USB_SERIAL.println(F("WiFi reconnect refused: DONT_TOUCH_WIFI_CONF is defined."));
  return false;
#else
  if (!ssid || !ssid[0]) return false;

  if (_client) _client.stop();

  WiFi.disconnect();
  delay(250);

  // Re-apply static config if USE_DHCP=false
  #if defined(USE_DHCP) && (USE_DHCP == false)
    (void)_wifiApplyStaticConfig();
  #endif

  WiFi.begin(ssid, pass ? pass : "");

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if ((millis() - start) > WIFI_CONNECT_TIMEOUT) return false;
    delay(100);
  }

  // Wait for IP
  for (uint8_t i = 0; i < 20; i++) {
    IPAddress ip = WiFi.localIP();
    if (ip[0] || ip[1] || ip[2] || ip[3]) break;
    delay(100);
  }

  _serverRebind();
  return true;
#endif
#else
  (void)ssid; (void)pass;
  return false;
#endif
}

// ------------------------- Static network set/apply -------------------------

bool tcpSetStaticNetwork(const uint8_t ip[4],
                         const uint8_t subnet[4],
                         const uint8_t gateway[4],
                         const uint8_t dns[4]) {
#if defined(EXIO_TCP_WIFI) || defined(EXIO_TCP_ETH)
  if (!ip || !subnet || !gateway || !dns) return false;

  memcpy(_ipBytes, ip, 4);
  memcpy(_subnetBytes, subnet, 4);
  memcpy(_gwBytes, gateway, 4);
  memcpy(_dnsBytes, dns, 4);

  // Apply immediately
#if defined(EXIO_TCP_ETH)
  _ethBegin();
  _serverRebind();
  return true;
#elif defined(EXIO_TCP_WIFI)
#if defined(DONT_TOUCH_WIFI_CONF)
  USB_SERIAL.println(F("Static IP set stored, but not applied: DONT_TOUCH_WIFI_CONF is defined."));
  return false;
#else
  if (_client) _client.stop();

  // Reconnect using existing SSID/PASS macros
  WiFi.disconnect();
  delay(250);

  (void)_wifiApplyStaticConfig();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if ((millis() - start) > WIFI_CONNECT_TIMEOUT) return false;
    delay(100);
  }

  // Wait for IP
  for (uint8_t i = 0; i < 20; i++) {
    IPAddress lip = WiFi.localIP();
    if (lip[0] || lip[1] || lip[2] || lip[3]) break;
    delay(100);
  }

  _serverRebind();
  return true;
#endif
#endif

#else
  (void)ip; (void)subnet; (void)gateway; (void)dns;
  return false;
#endif
}
