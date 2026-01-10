/*
 *  © 2023, Peter Cole. All rights reserved.
 *  
 *  This file is part of EX-IOExpander.
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include "globals.h"
#include "serial_functions.h"
#include "test_functions.h"
#include "device_functions.h"
#include "display_functions.h"
#include "tcp_functions.h"

bool newSerialData = false;
const byte numSerialChars = 80;
char serialInputChars[numSerialChars];

static bool _parseIp4(const char* s, uint8_t out[4]) {
  if (!s) return false;

  // Extract up to 4 numbers separated by any non-digit characters
  uint16_t vals[4] = {0,0,0,0};
  uint8_t idx = 0;
  bool inNum = false;
  uint16_t cur = 0;

  for (const char* p = s; *p; p++) {
    if (*p >= '0' && *p <= '9') {
      inNum = true;
      cur = (uint16_t)(cur * 10 + (uint16_t)(*p - '0'));
      if (cur > 255) return false;
    } else {
      if (inNum) {
        if (idx >= 4) return false;
        vals[idx++] = cur;
        cur = 0;
        inNum = false;
      }
    }
  }
  if (inNum) {
    if (idx >= 4) return false;
    vals[idx++] = cur;
  }

  if (idx != 4) return false;

  out[0] = (uint8_t)vals[0];
  out[1] = (uint8_t)vals[1];
  out[2] = (uint8_t)vals[2];
  out[3] = (uint8_t)vals[3];
  return true;
}


/*
* Function to read and process serial input for runtime config
* Format: <X ...>
*/
void processSerialInput() {
  static bool serialInProgress = false;
  static byte serialIndex = 0;
  const char startMarker = '<';
  const char endMarker = '>';
  char serialChar;

  while (Serial.available() > 0 && newSerialData == false) {
    serialChar = Serial.read();
    if (serialInProgress == true) {
      if (serialChar != endMarker) {
        serialInputChars[serialIndex] = serialChar;
        serialIndex++;
        if (serialIndex >= numSerialChars) {
          serialIndex = numSerialChars - 1;
        }
      } else {
        serialInputChars[serialIndex] = '\0';
        serialInProgress = false;
        serialIndex = 0;
        newSerialData = true;
      }
    } else if (serialChar == startMarker) {
      serialInProgress = true;
    }
  }

  if (newSerialData == true) {
    newSerialData = false;

    char* strtokIndex = strtok(serialInputChars, " ");
    if (!strtokIndex || !strtokIndex[0]) return;

    char activity = strtokIndex[0];

    unsigned long parameter = 0;
    uint8_t vpin = 0, profile = 0;
    uint16_t value = 0;

    strtokIndex = strtok(NULL, " ");

    if (activity == 'C') {
      const char* ssid = strtokIndex ? strtokIndex : "";
      const char* pass = "";
      char* passTok = strtok(NULL, " ");
      if (passTok) pass = passTok;
      serialCaseC(ssid, pass);
      return;
    }
    
    if (activity == 'N') { serialCaseN(); return; }
    if (activity == 'L') { serialCaseL(); return; }
        if (activity == 'I') {
      // <I {ip} {subnet} {gateway} {dns}>
      const char* ipStr = strtokIndex ? strtokIndex : "";
      const char* subnetStr = "";
      const char* gwStr = "";
      const char* dnsStr = "";

      char* t = strtok(NULL, " ");
      if (t) subnetStr = t;
      t = strtok(NULL, " ");
      if (t) gwStr = t;
      t = strtok(NULL, " ");
      if (t) dnsStr = t;

      serialCaseI(ipStr, subnetStr, gwStr, dnsStr);
      return;
    }

    if (activity == 'W') {
      if (strtokIndex) parameter = strtol(strtokIndex, NULL, 16);
    } else if (activity == 'T') {
      if (strtokIndex) {
        parameter = strtokIndex[0];
        if (parameter == 'S') {
          strtokIndex = strtok(NULL, " ");
          if (!strtokIndex) return;
          vpin = strtoul(strtokIndex, NULL, 10);

          strtokIndex = strtok(NULL, " ");
          if (!strtokIndex) return;
          value = strtoul(strtokIndex, NULL, 10);

          strtokIndex = strtok(NULL, " ");
          if (!strtokIndex) return;
          profile = strtoul(strtokIndex, NULL, 10);
        }
      }
    } else {
      if (strtokIndex) parameter = strtol(strtokIndex, NULL, 10);
    }

    switch (activity) {
      case 'D': // Enable/disable diagnostic output
        serialCaseD(parameter);
        break;
      case 'E': // Erase EEPROM
        eraseI2CAddress();
        break;
      case 'R': // Read address from EEPROM
        serialCaseR();
        break;
      case 'T': // Display current state of test modes
        if (parameter == 'A') {
          setAnalogueTesting();
        } else if (parameter == 'I') {
          setInputTesting();
        } else if (parameter == 'O') {
          setOutputTesting();
        } else if (parameter == 'P') {
          setPullupTesting();
        } else if (parameter == 'S') {
          testServo(vpin, value, profile);
        } else {
          serialCaseT();
        }
        break;
      case 'V': // Display Vpin map
        startupDisplay();
        displayVpinMap();
        break;
      case 'W': // Write address to EEPROM
        serialCaseW(parameter);
        break;
      case 'Z': // Software reboot
        reset();
        break;
      default:
        break;
    }
  }
}

void setAnalogueTesting() {
  if (analogueTesting) {
    testAnalogue(false);
    USB_SERIAL.println(F("Analogue testing disabled"));
  } else {
    testAnalogue(true);
  }
}

void serialCaseD(unsigned long parameter) {
  if (diag && parameter) {
    displayDelay = parameter * 1000;
    USB_SERIAL.print(F("Diagnostics enabled, delay set to "));
    USB_SERIAL.println(displayDelay);
    diag = true;
  } else if (diag && !parameter) {
    USB_SERIAL.println(F("Diagnostics disabled"));
    diag = false;
  } else {
    if (parameter) {
      displayDelay = parameter * 1000;
    }
    USB_SERIAL.print(F("Diagnostics enabled, delay set to "));
    USB_SERIAL.println(displayDelay);
    diag = true;
  }
}

void setInputTesting() {
  if (inputTesting) {
    testInput(false);
    USB_SERIAL.println(F("Input testing (no pullups) disabled"));
  } else {
    testInput(true);
  }
}

void setOutputTesting() {
  if (outputTesting) {
    testOutput(false);
    USB_SERIAL.println(F("Output testing disabled"));
  } else {
    testOutput(true);
  }
}

void setPullupTesting() {
  if (pullupTesting) {
    testPullup(false);
    USB_SERIAL.println(F("Pullup input testing disabled"));
  } else {
    testPullup(true);
  }
}

void serialCaseR() {
  if (getI2CAddress() == 0) {
    USB_SERIAL.println(F("I2C address not stored, using myConfig.h"));
  } else {
    USB_SERIAL.print(F("I2C address stored is 0x"));
    USB_SERIAL.println(getI2CAddress(), HEX);
  }
}

void serialCaseT() {
  if (analogueTesting) {
    USB_SERIAL.println(F("Analogue testing <A> enabled"));
  } else if (inputTesting) {
    USB_SERIAL.println(F("Input testing <I> (no pullups) enabled"));
  } else if (outputTesting) {
    USB_SERIAL.println(F("Output testing <O> enabled"));
  } else if (pullupTesting) {
    USB_SERIAL.println(F("Pullup input <P> testing enabled"));
  } else {
    USB_SERIAL.println(F("No testing in progress"));
  }
}

void serialCaseW(unsigned long parameter) {
  if (parameter > 0x07 && parameter < 0x78) {
    writeI2CAddress(parameter);
  } else {
    USB_SERIAL.println(F("Invalid I2C address, must be between 0x08 and 0x77"));
  }
}

void serialCaseN() {
  if (!tcpEnabled()) {
    USB_SERIAL.println(F("TCP not enabled/available in this build."));
    return;
  }
  tcpPrintNetworkStatus();
}

void serialCaseL() {
  if (!tcpEnabled()) {
    USB_SERIAL.println(F("TCP not enabled/available in this build."));
    return;
  }

  USB_SERIAL.println(F("=== TCP Latency Stats ==="));
  USB_SERIAL.print(F("Frames: ")); USB_SERIAL.println(tcpGetFrameCount());
  USB_SERIAL.print(F("Last:   ")); USB_SERIAL.print(tcpGetLastLatencyUs()); USB_SERIAL.println(F(" us"));
  USB_SERIAL.print(F("Avg:    ")); USB_SERIAL.print(tcpGetAvgLatencyUs());  USB_SERIAL.println(F(" us"));
  USB_SERIAL.print(F("Max:    ")); USB_SERIAL.print(tcpGetMaxLatencyUs());  USB_SERIAL.println(F(" us"));
  USB_SERIAL.println(F("========================="));
}

void serialCaseC(const char* ssid, const char* pass) {
  if (!tcpEnabled()) {
    USB_SERIAL.println(F("TCP not enabled/available in this build."));
    return;
  }

  if (!ssid || !ssid[0]) {
    USB_SERIAL.println(F("Usage: <C yourSSID yourPassword>  (password optional)"));
    return;
  }

  bool ok = tcpWifiReconnect(ssid, pass);
  if (ok) {
    USB_SERIAL.println(F("WiFi reconnect OK."));
  } else {
    USB_SERIAL.println(F("WiFi reconnect FAILED."));
  }
}

void serialCaseI(const char* ipStr, const char* subnetStr, const char* gwStr, const char* dnsStr) {
  if (!tcpEnabled()) {
    USB_SERIAL.println(F("TCP not enabled/available in this build."));
    return;
  }

  if (!ipStr || !subnetStr || !gwStr || !dnsStr || !ipStr[0] || !subnetStr[0] || !gwStr[0] || !dnsStr[0]) {
    USB_SERIAL.println(F("Usage: <I {ip} {subnet} {gateway} {dns}>"));
    USB_SERIAL.println(F("Example: <I {192,168,1,200} {255,255,255,0} {192,168,1,1} {192,168,1,1}>"));
    return;
  }

  uint8_t ip[4], sn[4], gw[4], dns[4];
  if (!_parseIp4(ipStr, ip) || !_parseIp4(subnetStr, sn) || !_parseIp4(gwStr, gw) || !_parseIp4(dnsStr, dns)) {
    USB_SERIAL.println(F("Invalid IP format. Use {a,b,c,d} or { a, b, c, d } (no spaces preferred)."));
    return;
  }

  bool ok = tcpSetStaticNetwork(ip, sn, gw, dns);
  if (ok) {
    USB_SERIAL.println(F("Static network applied."));
    tcpPrintNetworkStatus();
  } else {
    USB_SERIAL.println(F("Static network NOT applied (transport may not support it, or WiFi locked by DONT_TOUCH_WIFI_CONF)."));
  }
}
