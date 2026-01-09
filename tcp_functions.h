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

#ifndef TCP_FUNCTIONS_H
#define TCP_FUNCTIONS_H

#include <Arduino.h>
#include "globals.h"

// True if TCP mode is compiled+enabled (ENABLE_WIFI or ENABLE_ETHERNET)
bool tcpEnabled();

// Bring up WiFi/Ethernet and start TCP server on IP_PORT
void tcpBegin();

// Poll/accept client + process framed requests
void tcpLoop();

// Stop active TCP client (server continues)
void tcpEnd();

//
// Optional TCP diagnostics / serial helpers
//

// Print current TCP transport and connection details (best effort)
void tcpPrintNetworkStatus();

// Attempt runtime WiFi reconnect with provided credentials (best effort)
// Returns true if connected at end, false otherwise.
// If DONT_TOUCH_WIFI_CONF is defined, will refuse and return false.
bool tcpWifiReconnect(const char* ssid, const char* pass);

// Latency stats (time spent handling one complete frame: read+dispatch)
uint32_t tcpGetFrameCount();
uint32_t tcpGetLastLatencyUs();
uint32_t tcpGetAvgLatencyUs();
uint32_t tcpGetMaxLatencyUs();

#endif // TCP_FUNCTIONS_H