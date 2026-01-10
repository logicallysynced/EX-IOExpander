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

// Public API
bool tcpEnabled();
void tcpBegin();
void tcpLoop();
void tcpEnd();

// Serial/diagnostic helpers (used by new serial commands)
void tcpPrintNetworkStatus();

// Latency stats (µs) for framed command handling
uint32_t tcpGetFrameCount();
uint32_t tcpGetLastLatencyUs();
uint32_t tcpGetAvgLatencyUs();
uint32_t tcpGetMaxLatencyUs();

// Runtime WiFi reconnect (best-effort; no-op if WiFi not compiled)
bool tcpWifiReconnect(const char* ssid, const char* pass);

#endif
