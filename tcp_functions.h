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
#include <stdint.h>

bool tcpEnabled();

void tcpBegin();
void tcpLoop();
void tcpEnd();

// --- Serial / diagnostics helpers ---
void tcpPrintNetworkStatus();

uint32_t tcpGetFrameCount();
uint32_t tcpGetLastLatencyUs();
uint32_t tcpGetAvgLatencyUs();
uint32_t tcpGetMaxLatencyUs();

// WiFi-only helper (returns false if not supported / not enabled)
bool tcpWifiReconnect(const char* ssid, const char* pass);

// Static network config (works for Ethernet + WiFi where supported).
// Applies immediately: reconnect WiFi / restart Ethernet + TCP server.
bool tcpSetStaticNetwork(const uint8_t ip[4],
                         const uint8_t subnet[4],
                         const uint8_t gateway[4],
                         const uint8_t dns[4]);

#endif
