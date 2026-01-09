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

#endif
