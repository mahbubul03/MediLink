#pragma once
#include <Arduino.h>

// Call once from setup() - starts the GPS UART.
void gpsModuleInit();

// FreeRTOS task entry point - drains NMEA sentences and updates `vitals`.
void gpsModuleTask(void* pvParameters);
