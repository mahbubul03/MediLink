#pragma once
#include <Arduino.h>

// Call once from setup() AFTER sensor/display tasks are already running.
// Uses WiFiManager: tries saved credentials, then opens a captive-portal
// softAP so the user can pick a network. Blocks until WiFi is connected
// (or the portal times out). Returns true on success.
bool firebaseModuleInit();

// FreeRTOS task entry point - keeps WiFi alive and uploads the full
// `vitals` snapshot (all sensors + GPS) to Firebase on an interval.
// Start this task only after firebaseModuleInit() has succeeded.
void firebaseModuleTask(void* pvParameters);
