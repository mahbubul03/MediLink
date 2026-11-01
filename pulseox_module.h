#pragma once
#include <Arduino.h>

// Call from setup() after creating vitalsMutex and i2cMutex.
// The module initializes its dedicated MAX30102 I2C controller itself.
bool pulseOxModuleInit();

// FreeRTOS task entry point.
void pulseOxModuleTask(void *pvParameters);
