#include "config.h"
#include "shared_data.h"
#include "pulseox_module.h"
#include "ecg_module.h"
#include "display_module.h"
#include "gps_module.h"
#include "firebase_module.h"

void setup(){
  Serial.begin(921600); delay(800);
  Serial.println("\n# PHMS updated firmware: ESP32-S3 / ADS1293 / MAX30105 / ST7796");

  vitalsMutex = xSemaphoreCreateMutex();
  i2cMutex    = xSemaphoreCreateMutex();
  spiMutex    = xSemaphoreCreateMutex();   // TFT + ADS1293 share SCLK/MOSI/MISO

  // Park BOTH chip-selects high before anything toggles the shared SPI bus,
  // otherwise the ADS1293 can decode TFT traffic as register writes.
  digitalWrite(ECG_CS, HIGH); pinMode(ECG_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH); pinMode(TFT_CS, OUTPUT);

  // displayModuleInit() owns Wire (FT6336U touch on GPIO5/6).
  // The MAX30102 uses its own TwoWire(1) on GPIO21/20 inside pulseox_module.
  // Do NOT call Wire.begin() again here: it re-pins controller 0.
  displayModuleInit();

  if(!pulseOxModuleInit()) Serial.println("[PULSEOX] MAX30105 not detected");
  const bool ecgOk = ecgModuleInit();
  if(!ecgOk) Serial.println("[ECG] ADS1293 not detected; check SPI/power/wiring");
  gpsModuleInit();

  xTaskCreatePinnedToCore(pulseOxModuleTask,"PulseOx",PULSEOX_TASK_STACK,nullptr,PULSEOX_TASK_PRIORITY,nullptr,PULSEOX_TASK_CORE);
  if(ecgOk) xTaskCreatePinnedToCore(ecgModuleTask,"ECG",ECG_TASK_STACK,nullptr,ECG_TASK_PRIORITY,nullptr,ECG_TASK_CORE);
  xTaskCreatePinnedToCore(displayModuleTask,"Display",DISPLAY_TASK_STACK,nullptr,DISPLAY_TASK_PRIORITY,nullptr,DISPLAY_TASK_CORE);
  xTaskCreatePinnedToCore(gpsModuleTask,"GPS",GPS_TASK_STACK,nullptr,GPS_TASK_PRIORITY,nullptr,GPS_TASK_CORE);

  if(firebaseModuleInit()){
    xTaskCreatePinnedToCore(firebaseModuleTask,"Firebase",FIREBASE_TASK_STACK,nullptr,FIREBASE_TASK_PRIORITY,nullptr,FIREBASE_TASK_CORE);
  }else Serial.println("[FIREBASE] WiFi setup not completed; Firebase task not started");
  Serial.println("# PHMS startup complete");
}
void loop(){vTaskDelay(pdMS_TO_TICKS(1000));}
