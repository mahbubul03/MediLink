#include "config.h"
#include "shared_data.h"
#include "pulseox_module.h"
#include "ecg_module.h"
#include "display_module.h"
#include "gps_module.h"
#include "firebase_module.h"
#include <Wire.h>

void setup(){
  Serial.begin(921600); delay(800);
  Serial.println("\n# PHMS updated firmware: ESP32-S3 / ADS1293 / MAX30105 / ST7796");
  vitalsMutex=xSemaphoreCreateMutex();
  i2cMutex=xSemaphoreCreateMutex();

  displayModuleInit();

  // MAX30105 and FT6336U touch controller share I2C pins 5/6.
  Wire.begin(I2C_SDA_PIN,I2C_SCL_PIN);
  Wire.setClock(I2C_BUS_CLOCK_HZ);
  if(!pulseOxModuleInit()) Serial.println("[PULSEOX] MAX30105 not detected");
  if(!ecgModuleInit()) Serial.println("[ECG] ADS1293 not detected; check SPI/power/wiring");
  gpsModuleInit();

  Wire.begin(I2C_SDA_PIN,I2C_SCL_PIN);
  Wire.setClock(I2C_BUS_CLOCK_HZ);

  xTaskCreatePinnedToCore(pulseOxModuleTask,"PulseOx",PULSEOX_TASK_STACK,nullptr,PULSEOX_TASK_PRIORITY,nullptr,PULSEOX_TASK_CORE);
  xTaskCreatePinnedToCore(ecgModuleTask,"ECG",ECG_TASK_STACK,nullptr,ECG_TASK_PRIORITY,nullptr,ECG_TASK_CORE);
  xTaskCreatePinnedToCore(displayModuleTask,"Display",DISPLAY_TASK_STACK,nullptr,DISPLAY_TASK_PRIORITY,nullptr,DISPLAY_TASK_CORE);
  xTaskCreatePinnedToCore(gpsModuleTask,"GPS",GPS_TASK_STACK,nullptr,GPS_TASK_PRIORITY,nullptr,GPS_TASK_CORE);

  if(firebaseModuleInit()){
    xTaskCreatePinnedToCore(firebaseModuleTask,"Firebase",FIREBASE_TASK_STACK,nullptr,FIREBASE_TASK_PRIORITY,nullptr,FIREBASE_TASK_CORE);
  }else Serial.println("[FIREBASE] WiFi setup not completed; Firebase task not started");
  Serial.println("# PHMS startup complete");
}
void loop(){vTaskDelay(pdMS_TO_TICKS(1000));}
