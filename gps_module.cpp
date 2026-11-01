// ============================================================
//  GPS module - NEO-6M (or similar) NMEA GPS on Serial1
// ============================================================
// Same wiring/baud as the standalone gps_firebase_uploader.ino. This
// module only parses NMEA sentences and writes the result into `vitals`;
// firebase_module reads it from there along with everything else -
// GPS and Firebase don't talk to each other directly.
// ============================================================
#include "gps_module.h"
#include "config.h"
#include "shared_data.h"
#include <TinyGPSPlus.h>

static TinyGPSPlus gps;

void gpsModuleInit() {
  Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void gpsModuleTask(void* pvParameters) {
  (void) pvParameters;
  unsigned long lastWarn = 0;

  for (;;) {
    bool updated = false;
    while (Serial1.available()) {
      if (gps.encode(Serial1.read())) updated = true;
    }

    if (updated && xSemaphoreTake(vitalsMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      vitals.gpsFix = gps.location.isValid();
      if (vitals.gpsFix) {
        vitals.gpsLat = gps.location.lat();
        vitals.gpsLng = gps.location.lng();
      }
      vitals.gpsSatellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
      vitals.gpsHdop       = gps.hdop.isValid() ? gps.hdop.hdop() : 99.0f;
      vitals.gpsUpdatedMs  = millis();
      xSemaphoreGive(vitalsMutex);
    }

    // Same wiring sanity check as the original standalone sketch.
    if (gps.charsProcessed() < 10 && millis() - lastWarn > 10000) {
      Serial.println("[GPS] WARNING: no data received - check wiring/baud rate.");
      lastWarn = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
