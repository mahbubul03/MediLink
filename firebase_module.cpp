// ============================================================
//  Firebase module - WiFiManager captive portal + vitals upload
// ============================================================
// WiFi SSID/password are NOT hardcoded. On boot, WiFiManager:
//   1. Tries the last network saved in NVS (if any)
//   2. If that fails, starts a softAP captive portal so the user can
//      select a network and enter the password on their phone
// Once STA is connected, this module PUTs the full vitals snapshot to
// Firebase on an interval (same REST approach as before).
// ============================================================
#include "firebase_module.h"
#include "config.h"
#include "shared_data.h"

#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

static const char* FIREBASE_URL = FIREBASE_DATABASE_URL "vitals.json?auth=" FIREBASE_API_KEY;

bool firebaseModuleInit() {
  WiFi.mode(WIFI_STA);

  WiFiManager wm;
  wm.setHostname("PHMS");
  // Portal stays open until the user picks a network and the ESP connects.
  // (No timeout — matches "configure WiFi on every fresh setup session".)

  Serial.println("[WIFI] Starting configuration portal (captive portal)...");
  Serial.print("[WIFI] Connect your phone to WiFi AP: ");
  Serial.println(WIFI_MANAGER_AP_NAME);
  Serial.println("[WIFI] Then open http://192.168.4.1 (phones often open it automatically).");
  Serial.println("[WIFI] Select your network, enter the password, and save.");

  bool ok;
  if (WIFI_MANAGER_AP_PASSWORD[0] != '\0') {
    ok = wm.startConfigPortal(WIFI_MANAGER_AP_NAME, WIFI_MANAGER_AP_PASSWORD);
  } else {
    ok = wm.startConfigPortal(WIFI_MANAGER_AP_NAME);
  }

  if (!ok) {
    Serial.println("[WIFI] ERROR: portal closed without a successful connection.");
    Serial.println("[WIFI] Firebase uploads will not start until the next successful boot/config.");
    return false;
  }

  Serial.print("[WIFI] Connected to \"");
  Serial.print(WiFi.SSID());
  Serial.print("\", IP: ");
  Serial.println(WiFi.localIP());
  Serial.println("[FIREBASE] WiFi ready - starting uploads.");
  return true;
}

// Reconnect using credentials WiFiManager already stored in NVS.
// Never blocks waiting for association.
static void ensureWiFi() {
  static unsigned long lastAttempt = 0;
  static bool wasConnected = false;

  if (WiFi.status() == WL_CONNECTED) {
    if (!wasConnected) {
      Serial.print("[FIREBASE] WiFi connected, IP: ");
      Serial.println(WiFi.localIP());
      wasConnected = true;
    }
    return;
  }

  wasConnected = false;
  if (millis() - lastAttempt < FIREBASE_WIFI_RETRY_MS) return;
  lastAttempt = millis();

  Serial.println("[FIREBASE] WiFi not connected - retrying with saved credentials...");
  WiFi.disconnect();
  WiFi.begin();   // ESP32: uses last credentials saved by WiFiManager / NVS
}

static String buildVitalsJson() {
  VitalsData v = getVitalsSnapshot();

  String json = "{";
  json += "\"heartRateBPM\":"    + String(v.hrValid ? v.heartRateBPM : -1) + ",";
  json += "\"hrAgeMs\":"         + String(v.hrUpdatedMs ? (long)(millis() - v.hrUpdatedMs) : -1) + ",";
  json += "\"spo2Percent\":"     + String(v.spo2Valid ? v.spo2Percent : -1) + ",";
  json += "\"spo2AgeMs\":"       + String(v.spo2UpdatedMs ? (long)(millis() - v.spo2UpdatedMs) : -1) + ",";
  json += "\"pulseOxStatus\":\"" + v.pulseOxStatus + "\",";
  json += "\"gpsFix\":"          + String(v.gpsFix ? "true" : "false") + ",";
  json += "\"lat\":"             + String(v.gpsFix ? v.gpsLat : 0, 6) + ",";
  json += "\"lng\":"             + String(v.gpsFix ? v.gpsLng : 0, 6) + ",";
  json += "\"satellites\":"      + String(v.gpsSatellites) + ",";
  json += "\"hdop\":"            + String(v.gpsHdop, 1) + ",";
  json += "\"updatedMs\":"       + String(millis());
  json += "}";
  return json;
}

static void uploadOnce() {
  WiFiClientSecure client;
  client.setInsecure();   // skip cert validation - fine for a hobby/class project

  HTTPClient http;
  http.begin(client, FIREBASE_URL);
  http.addHeader("Content-Type", "application/json");

  String json = buildVitalsJson();
  int httpCode = http.PUT(json);   // PUT overwrites /vitals with the latest snapshot

  Serial.print("[FIREBASE] Upload -> HTTP ");
  Serial.print(httpCode);
  Serial.print(" | ");
  Serial.println(json);

  http.end();
}

void firebaseModuleTask(void* pvParameters) {
  (void) pvParameters;
  unsigned long lastUpload = 0;

  for (;;) {
    ensureWiFi();

    if (WiFi.status() == WL_CONNECTED && millis() - lastUpload > FIREBASE_UPLOAD_INTERVAL_MS) {
      lastUpload = millis();
      uploadOnce();
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}
