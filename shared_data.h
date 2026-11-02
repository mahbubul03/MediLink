#pragma once
#include <Arduino.h>
#include "config.h"

#define ECG_WAVE_BUF_LEN 480
struct VitalsData {
  int heartRateBPM=-1, spo2Percent=-1;
  bool hrValid=false, spo2Valid=false;
  String pulseOxStatus="PLACE FINGER";
  unsigned long hrUpdatedMs=0, spo2UpdatedMs=0;
  int ecgBPM=-1;
  bool ecgBpmValid=false;
  String ecgStatus="STARTING";
  bool gpsFix=false;
  double gpsLat=0, gpsLng=0;
  int gpsSatellites=0;
  float gpsHdop=99;
  unsigned long gpsUpdatedMs=0;
};
extern VitalsData vitals;
extern SemaphoreHandle_t vitalsMutex;
extern SemaphoreHandle_t i2cMutex;
// Guards the shared FSPI bus (TFT + ADS1293 share SCLK/MOSI/MISO).
extern SemaphoreHandle_t spiMutex;
VitalsData getVitalsSnapshot();

// Three displayed limb leads, stored as baseline-centered tenths of mV.
// Lead III is calculated using Einthoven's law: III = II - I.
extern volatile int16_t ecgWaveI[ECG_WAVE_BUF_LEN];
extern volatile int16_t ecgWaveII[ECG_WAVE_BUF_LEN];
extern volatile int16_t ecgWaveIII[ECG_WAVE_BUF_LEN];
extern volatile uint32_t ecgWaveWriteIdx;
