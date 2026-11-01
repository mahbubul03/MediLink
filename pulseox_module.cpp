// ============================================================
//  Pulse-ox module - heart rate + SpO2 from ONE MAX30102/MAX30105
// ============================================================
// Heart_Rate.ino and SpO2.ino each assumed exclusive access to the
// sensor. On real hardware there's only one MAX30105, so this module
// reads the FIFO once per sample and feeds BOTH algorithms from that
// same reading:
//   - checkForBeat()                    -> beat-to-beat HR
//   - maxim_heart_rate_and_oxygen_saturation() -> SpO2 (sliding window)
// The original beat detection, HR filtering, SpO2 sliding window,
// finger state machine and recovery flow are retained. I2C recovery
// is isolated to controller 1 so touch on controller 0 is unaffected.
// ============================================================
#include "pulseox_module.h"
#include "config.h"
#include "shared_data.h"
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

static MAX30105 sensor;

// MAX30102 uses ESP32-S3 I2C controller 1 on GPIO 21/20.
// Wire/controller 0 is kept separate for FT6336U touch on GPIO 5/6.
static TwoWire MAX30102Wire(1);
static bool max30102BusStarted = false;

#define FINGER_THRESHOLD           5000
#define GOOD_SIGNAL_THRESHOLD      50000
#define SENSOR_TIMEOUT_MS          5000UL
#define POST_RECOVERY_FIFO_WAIT_MS 600UL
#define FINGER_LOST_CONFIRM_COUNT  5

#define REG_MODE_CONFIG  0x09
#define MODE_RESET_BIT   0x40
#define MODE_SHDN_BIT    0x80

// ---- SpO2 (Maxim algorithm) sliding window ----
#define SPO2_BUFFER_LEN 100
static uint32_t irBuffer[SPO2_BUFFER_LEN];
static uint32_t redBuffer[SPO2_BUFFER_LEN];
static int      spo2FillIndex = 0;

static int32_t spo2      = -999;
static int8_t  validSPO2 = 0;
static int32_t algoHR    = -999;   // Maxim's own HR estimate - not used for display,
static int8_t  validHR   = 0;      // beat-detection HR (below) is more responsive.

// ---- HR (beat-to-beat detection) rolling buffer ----
#define BPM_MIN                 40.0f
#define BPM_MAX                 180.0f
#define RR_DEVIATION_THRESHOLD  0.20f
#define IIR_ALPHA               0.75f
#define HR_RING_LEN             10
#define MIN_VALID_BEATS         5

static float hrRing[HR_RING_LEN];
static int   hrRingCount = 0;
static long  lastBeat    = 0;
static long  lastRR      = 0;
static float iirBPM      = 0.0f;
static bool  iirSeeded   = false;

// ---- Shared finger / session state ----
typedef enum { PO_IDLE, PO_STABILIZING, PO_ACTIVE } PulseOxState;
static PulseOxState   state           = PO_IDLE;
static unsigned long  fingerTime      = 0;
static int            fingerLostCount = 0;
static unsigned long  lastIRUpdate    = 0;

// ============================================================
//  I2C recovery (identical strategy to the standalone sketches)
// ============================================================
static uint32_t waitForFirstSample(unsigned long timeoutMs) {
  unsigned long deadline = millis() + timeoutMs;
  while (millis() < deadline) {
    sensor.check();
    if (sensor.available()) {
      uint32_t ir = sensor.getFIFOIR();
      sensor.nextSample();
      return ir;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  return 0;
}

static void recoverI2CBus() {
  Serial.println("[PULSEOX][RECOVERY] Sending 9 SCL pulses...");
  MAX30102Wire.end();
  vTaskDelay(pdMS_TO_TICKS(10));

  pinMode(PULSEOX_SDA_PIN, INPUT_PULLUP);
  pinMode(PULSEOX_SCL_PIN, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(PULSEOX_SCL_PIN, HIGH); delayMicroseconds(5);
    digitalWrite(PULSEOX_SCL_PIN, LOW);  delayMicroseconds(5);
  }
  pinMode(PULSEOX_SDA_PIN, OUTPUT);
  digitalWrite(PULSEOX_SDA_PIN, LOW);  delayMicroseconds(5);
  digitalWrite(PULSEOX_SCL_PIN, HIGH); delayMicroseconds(5);
  digitalWrite(PULSEOX_SDA_PIN, HIGH); delayMicroseconds(5);

  pinMode(PULSEOX_SDA_PIN, INPUT_PULLUP);
  pinMode(PULSEOX_SCL_PIN, INPUT_PULLUP);
  vTaskDelay(pdMS_TO_TICKS(10));
  MAX30102Wire.begin(PULSEOX_SDA_PIN, PULSEOX_SCL_PIN, I2C_BUS_CLOCK_HZ);
  MAX30102Wire.setTimeOut(100);
  max30102BusStarted = true;
  vTaskDelay(pdMS_TO_TICKS(10));
}

static bool checkAndWakeSensor() {
  MAX30102Wire.beginTransmission(MAX30102_I2C_ADDR);
  MAX30102Wire.write(REG_MODE_CONFIG);
  if (MAX30102Wire.endTransmission(false) != 0) return false;
  MAX30102Wire.requestFrom((uint8_t) MAX30102_I2C_ADDR, (uint8_t) 1);
  if (!MAX30102Wire.available()) return false;

  uint8_t modeReg = MAX30102Wire.read();
  if (modeReg & MODE_SHDN_BIT) {
    Serial.println("[PULSEOX][RECOVERY] Sensor in SHDN - waking...");
    MAX30102Wire.beginTransmission(MAX30102_I2C_ADDR);
    MAX30102Wire.write(REG_MODE_CONFIG);
    MAX30102Wire.write(modeReg & ~MODE_SHDN_BIT);
    MAX30102Wire.endTransmission();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return true;
}

static bool reinitSensor() {
  Serial.println("[PULSEOX][RECOVERY] Software reset...");
  MAX30102Wire.beginTransmission(MAX30102_I2C_ADDR);
  MAX30102Wire.write(REG_MODE_CONFIG);
  MAX30102Wire.write(MODE_RESET_BIT);
  MAX30102Wire.endTransmission();

  unsigned long t = millis();
  while (millis() - t < 100) {
    vTaskDelay(pdMS_TO_TICKS(5));
    MAX30102Wire.beginTransmission(MAX30102_I2C_ADDR);
    MAX30102Wire.write(REG_MODE_CONFIG);
    if (MAX30102Wire.endTransmission(false) == 0) {
      MAX30102Wire.requestFrom((uint8_t) MAX30102_I2C_ADDR, (uint8_t) 1);
      if (MAX30102Wire.available() && !(MAX30102Wire.read() & MODE_RESET_BIT)) break;
    }
  }
  vTaskDelay(pdMS_TO_TICKS(10));

  // Keep the dedicated MAX30102 bus at 100 kHz for reliable recovery.
  if (!sensor.begin(MAX30102Wire, I2C_SPEED_STANDARD)) return false;

  sensor.setup(60, 4, 2, 100, 411, 4096);   // brightness 60 matches the Maxim SpO2 lookup table
  sensor.setPulseAmplitudeRed(0x24);
  sensor.setPulseAmplitudeIR(0x24);
  sensor.setPulseAmplitudeGreen(0);
  sensor.clearFIFO();

  Serial.println("[PULSEOX][RECOVERY] Waiting for FIFO...");
  return waitForFirstSample(POST_RECOVERY_FIFO_WAIT_MS) > 0;
}

static bool recoverSensor() {
  Serial.println("\n[PULSEOX] === SENSOR WATCHDOG TRIGGERED ===");

  if (checkAndWakeSensor()) {
    if (waitForFirstSample(POST_RECOVERY_FIFO_WAIT_MS) > 0) {
      Serial.println("[PULSEOX][RECOVERY] Woke from SHDN - OK.");
      return true;
    }
  }

  recoverI2CBus();
  vTaskDelay(pdMS_TO_TICKS(50));
  if (checkAndWakeSensor()) {
    if (waitForFirstSample(POST_RECOVERY_FIFO_WAIT_MS) > 0) {
      Serial.println("[PULSEOX][RECOVERY] I2C bus recovery OK.");
      return true;
    }
  }

  if (reinitSensor()) {
    Serial.println("[PULSEOX][RECOVERY] Full reset OK.");
    return true;
  }

  Serial.println("[PULSEOX][RECOVERY] All attempts failed. Check hardware.");
  return false;
}

// ============================================================
//  Session / algorithm helpers
// ============================================================
static void resetSession() {
  state           = PO_IDLE;
  fingerLostCount = 0;
  spo2FillIndex   = 0;
  spo2            = -999;
  validSPO2       = 0;
  algoHR          = -999;
  validHR         = 0;
  hrRingCount     = 0;
  lastRR          = 0;
  iirSeeded       = false;

  if (vitalsMutex != nullptr &&
      xSemaphoreTake(vitalsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    vitals.hrValid       = false;
    vitals.spo2Valid     = false;
    vitals.heartRateBPM  = -1;
    vitals.spo2Percent   = -1;
    vitals.pulseOxStatus = "PLACE FINGER";
    xSemaphoreGive(vitalsMutex);
  }
}

static bool rrQuotientAccept(long currentRR, long previousRR) {
  if (previousRR <= 0) return true;
  float ratio = (float) currentRR / (float) previousRR;
  return ratio >= (1.0f - RR_DEVIATION_THRESHOLD) && ratio <= (1.0f + RR_DEVIATION_THRESHOLD);
}

static float iirSmooth(float newBPM) {
  if (!iirSeeded) { iirBPM = newBPM; iirSeeded = true; return iirBPM; }
  iirBPM = IIR_ALPHA * iirBPM + (1.0f - IIR_ALPHA) * newBPM;
  return iirBPM;
}

// IQR-filtered mean of the rolling BPM ring - same technique as the
// original computeFinalBPM(), just run continuously on a ring buffer
// instead of once per fixed 10 s session, so the display always has a
// live, outlier-rejected number instead of resetting every 10 s.
static float robustBpmFromRing() {
  if (hrRingCount < MIN_VALID_BEATS) return -1.0f;

  float sorted[HR_RING_LEN];
  for (int i = 0; i < hrRingCount; i++) sorted[i] = hrRing[i];
  for (int i = 0; i < hrRingCount - 1; i++)
    for (int j = i + 1; j < hrRingCount; j++)
      if (sorted[i] > sorted[j]) { float t = sorted[i]; sorted[i] = sorted[j]; sorted[j] = t; }

  int half   = hrRingCount / 2;
  int uStart = hrRingCount - half;
  float q1 = (half % 2 == 0) ? (sorted[half / 2 - 1] + sorted[half / 2]) / 2.0f : sorted[half / 2];
  float q3 = (half % 2 == 0) ? (sorted[uStart + half / 2 - 1] + sorted[uStart + half / 2]) / 2.0f : sorted[uStart + half / 2];

  float lf = q1 - 1.5f * (q3 - q1);
  float uf = q3 + 1.5f * (q3 - q1);

  float sum = 0; int count = 0;
  for (int i = 0; i < hrRingCount; i++)
    if (sorted[i] >= lf && sorted[i] <= uf) { sum += sorted[i]; count++; }

  if (count < MIN_VALID_BEATS) return -1.0f;
  return sum / (float) count;
}

static void pushBpm(float smoothed) {
  if (hrRingCount < HR_RING_LEN) {
    hrRing[hrRingCount++] = smoothed;
  } else {
    for (int i = 1; i < HR_RING_LEN; i++) hrRing[i - 1] = hrRing[i];
    hrRing[HR_RING_LEN - 1] = smoothed;
  }
}

// ============================================================
//  Public API
// ============================================================
bool pulseOxModuleInit() {
  bool ok = false;

  // MAX30102 physical wiring: SDA=GPIO21, SCL=GPIO20.
  // This is a dedicated controller; it does not reconfigure Wire/controller 0.
  if (!max30102BusStarted) {
    MAX30102Wire.begin(PULSEOX_SDA_PIN, PULSEOX_SCL_PIN, I2C_BUS_CLOCK_HZ);
    MAX30102Wire.setTimeOut(100);
    max30102BusStarted = true;
  }

  if (i2cMutex == nullptr) {
    Serial.println("[PULSEOX] ERROR: i2cMutex not created before pulseOxModuleInit().");
    return false;
  }

  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    ok = sensor.begin(MAX30102Wire, I2C_SPEED_STANDARD);
    if (ok) {
      // SparkFun MAX3010x: brightness, averaging, LED mode, sample rate,
      // pulse width, ADC range. Mode 2 enables RED + IR for SpO2.
      sensor.setup(60, 4, 2, 100, 411, 4096);
      sensor.setPulseAmplitudeRed(0x24);
      sensor.setPulseAmplitudeIR(0x24);
      sensor.setPulseAmplitudeGreen(0);
      sensor.clearFIFO();
    }
    xSemaphoreGive(i2cMutex);
  }

  lastIRUpdate = millis();
  if (ok) Serial.println("[PULSEOX] MAX30102 initialized on I2C1 (SDA=21, SCL=20).");
  else Serial.println("[PULSEOX] MAX30102 not detected on I2C1 (SDA=21, SCL=20).");
  return ok;
}

void pulseOxModuleTask(void* pvParameters) {
  (void) pvParameters;

  if (i2cMutex == nullptr || vitalsMutex == nullptr) {
    Serial.println("[PULSEOX] ERROR: mutexes must be created before task start.");
    vTaskDelete(nullptr);
    return;
  }

  lastIRUpdate = millis();
  for (;;) {
    uint32_t irValue = 0, redValue = 0;
    bool haveSample = false;

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      sensor.check();
      if (sensor.available()) {
        redValue   = sensor.getFIFORed();
        irValue    = sensor.getFIFOIR();
        sensor.nextSample();
        haveSample = true;
      }
      xSemaphoreGive(i2cMutex);
    }

    // A new FIFO sample, not a stale register value, is proof of sensor life.
    if (!haveSample) {
      if (millis() - lastIRUpdate > SENSOR_TIMEOUT_MS) {
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
          recoverSensor();
          xSemaphoreGive(i2cMutex);
        }
        resetSession();
        lastIRUpdate = millis();
      }
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    lastIRUpdate = millis();

    // ---- Finger debounce ----
    if (irValue < FINGER_THRESHOLD) {
      if (++fingerLostCount >= FINGER_LOST_CONFIRM_COUNT) resetSession();
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    fingerLostCount = 0;

    // ---- State machine ----
    switch (state) {
      case PO_IDLE:
        if (irValue > GOOD_SIGNAL_THRESHOLD) {
          fingerTime = millis();
          state = PO_STABILIZING;
          spo2FillIndex = 0;
          hrRingCount = 0;
          lastBeat = 0;
          lastRR = 0;
          iirSeeded = false;
          if (xSemaphoreTake(vitalsMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            vitals.pulseOxStatus = "STABILIZING";
            xSemaphoreGive(vitalsMutex);
          }
        }
        break;

      case PO_STABILIZING:
        if (millis() - fingerTime >= 2000) {
          state = PO_ACTIVE;
          if (xSemaphoreTake(vitalsMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            vitals.pulseOxStatus = "MEASURING";
            xSemaphoreGive(vitalsMutex);
          }
        }
        break;

      case PO_ACTIVE:
        if (!haveSample) break;   // only feed the algorithms genuine new FIFO samples

        // -- Beat-to-beat HR --
        if (checkForBeat((long) irValue)) {
          long now       = millis();
          long currentRR = now - lastBeat;
          lastBeat       = now;

          float instantBPM = 60000.0f / (float) currentRR;
          if (instantBPM >= BPM_MIN && instantBPM <= BPM_MAX &&
              rrQuotientAccept(currentRR, lastRR)) {
            lastRR = currentRR;
            float smoothed = iirSmooth(instantBPM);
            pushBpm(smoothed);

            float robust = robustBpmFromRing();
            if (robust > 0 && xSemaphoreTake(vitalsMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
              vitals.heartRateBPM = (int) round(robust);
              vitals.hrValid      = true;
              vitals.hrUpdatedMs  = millis();
              xSemaphoreGive(vitalsMutex);
            }
          }
        }

        // -- SpO2 sliding window (Maxim algorithm), fed by the same FIFO sample --
        redBuffer[spo2FillIndex] = redValue;
        irBuffer[spo2FillIndex]  = irValue;
        spo2FillIndex++;

        if (spo2FillIndex >= SPO2_BUFFER_LEN) {
          maxim_heart_rate_and_oxygen_saturation(
            irBuffer, SPO2_BUFFER_LEN, redBuffer,
            &spo2, &validSPO2, &algoHR, &validHR);

          if (xSemaphoreTake(vitalsMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (validSPO2 && spo2 >= 70 && spo2 <= 100) {
              vitals.spo2Percent   = spo2;
              vitals.spo2Valid     = true;
              vitals.spo2UpdatedMs = millis();
            } else {
              vitals.spo2Valid = false;
            }
            xSemaphoreGive(vitalsMutex);
          }
          Serial.print("[PULSEOX] SpO2 window computed -> valid=");
          Serial.print(validSPO2);
          Serial.print(" value=");
          Serial.println(spo2);

          // Slide window: drop oldest 25, keep 75, refill 25 next round
          for (int i = 25; i < SPO2_BUFFER_LEN; i++) {
            redBuffer[i - 25] = redBuffer[i];
            irBuffer[i - 25]  = irBuffer[i];
          }
          spo2FillIndex = 75;
        }
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
