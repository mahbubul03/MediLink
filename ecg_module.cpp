#include "ecg_module.h"
#include "config.h"
#include "shared_data.h"

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ADS1293 registers
#define REG_CONFIG        0x00
#define REG_FLEX_CH1      0x01
#define REG_FLEX_CH2      0x02
#define REG_CMDET_EN      0x0A
#define REG_RLD_CN        0x0C
#define REG_OSC_CN        0x12
#define REG_AFE_SHDN      0x14
#define REG_R2_RATE       0x21
#define REG_R3_RATE_CH1   0x22
#define REG_R3_RATE_CH2   0x23
#define REG_DRDYB_SRC     0x27
#define REG_CH_CNFG       0x2F
#define REG_ECG_DATA_CH1  0x37
#define REG_REVID         0x40

// Set to 1 only when you need a per-sample CSV stream.
// At 853 SPS, continuous serial printing can burden the acquisition task.
#define ECG_STREAM_CSV 0

static constexpr uint32_t ECG_SPI_HZ = 2000000;
static constexpr uint32_t ECG_DIAG_INTERVAL_MS = 1000;

// Values retained from the user's standalone ECG v3 conversion.
static constexpr int32_t ADC_MID = 6075000;
static constexpr double UV_PER_LSB =
    (2.0 * 2.4 / 3.5) / 12150000.0 * 1e6;

// The ADS1293 is configured for 25600 / (5 * 6) = 853.333 SPS.
static constexpr float ECG_FS_HZ = 25600.0f / 30.0f;

static SPIClass ecgSPI(FSPI);
static TaskHandle_t ecgTaskHandle = nullptr;
static portMUX_TYPE ecgIsrMux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint32_t drdyInterruptCount = 0;
static volatile uint32_t notificationBatches = 0;

static uint32_t successfulSamples = 0;
static uint32_t invalidSamples = 0;
static uint32_t missedConversions = 0;
static uint32_t sampleSequence = 0;
static uint32_t lastDiagnosticMs = 0;

static float baseline1 = 0.0f;
static float baseline2 = 0.0f;
static float lowpass1 = 0.0f;
static float lowpass2 = 0.0f;
static bool filterInitialized = false;

// -----------------------------------------------------------------------------
// SPI access
// -----------------------------------------------------------------------------
static inline void ecgSelect() {
    digitalWrite(ECG_CS, LOW);
}

static inline void ecgDeselect() {
    digitalWrite(ECG_CS, HIGH);
}

static void adsWriteRegister(uint8_t reg, uint8_t value) {
    ecgSPI.beginTransaction(SPISettings(ECG_SPI_HZ, MSBFIRST, SPI_MODE0));
    ecgSelect();
    ecgSPI.transfer(reg & 0x7F);
    ecgSPI.transfer(value);
    ecgDeselect();
    ecgSPI.endTransaction();
}

static uint8_t adsReadRegister(uint8_t reg) {
    ecgSPI.beginTransaction(SPISettings(ECG_SPI_HZ, MSBFIRST, SPI_MODE0));
    ecgSelect();
    ecgSPI.transfer(reg | 0x80);
    const uint8_t value = ecgSPI.transfer(0x00);
    ecgDeselect();
    ecgSPI.endTransaction();
    return value;
}

static void adsReadBurst(uint8_t firstRegister, uint8_t *destination, size_t length) {
    ecgSPI.beginTransaction(SPISettings(ECG_SPI_HZ, MSBFIRST, SPI_MODE0));
    ecgSelect();
    ecgSPI.transfer(firstRegister | 0x80);

    for (size_t i = 0; i < length; ++i) {
        destination[i] = ecgSPI.transfer(0x00);
    }

    ecgDeselect();
    ecgSPI.endTransaction();
}

// -----------------------------------------------------------------------------
// Data-ready interrupt
// Important: use the FreeRTOS notification count, not a single bool flag.
// A bool can collapse multiple DRDY events while the task is busy.
// -----------------------------------------------------------------------------
static void IRAM_ATTR ecgDataReadyISR() {
    portENTER_CRITICAL_ISR(&ecgIsrMux);
    ++drdyInterruptCount;
    portEXIT_CRITICAL_ISR(&ecgIsrMux);

    BaseType_t higherPriorityTaskWoken = pdFALSE;
    TaskHandle_t task = ecgTaskHandle;

    if (task != nullptr) {
        vTaskNotifyGiveFromISR(task, &higherPriorityTaskWoken);
    }

    if (higherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

// -----------------------------------------------------------------------------
// ADS1293 initialization
// -----------------------------------------------------------------------------
bool ecgModuleInit() {
    pinMode(ECG_CS, OUTPUT);
    ecgDeselect();

    pinMode(ECG_DRDY, INPUT_PULLUP);
    pinMode(ECG_ALARM, INPUT_PULLUP);

    ecgSPI.begin(ECG_SCLK, ECG_MISO, ECG_MOSI, ECG_CS);
    delay(5);

    // Put device in standby before configuring it.
    adsWriteRegister(REG_CONFIG, 0x00);
    delay(10);

    const uint8_t revision = adsReadRegister(REG_REVID);
    Serial.printf("[ECG] ADS1293 REVID=0x%02X\n", revision);

    if (revision == 0x00 || revision == 0xFF) {
        Serial.println("[ECG] ERROR: ADS1293 not responding. Check power, GND, SPI and CS.");
        return false;
    }

    // Same register values as the user's standalone MediLink ECG v3.
    // CH1 = IN2 - IN1 (Lead I); CH2 = IN3 - IN1 (Lead II).
    const uint8_t registerTable[][2] = {
        {REG_FLEX_CH1,    0x11},
        {REG_FLEX_CH2,    0x19},
        {REG_CMDET_EN,    0x07},
        {REG_RLD_CN,      0x04},
        {REG_OSC_CN,      0x04},
        {REG_AFE_SHDN,    0x24},
        {REG_R2_RATE,     0x02},
        {REG_R3_RATE_CH1, 0x02},
        {REG_R3_RATE_CH2, 0x02},
        {REG_DRDYB_SRC,   0x08},
        {REG_CH_CNFG,     0x30}
    };

    for (size_t i = 0; i < sizeof(registerTable) / sizeof(registerTable[0]); ++i) {
        adsWriteRegister(registerTable[i][0], registerTable[i][1]);
    }

    // Verify the configuration was accepted.
    uint8_t mismatches = 0;
    for (size_t i = 0; i < sizeof(registerTable) / sizeof(registerTable[0]); ++i) {
        const uint8_t actual = adsReadRegister(registerTable[i][0]);
        if (actual != registerTable[i][1]) {
            ++mismatches;
            Serial.printf(
                "[ECG] WARNING reg 0x%02X: wrote 0x%02X, read 0x%02X\n",
                registerTable[i][0], registerTable[i][1], actual
            );
        }
    }

    Serial.printf("[ECG] Register verification: %u mismatch(es)\n", mismatches);

    // Start conversions.
    adsWriteRegister(REG_CONFIG, 0x01);
    delay(50);

    // Clear any pending task notification before enabling DRDY interrupts.
    if (ecgTaskHandle != nullptr) {
        ulTaskNotifyTake(pdTRUE, 0);
    }

    attachInterrupt(digitalPinToInterrupt(ECG_DRDY), ecgDataReadyISR, FALLING);

    if (vitalsMutex != nullptr &&
        xSemaphoreTake(vitalsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        vitals.ecgStatus = "ACQUIRING";
        xSemaphoreGive(vitalsMutex);
    }

    Serial.printf(
        "[ECG] Acquisition started: %.2f SPS, DRDY=GPIO%d, CS=GPIO%d, SPI=%lu Hz MODE0\n",
        ECG_FS_HZ, ECG_DRDY, ECG_CS, (unsigned long)ECG_SPI_HZ
    );
    Serial.printf(
        "[ECG] Raw ADC read: registers 0x%02X-0x%02X, 6 bytes\n",
        REG_ECG_DATA_CH1, REG_ECG_DATA_CH1 + 5
    );

    return true;
}

// -----------------------------------------------------------------------------
// Convert raw ADC codes, filter, and publish to the shared waveform buffers.
// Buffer values are stored as tenths of millivolts, matching display_module.cpp.
// -----------------------------------------------------------------------------
static void publishEcgSample(
    float channel1Microvolts,
    float channel2Microvolts,
    uint32_t sequence,
    int64_t timestampUs
) {
    const float channel1mV = channel1Microvolts / 1000.0f;
    const float channel2mV = channel2Microvolts / 1000.0f;

    if (!filterInitialized) {
        baseline1 = channel1mV;
        baseline2 = channel2mV;
        lowpass1 = 0.0f;
        lowpass2 = 0.0f;
        filterInitialized = true;
    }

    // Approximate 0.5 Hz baseline tracking.
    baseline1 += 0.0018f * (channel1mV - baseline1);
    baseline2 += 0.0018f * (channel2mV - baseline2);

    const float highpass1 = channel1mV - baseline1;
    const float highpass2 = channel2mV - baseline2;

    // Approximate 40 Hz low-pass at 853.33 samples/second.
    lowpass1 += 0.24f * (highpass1 - lowpass1);
    lowpass2 += 0.24f * (highpass2 - lowpass2);

    const float leadI = lowpass1;
    const float leadII = lowpass2;
    const float leadIII = leadII - leadI;

    // Publish all three samples before advancing the write index.
    const uint32_t writeIndex = ecgWaveWriteIdx;
    const uint32_t bufferIndex = writeIndex % ECG_WAVE_BUF_LEN;

    ecgWaveI[bufferIndex] =
        (int16_t)constrain(leadI * 10.0f, -30000.0f, 30000.0f);
    ecgWaveII[bufferIndex] =
        (int16_t)constrain(leadII * 10.0f, -30000.0f, 30000.0f);
    ecgWaveIII[bufferIndex] =
        (int16_t)constrain(leadIII * 10.0f, -30000.0f, 30000.0f);

    // Release-publish: display task sees the index only after data is written.
    __sync_synchronize();
    ecgWaveWriteIdx = writeIndex + 1;

#if ECG_STREAM_CSV
    Serial.printf(
        "D,%lu,%lld,%.3f,%.3f,%.3f,%u,0\n",
        (unsigned long)sequence,
        (long long)timestampUs,
        leadI, leadII, leadIII,
        (unsigned)(digitalRead(ECG_ALARM) == LOW)
    );
#endif

    (void)sequence;
    (void)timestampUs;
}

// -----------------------------------------------------------------------------
// ECG acquisition task
// -----------------------------------------------------------------------------
void ecgModuleTask(void *parameter) {
    (void)parameter;

    ecgTaskHandle = xTaskGetCurrentTaskHandle();

    Serial.println("[ECG] Acquisition task running.");

    for (;;) {
        // pdTRUE clears the accumulated notification count and returns it.
        // If multiple DRDY edges occurred, the ADC output registers contain
        // the newest conversion, not a FIFO of all historical conversions.
        const uint32_t pendingNotifications =
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        if (pendingNotifications == 0) {
            // Safety-net polling in case an interrupt edge was missed.
            if (digitalRead(ECG_DRDY) != LOW) {
                if (millis() - lastDiagnosticMs >= ECG_DIAG_INTERVAL_MS) {
                    lastDiagnosticMs = millis();

                    uint32_t irqCount;
                    portENTER_CRITICAL(&ecgIsrMux);
                    irqCount = drdyInterruptCount;
                    portEXIT_CRITICAL(&ecgIsrMux);

                    Serial.printf(
                        "[ECG DEBUG] DRDY IRQ=%lu, samples=%lu, invalid=%lu, missed=%lu, pin=%d\n",
                        (unsigned long)irqCount,
                        (unsigned long)successfulSamples,
                        (unsigned long)invalidSamples,
                        (unsigned long)missedConversions,
                        digitalRead(ECG_DRDY)
                    );
                }
                continue;
            }
        }

        if (pendingNotifications > 1) {
            missedConversions += pendingNotifications - 1;
        }

        uint8_t raw[6] = {0};
        const int drdyBeforeRead = digitalRead(ECG_DRDY);
        adsReadBurst(REG_ECG_DATA_CH1, raw, sizeof(raw));
        const int drdyAfterRead = digitalRead(ECG_DRDY);

        // Print only the first 10 raw conversions. This is diagnostic output;
        // it does not alter the ADC data or the sample-validation logic.
        static uint8_t rawDebugCount = 0;
        if (rawDebugCount < 10) {
            Serial.printf(
                "[ECG RAW %u] %02X %02X %02X | %02X %02X %02X | DRDY before=%d after=%d\n",
                (unsigned)(rawDebugCount + 1),
                raw[0], raw[1], raw[2],
                raw[3], raw[4], raw[5],
                drdyBeforeRead, drdyAfterRead
            );
            ++rawDebugCount;
        }

        const uint32_t code1 =
            ((uint32_t)raw[0] << 16) |
            ((uint32_t)raw[1] << 8) |
            (uint32_t)raw[2];

        const uint32_t code2 =
            ((uint32_t)raw[3] << 16) |
            ((uint32_t)raw[4] << 8) |
            (uint32_t)raw[5];

        // Reject only clearly invalid all-zero/all-one reads. Do not reject
        // a valid channel merely because the other channel is at a rail.
        if ((code1 == 0x000000UL && code2 == 0x000000UL) ||
            (code1 == 0xFFFFFFUL && code2 == 0xFFFFFFUL)) {
            ++invalidSamples;
            if (invalidSamples <= 10) {
                Serial.printf(
                    "[ECG INVALID] CH1=0x%06lX CH2=0x%06lX (both channels at %s)\n",
                    (unsigned long)code1,
                    (unsigned long)code2,
                    (code1 == 0x000000UL) ? "zero" : "0xFFFFFF"
                );
            }
            continue;
        }

        // ADS1293 conversion retained from standalone ECG v3:
        // offset-binary code centered at ADC_MID.
        const double channel1uV =
            ((int32_t)code1 - ADC_MID) * UV_PER_LSB;
        const double channel2uV =
            ((int32_t)code2 - ADC_MID) * UV_PER_LSB;

        const int64_t timestampUs = esp_timer_get_time();

        publishEcgSample(
            (float)channel1uV,
            (float)channel2uV,
            sampleSequence,
            timestampUs
        );

        ++sampleSequence;
        ++successfulSamples;

        // Update diagnostic status once per second, not once per sample.
        if (millis() - lastDiagnosticMs >= ECG_DIAG_INTERVAL_MS) {
            lastDiagnosticMs = millis();

            uint32_t irqCount;
            portENTER_CRITICAL(&ecgIsrMux);
            irqCount = drdyInterruptCount;
            portEXIT_CRITICAL(&ecgIsrMux);

            Serial.printf(
                "[ECG DEBUG] DRDY IRQ=%lu, samples=%lu, invalid=%lu, missed=%lu, writeIdx=%lu, pin=%d\n",
                (unsigned long)irqCount,
                (unsigned long)successfulSamples,
                (unsigned long)invalidSamples,
                (unsigned long)missedConversions,
                (unsigned long)ecgWaveWriteIdx,
                digitalRead(ECG_DRDY)
            );
        }
    }
}
