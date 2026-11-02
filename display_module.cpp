#include "display_module.h"
#include "config.h"
#include "shared_data.h"

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <math.h>

// -----------------------------------------------------------------------------
// PHMS ST7796 display module
// ESP32-S3 / ST7796 480x320 landscape
// ECG waveform data is supplied by ecg_module.cpp through shared_data.h.
// -----------------------------------------------------------------------------

// Set to 1 ONLY to test the TFT waveform renderer without ADS1293 acquisition.
// Set to 0 for normal ECG operation.
#define ECG_DISPLAY_TEST_MODE 0

// ADS1293 runs at about 853 samples/s. Draw one point for every five samples,
// giving approximately 171 horizontal pixels/s and a ~2.4 s sweep across 414 px.
#define ECG_DISPLAY_DECIMATION 5U

static constexpr int16_t SCREEN_WIDTH_PIXELS = 480;
static constexpr int16_t SCREEN_HEIGHT_PIXELS = 320;
static constexpr int16_t GRAPH_X = 56;
static constexpr int16_t GRAPH_WIDTH = 414;
static constexpr int16_t GRAPH_TOPS[3] = {55, 127, 199};
static constexpr int16_t GRAPH_HEIGHT = 60;
static constexpr int16_t GRAPH_CENTER_OFFSET = 15;

static constexpr uint16_t COLOR_BLACK = 0x0000;
static constexpr uint16_t COLOR_WHITE = 0xFFFF;
static constexpr uint16_t COLOR_BLUE = 0x001F;
static constexpr uint16_t COLOR_GRID = 0x39E7;
static constexpr uint16_t COLOR_BASELINE = 0x2104;
static constexpr uint16_t TRACE_COLORS[3] = {0x07E0, 0x07FF, 0xFFE0};
static const char *const LEAD_NAMES[3] = {"I", "II", "III"};

static Arduino_DataBus *displayBus = nullptr;
static Arduino_GFX *gfx = nullptr;

static uint32_t lastReadIndex = 0;
static uint32_t displaySampleCounter = 0;
static int16_t xPosition = 0;
static int16_t previousY[3] = {-1, -1, -1};
static bool displayReady = false;
static uint32_t lastVitalsRefresh = 0;
static uint32_t lastStatusRefresh = 0;

// RAII lock for the SPI bus shared with the ADS1293 (see ecg_module.cpp).
struct SpiLock {
    SpiLock()  { if (spiMutex != nullptr) xSemaphoreTake(spiMutex, portMAX_DELAY); }
    ~SpiLock() { if (spiMutex != nullptr) xSemaphoreGive(spiMutex); }
};

#if ECG_DISPLAY_TEST_MODE
static uint32_t testSample = 0;
#endif

static void drawStaticScreen() {
    gfx->fillScreen(COLOR_BLACK);

    gfx->fillRect(0, 0, SCREEN_WIDTH_PIXELS, 38, COLOR_BLUE);
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(12, 10);
    gfx->print("PHMS | ECG + PULSE OX");
    gfx->drawFastHLine(0, 39, SCREEN_WIDTH_PIXELS, 0x7BEF);

    gfx->setTextSize(1);
    for (int lead = 0; lead < 3; ++lead) {
        const int top = GRAPH_TOPS[lead];
        gfx->setTextColor(TRACE_COLORS[lead]);
        gfx->setCursor(8, top - 12);
        gfx->printf("LEAD %s", LEAD_NAMES[lead]);

        gfx->drawRect(GRAPH_X - 1, top - 15, GRAPH_WIDTH + 2,
                      GRAPH_HEIGHT, COLOR_GRID);
        gfx->drawFastHLine(GRAPH_X, top + GRAPH_CENTER_OFFSET,
                           GRAPH_WIDTH, COLOR_BASELINE);
    }

    gfx->drawFastHLine(0, 275, SCREEN_WIDTH_PIXELS, 0x7BEF);

    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(12, 286);
    gfx->print("HR");
    gfx->setCursor(178, 286);
    gfx->print("SpO2");

    gfx->setTextSize(1);
    gfx->setCursor(12, 310);
    gfx->print("BPM");
    gfx->setCursor(178, 310);
    gfx->print("OXYGEN");

    gfx->setCursor(350, 286);
    gfx->print("GPS");
}

void displayModuleInit() {
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    // Touch controller reset and I2C setup. This is the touch bus only.
    pinMode(TP_RST, OUTPUT);
    digitalWrite(TP_RST, LOW);
    delay(20);
    digitalWrite(TP_RST, HIGH);
    delay(250);
    pinMode(TP_INT, INPUT_PULLUP);

    // Do not change the MAX30102's dedicated I2C controller from this module.
    Wire.begin(TP_SDA, TP_SCL);
    Wire.setClock(I2C_BUS_CLOCK_HZ);

    displayBus = new Arduino_ESP32SPI(
        TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, TFT_MISO
    );
    gfx = new Arduino_ST7796(displayBus, TFT_RST, 0, true, 320, 480);

    if (gfx == nullptr || !gfx->begin(40000000)) {
        Serial.println("[DISPLAY] ERROR: ST7796 initialization failed");
        displayReady = false;
        return;
    }

    gfx->setRotation(1);  // Landscape: 480 x 320
    drawStaticScreen();

    // Start reading from the current producer position. New samples are drawn
    // as soon as the ECG task publishes them.
    __sync_synchronize();
    lastReadIndex = ecgWaveWriteIdx;
    xPosition = 0;
    previousY[0] = previousY[1] = previousY[2] = -1;
    displayReady = true;

    Serial.println("[DISPLAY] ST7796 ready; ECG waveform renderer started");
#if ECG_DISPLAY_TEST_MODE
    Serial.println("[DISPLAY] WARNING: synthetic ECG test mode is enabled");
#endif
}

// shared_data.h defines waveform values as tenths of millivolts.
// Keep the trace centered and clipped to its own lead box.
static int16_t mapEcgToY(int16_t sample, int lead) {
    const int centerY = GRAPH_TOPS[lead] + GRAPH_CENTER_OFFSET;
    const float pixelsPerTenthMillivolt = 2.0f;   // 20 px/mV
    const int y = centerY - (int)lroundf(sample * pixelsPerTenthMillivolt);
    const int minY = GRAPH_TOPS[lead] - 14;
    const int maxY = GRAPH_TOPS[lead] + 44;
    return (int16_t)constrain(y, minY, maxY);
}

static void drawVitals() {
    const VitalsData v = getVitalsSnapshot();

    // Lock the SPI bus per field (not for the whole function) so the ECG
    // task is never blocked long enough to drop conversions.
    {
        SpiLock lock;
        gfx->fillRect(55, 282, 110, 37, COLOR_BLACK);
        gfx->setTextSize(3);
        gfx->setTextColor(v.hrValid ? 0x07E0 : 0xFBE0);
        gfx->setCursor(55, 282);
        if (v.hrValid) gfx->printf("%d", v.heartRateBPM);
        else gfx->print("--");
    }
    {
        SpiLock lock;
        gfx->fillRect(220, 282, 110, 37, COLOR_BLACK);
        gfx->setTextSize(3);
        gfx->setTextColor(v.spo2Valid ? 0x07E0 : 0xFBE0);
        gfx->setCursor(220, 282);
        if (v.spo2Valid) gfx->printf("%d%%", v.spo2Percent);
        else gfx->print("--");
    }
    {
        SpiLock lock;
        gfx->setTextSize(1);
        gfx->setTextColor(COLOR_WHITE);
        gfx->fillRect(350, 282, 125, 36, COLOR_BLACK);
        gfx->setCursor(350, 286);
        gfx->printf("GPS %s", v.gpsFix ? "FIX" : "NO FIX");
        gfx->setCursor(350, 301);
        gfx->printf("Satellites: %d", v.gpsSatellites);
    }
}

static void drawStatusLine() {
    const VitalsData v = getVitalsSnapshot();
    SpiLock lock;
    gfx->fillRect(8, 250, 465, 16, COLOR_BLACK);
    gfx->setTextSize(1);
    gfx->setTextColor(0x7BEF);
    gfx->setCursor(12, 254);
    gfx->printf("ECG: %s  |  PULSE: %s",
                v.ecgStatus.c_str(), v.pulseOxStatus.c_str());
}

static void drawOneEcgColumn(const int16_t samples[3]) {
    const int16_t x = GRAPH_X + xPosition;
    SpiLock lock;

    // Erase only the current vertical column inside the three waveform boxes.
    // This avoids clearing the labels, status line, or vital-sign panel.
    for (int lead = 0; lead < 3; ++lead) {
        gfx->drawFastVLine(x, GRAPH_TOPS[lead] - 14,
                           GRAPH_HEIGHT - 2, COLOR_BLACK);
    }

    for (int lead = 0; lead < 3; ++lead) {
        // Restore the baseline pixel the column erase removed.
        gfx->drawPixel(x, GRAPH_TOPS[lead] + GRAPH_CENTER_OFFSET, COLOR_BASELINE);
        const int16_t y = mapEcgToY(samples[lead], lead);
        if (previousY[lead] >= 0) {
            gfx->drawLine(x - 1, previousY[lead], x, y, TRACE_COLORS[lead]);
        } else {
            gfx->drawPixel(x, y, TRACE_COLORS[lead]);
        }
        previousY[lead] = y;
    }

    ++xPosition;
    if (xPosition >= GRAPH_WIDTH) {
        xPosition = 0;
        // Do not connect the right edge to the left edge on the next sweep.
        previousY[0] = previousY[1] = previousY[2] = -1;
    }
}

static void drawWaveforms() {
    if (!displayReady) return;

    // Read the producer index after its release barrier. The producer writes
    // all three samples before incrementing ecgWaveWriteIdx.
    __sync_synchronize();
    const uint32_t head = ecgWaveWriteIdx;
    uint32_t pending = head - lastReadIndex;

    if (pending == 0) return;

    // If the display fell behind, discard overwritten samples and resume from
    // the oldest sample that can still exist in the circular buffer.
    if (pending > ECG_WAVE_BUF_LEN) {
        lastReadIndex = head - ECG_WAVE_BUF_LEN;
        pending = ECG_WAVE_BUF_LEN;
        previousY[0] = previousY[1] = previousY[2] = -1;
        Serial.println("[DISPLAY] ECG consumer overrun; resynchronizing buffer");
    }

    for (uint32_t offset = 0; offset < pending; ++offset) {
        const uint32_t absoluteIndex = lastReadIndex + offset;
        const uint32_t bufferIndex = absoluteIndex % ECG_WAVE_BUF_LEN;

        // Advance through every acquired sample, but render only every Nth
        // sample. This matches the physical sample rate to TFT pixel speed.
        if ((displaySampleCounter++ % ECG_DISPLAY_DECIMATION) != 0) {
            continue;
        }

        const int16_t sampleSet[3] = {
            ecgWaveI[bufferIndex],
            ecgWaveII[bufferIndex],
            ecgWaveIII[bufferIndex]
        };
        drawOneEcgColumn(sampleSet);
    }

    lastReadIndex = head;
}

#if ECG_DISPLAY_TEST_MODE
// Artificial ECG-like signal for display-only diagnostics. This is NOT a
// physiological signal and must never be used for medical interpretation.
static void generateSyntheticDisplaySamples() {
    const float phase = fmodf((float)testSample * 0.035f, 2.0f * PI);
    float value = 0.0f;

    // P wave
    if (phase >= 0.20f && phase < 0.48f) {
        value = 2.0f * sinf((phase - 0.20f) * PI / 0.28f);
    }
    // QRS complex
    else if (phase >= 0.90f && phase < 1.00f) {
        value = -3.0f;
    } else if (phase >= 1.00f && phase < 1.14f) {
        value = 18.0f * sinf((phase - 1.00f) * PI / 0.14f);
    } else if (phase >= 1.14f && phase < 1.24f) {
        value = -4.0f;
    }
    // T wave
    else if (phase >= 1.55f && phase < 2.15f) {
        value = 3.0f * sinf((phase - 1.55f) * PI / 0.60f);
    }

    const uint32_t index = ecgWaveWriteIdx % ECG_WAVE_BUF_LEN;
    ecgWaveI[index] = (int16_t)(value * 10.0f);
    ecgWaveII[index] = (int16_t)(value * 8.0f);
    ecgWaveIII[index] = (int16_t)(-value * 2.0f);
    __sync_synchronize();
    ++ecgWaveWriteIdx;
    ++testSample;
}
#endif

void displayModuleTask(void *parameter) {
    (void)parameter;
    if (!displayReady) {
        Serial.println("[DISPLAY] Task stopped: display initialization failed");
        vTaskDelete(nullptr);
        return;
    }

    Serial.println("[DISPLAY] Task running");

    uint32_t lastDebug = millis();
    uint32_t lastDebugIndex = ecgWaveWriteIdx;

    for (;;) {
#if ECG_DISPLAY_TEST_MODE
        // Test mode must be used with the ECG acquisition task disabled.
        // About 200 synthetic samples/s; renderer decimates them for display.
        for (int i = 0; i < 2; ++i) generateSyntheticDisplaySamples();
#endif
        drawWaveforms();

        const uint32_t now = millis();
        if (now - lastVitalsRefresh >= 250) {
            drawVitals();
            lastVitalsRefresh = now;
        }
        if (now - lastStatusRefresh >= 1000) {
            drawStatusLine();
            lastStatusRefresh = now;
        }
        if (now - lastDebug >= 1000) {
            const uint32_t currentIndex = ecgWaveWriteIdx;
            Serial.printf("[DISPLAY DEBUG] writeIdx=%lu, newSamples=%lu, x=%d\n",
                          (unsigned long)currentIndex,
                          (unsigned long)(currentIndex - lastDebugIndex),
                          (int)xPosition);
            lastDebugIndex = currentIndex;
            lastDebug = now;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
