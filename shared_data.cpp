#include "shared_data.h"
VitalsData vitals;
SemaphoreHandle_t vitalsMutex=nullptr;
SemaphoreHandle_t i2cMutex=nullptr;
volatile int16_t ecgWaveI[ECG_WAVE_BUF_LEN]={0};
volatile int16_t ecgWaveII[ECG_WAVE_BUF_LEN]={0};
volatile int16_t ecgWaveIII[ECG_WAVE_BUF_LEN]={0};
volatile uint32_t ecgWaveWriteIdx=0;
VitalsData getVitalsSnapshot(){
  VitalsData v;
  if(xSemaphoreTake(vitalsMutex,pdMS_TO_TICKS(20))==pdTRUE){
    v=vitals; xSemaphoreGive(vitalsMutex);
  }
  return v;
}
