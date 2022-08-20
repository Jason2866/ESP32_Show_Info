#include <Arduino.h>
#include "bootloader_flash.h"

uint32_t ESP_getFlashChipId(void)
{
  uint32_t id = bootloader_read_flash_id();
  id = ((id & 0xff) << 16) | ((id >> 16) & 0xff) | (id & 0xff00);
  return id;
}

uint32_t ESP_getFlashChipRealSize(void)
{
  uint32_t id = (ESP_getFlashChipId() >> 16) & 0xFF;
  return 2 << (id - 1);
}

void setup() {

  Serial.begin(115200);
  ps_malloc(550 * 1024);
  uint8_t * buf = (uint8_t*)malloc(500 * 1024);
  if (buf == NULL)
    Serial.println("can't allocate memory with malloc\n");

    Serial.print("ESP32 SDK: "); Serial.println(ESP.getSdkVersion());
    Serial.print("ESP32 CPU FREQ: "); Serial.print(getCpuFrequencyMhz()); Serial.println(" MHz");
    Serial.print("ESP32 APB FREQ: "); Serial.print(getApbFrequency() / 1000000.0, 1); Serial.println(" MHz");
    Serial.print("ESP32 FLASH CHIP ID: "); Serial.println(ESP_getFlashChipId());
    Serial.print("ESP32 FLASH REAL SIZE: "); Serial.print(ESP_getFlashChipRealSize() / (1024.0 * 1024), 2); Serial.println(" MB");
    Serial.print("ESP32 FLASH SIZE (MAGIC BYTE): "); Serial.print(ESP.getFlashChipSize() / (1024.0 * 1024), 2); Serial.println(" MB");
    Serial.print("ESP32 FLASH MODE: "); Serial.print(ESP.getFlashChipMode()); Serial.println(", 0=QIO, 1=QOUT, 2=DIO, 3=DOUT");
    Serial.print("ESP32 RAM SIZE: "); Serial.print(ESP.getHeapSize() / 1024.0, 2); Serial.println(" KB");
    Serial.print("ESP32 FREE RAM: "); Serial.print(ESP.getFreeHeap() / 1024.0, 2); Serial.println(" KB");
    Serial.print("ESP32 MAX RAM ALLOC: "); Serial.print(ESP.getMaxAllocHeap() / 1024.0, 2); Serial.println(" KB");
    Serial.print("ESP32 FREE PSRAM: "); Serial.print(ESP.getFreePsram() / 1024.0, 2); Serial.println(" KB");
}

void loop() {
}