#include <Arduino.h>
#include "soc/spi_reg.h"

#if CONFIG_IDF_TARGET_ESP32
  #include "esp32/rom/spi_flash.h"
#elif CONFIG_IDF_TARGET_ESP32S2  // ESP32-S2
  #include "esp32s2/rom/spi_flash.h"
#elif CONFIG_IDF_TARGET_ESP32S3  // ESP32-S3
  #include "esp32s3/rom/spi_flash.h"
#elif CONFIG_IDF_TARGET_ESP32C3  // ESP32-C3
  #include "esp32c3/rom/spi_flash.h"
#endif

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3
  #ifndef REG_SPI_BASE
  #define REG_SPI_BASE(i)     (DR_REG_SPI1_BASE + (((i)>1) ? (((i)* 0x1000) + 0x20000) : (((~(i)) & 1)* 0x1000 )))
  #endif // REG_SPI_BASE
#endif // TARGET


String ESP_getFlashChipMode(void) {
#if CONFIG_IDF_TARGET_ESP32S2
const uint32_t spi_ctrl = REG_READ(PERIPHS_SPI_FLASH_CTRL);
#else
const uint32_t spi_ctrl = REG_READ(SPI_CTRL_REG(0));
#endif
/* Not all of the following constants are already defined in older versions of spi_reg.h, so do it manually for now*/
if (spi_ctrl & BIT(24)) { //SPI_FREAD_QIO
    return F("QIO");
} else if (spi_ctrl & BIT(20)) { //SPI_FREAD_QUAD
    return F("QOUT");
} else if (spi_ctrl &  BIT(23)) { //SPI_FREAD_DIO
    return F("DIO");
} else if (spi_ctrl & BIT(14)) { // SPI_FREAD_DUAL
    return F("DOUT");
} else if (spi_ctrl & BIT(13)) { //SPI_FASTRD_MODE
    return F("Fast");
} else {
    return F("Slow");
}
return F("DOUT");
}

uint32_t ESP_getFlashChipId(void)
{
  uint32_t id = g_rom_flashchip.device_id;
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
    Serial.print("ESP32 XTAL FREQ: "); Serial.print(getXtalFrequencyMhz()); Serial.println(" MHz");
    Serial.print("ESP32 APB FREQ: "); Serial.print(getApbFrequency() / 1000000.0, 1); Serial.println(" MHz");
    Serial.print("ESP32 FLASH CHIP ID: "); Serial.println(ESP_getFlashChipId());
    Serial.print("ESP32 FLASH REAL SIZE: "); Serial.print(ESP_getFlashChipRealSize() / (1024.0 * 1024), 2); Serial.println(" MB");
    Serial.print("ESP32 FLASH SIZE (MAGIC BYTE): "); Serial.print(ESP.getFlashChipSize() / (1024.0 * 1024), 2); Serial.println(" MB");
    Serial.print("ESP32 FLASH REAL MODE: "); Serial.println(ESP_getFlashChipMode());
    Serial.print("ESP32 FLASH MODE (MAGIC BYTE): "); Serial.print(ESP.getFlashChipMode()); Serial.println(", 0=QIO, 1=QOUT, 2=DIO, 3=DOUT");
    Serial.print("ESP32 RAM SIZE: "); Serial.print(ESP.getHeapSize() / 1024.0, 2); Serial.println(" KB");
    Serial.print("ESP32 FREE RAM: "); Serial.print(ESP.getFreeHeap() / 1024.0, 2); Serial.println(" KB");
    Serial.print("ESP32 MAX RAM ALLOC: "); Serial.print(ESP.getMaxAllocHeap() / 1024.0, 2); Serial.println(" KB");
    Serial.print("ESP32 FREE PSRAM: "); Serial.print(ESP.getFreePsram() / 1024.0, 2); Serial.println(" KB");
}

void loop() {
}