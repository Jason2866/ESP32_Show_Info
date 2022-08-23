#include <Arduino.h>
#include "soc/spi_reg.h"
#include <soc/efuse_reg.h>

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

String GetDeviceHardware(void) {
  // https://www.espressif.com/en/products/socs

/*
Source: esp-idf esp_system.h and esptool
typedef enum {
    CHIP_ESP32   = 1,  //!< ESP32
    CHIP_ESP32S2 = 2,  //!< ESP32-S2
    CHIP_ESP32S3 = 4,  //!< ESP32-S3
    CHIP_ESP32C3 = 5,  //!< ESP32-C3
} esp_chip_model_t;
// Chip feature flags, used in esp_chip_info_t
#define CHIP_FEATURE_EMB_FLASH      BIT(0)      //!< Chip has embedded flash memory
#define CHIP_FEATURE_WIFI_BGN       BIT(1)      //!< Chip has 2.4GHz WiFi
#define CHIP_FEATURE_BLE            BIT(4)      //!< Chip has Bluetooth LE
#define CHIP_FEATURE_BT             BIT(5)      //!< Chip has Bluetooth Classic
// The structure represents information about the chip
typedef struct {
    esp_chip_model_t model;  //!< chip model, one of esp_chip_model_t
    uint32_t features;       //!< bit mask of CHIP_FEATURE_x feature flags
    uint8_t cores;           //!< number of CPU cores
    uint8_t revision;        //!< chip revision number
} esp_chip_info_t;
*/
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);

  uint32_t chip_model = chip_info.model;
  uint32_t chip_revision = chip_info.revision;
//  uint32_t chip_revision = ESP.getChipRevision();
  bool rev3 = (3 == chip_revision);
//  bool single_core = (1 == ESP.getChipCores());
  bool single_core = (1 == chip_info.cores);

  if (chip_model < 2) {  // ESP32
#ifdef CONFIG_IDF_TARGET_ESP32
/* esptool:
    def get_pkg_version(self):
        word3 = self.read_efuse(3)
        pkg_version = (word3 >> 9) & 0x07
        pkg_version += ((word3 >> 2) & 0x1) << 3
        return pkg_version
*/
    uint32_t chip_ver = REG_GET_FIELD(EFUSE_BLK0_RDATA3_REG, EFUSE_RD_CHIP_VER_PKG);
    uint32_t pkg_version = chip_ver & 0x7;

//    AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("HDW: ESP32 Model %d, Revision %d, Core %d, Package %d"), chip_info.model, chip_revision, chip_info.cores, chip_ver);

    switch (pkg_version) {
      case 0:
        if (single_core) { return F("ESP32-S0WDQ6"); }     // Max 240MHz, Single core, QFN 6*6
        else if (rev3)   { return F("ESP32-D0WDQ6-V3"); }  // Max 240MHz, Dual core, QFN 6*6
        else {             return F("ESP32-D0WDQ6"); }     // Max 240MHz, Dual core, QFN 6*6
      case 1:
        if (single_core) { return F("ESP32-S0WD"); }       // Max 160MHz, Single core, QFN 5*5, ESP32-SOLO-1, ESP32-DevKitC
        else if (rev3)   { return F("ESP32-D0WD-V3"); }    // Max 240MHz, Dual core, QFN 5*5, ESP32-WROOM-32E, ESP32_WROVER-E, ESP32-DevKitC
        else {             return F("ESP32-D0WD"); }       // Max 240MHz, Dual core, QFN 5*5, ESP32-WROOM-32D, ESP32_WROVER-B, ESP32-DevKitC
      case 2:              return F("ESP32-D2WD");         // Max 160MHz, Dual core, QFN 5*5, 2MB embedded flash
      case 3:
        if (single_core) { return F("ESP32-S0WD-OEM"); }   // Max 160MHz, Single core, QFN 5*5, Xiaomi Yeelight
        else {             return F("ESP32-D0WD-OEM"); }   // Max 240MHz, Dual core, QFN 5*5
      case 4:              return F("ESP32-U4WDH");        // Max 160MHz, Single core, QFN 5*5, 4MB embedded flash, ESP32-MINI-1, ESP32-DevKitM-1
      case 5:
        if (rev3)        { return F("ESP32-PICO-V3"); }    // Max 240MHz, Dual core, LGA 7*7, ESP32-PICO-V3-ZERO, ESP32-PICO-V3-ZERO-DevKit
        else {             return F("ESP32-PICO-D4"); }    // Max 240MHz, Dual core, LGA 7*7, 4MB embedded flash, ESP32-PICO-KIT
      case 6:              return F("ESP32-PICO-V3-02");   // Max 240MHz, Dual core, LGA 7*7, 8MB embedded flash, 2MB embedded PSRAM, ESP32-PICO-MINI-02, ESP32-PICO-DevKitM-2
    }
#endif  // CONFIG_IDF_TARGET_ESP32
    return F("ESP32");
  }
  else if (2 == chip_model) {  // ESP32-S2
#ifdef CONFIG_IDF_TARGET_ESP32S2
/* esptool:
    def get_pkg_version(self):
        num_word = 3
        block1_addr = self.EFUSE_BASE + 0x044
        word3 = self.read_reg(block1_addr + (4 * num_word))
        pkg_version = (word3 >> 21) & 0x0F
        return pkg_version
*/
    uint32_t chip_ver = REG_GET_FIELD(EFUSE_RD_MAC_SPI_SYS_3_REG, EFUSE_PKG_VERSION);
    uint32_t pkg_version = chip_ver & 0x7;
//    uint32_t pkg_version = esp_efuse_get_pkg_ver();

//    AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("HDW: ESP32 Model %d, Revision %d, Core %d, Package %d"), chip_info.model, chip_revision, chip_info.cores, chip_ver);

    switch (pkg_version) {
      case 0:              return F("ESP32-S2");           // Max 240MHz, Single core, QFN 7*7, ESP32-S2-WROOM, ESP32-S2-WROVER, ESP32-S2-Saola-1, ESP32-S2-Kaluga-1
      case 1:              return F("ESP32-S2FH2");        // Max 240MHz, Single core, QFN 7*7, 2MB embedded flash, ESP32-S2-MINI-1, ESP32-S2-DevKitM-1
      case 2:              return F("ESP32-S2FH4");        // Max 240MHz, Single core, QFN 7*7, 4MB embedded flash
      case 3:              return F("ESP32-S2FN4R2");      // Max 240MHz, Single core, QFN 7*7, 4MB embedded flash, 2MB embedded PSRAM, , ESP32-S2-MINI-1U, ESP32-S2-DevKitM-1U
    }
#endif  // CONFIG_IDF_TARGET_ESP32S2
    return F("ESP32-S2");
  }
  else if (4 == chip_model) {  // ESP32-S3
    return F("ESP32-S3");                                  // Max 240MHz, Dual core, QFN 7*7, ESP32-S3-WROOM-1, ESP32-S3-DevKitC-1
  }
  else if (5 == chip_model) {  // ESP32-C3
#ifdef CONFIG_IDF_TARGET_ESP32C3
/* esptool:
    def get_pkg_version(self):
        num_word = 3
        block1_addr = self.EFUSE_BASE + 0x044
        word3 = self.read_reg(block1_addr + (4 * num_word))
        pkg_version = (word3 >> 21) & 0x0F
        return pkg_version
*/
    uint32_t chip_ver = REG_GET_FIELD(EFUSE_RD_MAC_SPI_SYS_3_REG, EFUSE_PKG_VERSION);
    uint32_t pkg_version = chip_ver & 0x7;
//    uint32_t pkg_version = esp_efuse_get_pkg_ver();

//    AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("HDW: ESP32 Model %d, Revision %d, Core %d, Package %d"), chip_info.model, chip_revision, chip_info.cores, chip_ver);

    switch (pkg_version) {
      case 0:              return F("ESP32-C3");           // Max 160MHz, Single core, QFN 5*5, ESP32-C3-WROOM-02, ESP32-C3-DevKitC-02
      case 1:              return F("ESP32-C3FH4");        // Max 160MHz, Single core, QFN 5*5, 4MB embedded flash, ESP32-C3-MINI-1, ESP32-C3-DevKitM-1
    }
#endif  // CONFIG_IDF_TARGET_ESP32C3
    return F("ESP32-C3");
  }
  else if (6 == chip_model) {  // ESP32-S3(beta3)
    return F("ESP32-S3");
  }
  else if (7 == chip_model) {  // ESP32-C6(beta)
#ifdef CONFIG_IDF_TARGET_ESP32C6
/* esptool:
    def get_pkg_version(self):
        num_word = 3
        block1_addr = self.EFUSE_BASE + 0x044
        word3 = self.read_reg(block1_addr + (4 * num_word))
        pkg_version = (word3 >> 21) & 0x0F
        return pkg_version
*/
    uint32_t chip_ver = REG_GET_FIELD(EFUSE_RD_MAC_SPI_SYS_3_REG, EFUSE_PKG_VERSION);
    uint32_t pkg_version = chip_ver & 0x7;
//    uint32_t pkg_version = esp_efuse_get_pkg_ver();

//    AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("HDW: ESP32 Model %d, Revision %d, Core %d, Package %d"), chip_info.model, chip_revision, chip_info.cores, chip_ver);

    switch (pkg_version) {
      case 0:              return F("ESP32-C6");
    }
#endif  // CONFIG_IDF_TARGET_ESP32C6
    return F("ESP32-C6");
  }
  else if (10 == chip_model) {  // ESP32-H2
#ifdef CONFIG_IDF_TARGET_ESP32H2
/* esptool:
    def get_pkg_version(self):
        num_word = 3
        block1_addr = self.EFUSE_BASE + 0x044
        word3 = self.read_reg(block1_addr + (4 * num_word))
        pkg_version = (word3 >> 21) & 0x0F
        return pkg_version
*/
    uint32_t chip_ver = REG_GET_FIELD(EFUSE_RD_MAC_SPI_SYS_3_REG, EFUSE_PKG_VERSION);
    uint32_t pkg_version = chip_ver & 0x7;
//    uint32_t pkg_version = esp_efuse_get_pkg_ver();

//    AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("HDW: ESP32 Model %d, Revision %d, Core %d, Package %d"), chip_info.model, chip_revision, chip_info.cores, chip_ver);

    switch (pkg_version) {
      case 0:              return F("ESP32-H2");
    }
#endif  // CONFIG_IDF_TARGET_ESP32H2
    return F("ESP32-H2");
  }
  return F("ESP32");
}

String GetDeviceHardwareRevision(void) {
  // ESP32-S2
  // ESP32-D0WDQ6 rev.1
  // ESP32-C3 rev.2
  // ESP32-C3 rev.3
  String result = GetDeviceHardware();   // ESP32-C3

  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  char revision[10] = { 0 };
  if (chip_info.revision) {
    snprintf_P(revision, sizeof(revision), PSTR(" rev.%d"), chip_info.revision);
  }
  result += revision;                    // ESP32-C3 rev.3

  return result;
}


void setup() {

  Serial.begin(115200);
  ps_malloc(550 * 1024);
  uint8_t * buf = (uint8_t*)malloc(500 * 1024);
  if (buf == NULL)
    Serial.println("can't allocate memory with malloc\n");

    Serial.print("ESP32 SDK: "); Serial.println(ESP.getSdkVersion());
    Serial.print("ESP32 DEVICE: "); Serial.println(GetDeviceHardwareRevision());
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