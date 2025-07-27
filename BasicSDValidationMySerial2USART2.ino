#include <SPI.h>
#include <SdFat.h>
#include <Arduino.h>

// 💾 SD Card - SPI2 on STM32 Black Pill
#define SD_CS_PIN PB12
SPIClass SPI_2(2);  // SPI2 = PB13(SCK), PB14(MISO), PB15(MOSI)
SdFat sd;
File test;

// 🧠 Serial Monitor via CH340 (PA2 = TX2, PA3 = RX2)
HardwareSerial MySerial2(USART2);

void setup() {
  MySerial2.begin(115200);  // Start Serial Monitor via CH340
  delay(1000);
  MySerial2.println("🔍 SD Card Test via SPI2 + CH340");

  SPI_2.begin();
  MySerial2.println("📡 Starting SPI2...");

  SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(4), &SPI_2);
  MySerial2.println("⚙️ Configuring SD...");

  if (!sd.begin(config)) {
    MySerial2.println("❌ SD init failed.");
  } else {
    MySerial2.println("✅ SD init success.");
    test = sd.open("test.txt", FILE_WRITE);
    if (test) {
      test.println("Hello from STM32!");
      test.close();
      MySerial2.println("✅ File written to SD.");
    } else {
      MySerial2.println("❌ Failed to open file for writing.");
    }
  }
}

void loop() {
  // Nothing here for test
}
