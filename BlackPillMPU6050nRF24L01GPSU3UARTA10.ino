#include <Wire.h>
#include <MPU6050_light.h>
#include <SPI.h>
#include <RF24.h>
#include <TinyGPS++.h>
#include <Arduino.h>

// 🧠 Interfaces
HardwareSerial MySerial2(USART2);  // Serial monitor via CH340 (TX2 = A2, RX2 = A3)
HardwareSerial GPSserial(USART1);  // GPS on A10 (TX1)

// 📦 Sensors
MPU6050 mpu(Wire);
TinyGPSPlus gps;

// 🛰️ nRF24L01 pins on STM32 Black Pill
#define CE_PIN  PA8
#define CSN_PIN PA9
RF24 radio(CE_PIN, CSN_PIN);  // CE, CSN

void setup() {
  MySerial2.begin(115200);
  delay(1000);
  MySerial2.println("🔧 Starting system setup...");

  // ✅ MPU6050 INIT
  Wire.begin();  // PB6 = SCL, PB7 = SDA
  delay(1000);
  if (mpu.begin() != 0) {
    MySerial2.println("❌ MPU6050 init failed.");
    while (1);
  }
  MySerial2.println("✅ MPU6050 initialized. Calibrating...");
  delay(1000);
  mpu.calcOffsets(true, true);
  MySerial2.println("✅ Calibration complete.");

  // ✅ nRF24L01 INIT
  if (!radio.begin()) {
    MySerial2.println("❌ nRF24L01 not detected!");
  } else {
    MySerial2.println("✅ nRF24L01 connected.");
    radio.setAutoAck(false);  // Disable ACK
    radio.setPALevel(RF24_PA_LOW);
    radio.setChannel(125);
    radio.openWritingPipe(0xF0F0F0F0E1LL);
    radio.stopListening();  // TX mode
  }

  // ✅ GPS INIT
  GPSserial.begin(9600);
  MySerial2.println("📡 GPS module ready.");
}

void loop() {
  // 🧭 MPU6050
  mpu.update();
  MySerial2.print("Pitch: ");
  MySerial2.print(mpu.getAngleX());
  MySerial2.print(" | Roll: ");
  MySerial2.print(mpu.getAngleY());
  MySerial2.print(" | Yaw: ");
  MySerial2.println(mpu.getAngleZ());

  // 🛰️ GPS
  while (GPSserial.available() > 0) {
    gps.encode(GPSserial.read());
  }

  if (gps.location.isUpdated()) {
    MySerial2.print("🌐 Lat: ");
    MySerial2.print(gps.location.lat(), 6);
    MySerial2.print(" | Lon: ");
    MySerial2.println(gps.location.lng(), 6);
  } else {
    MySerial2.println("⏳ Waiting for GPS fix...");
  }

  if (gps.date.isValid() && gps.time.isValid()) {
    MySerial2.print("📅 Date: ");
    MySerial2.print(gps.date.month());
    MySerial2.print("/");
    MySerial2.print(gps.date.day());
    MySerial2.print("/");
    MySerial2.println(gps.date.year());

    MySerial2.print("🕐 Time (UTC): ");
    MySerial2.print(gps.time.hour());
    MySerial2.print(":");
    MySerial2.print(gps.time.minute());
    MySerial2.print(":");
    MySerial2.println(gps.time.second());
  }

  // 🛰️ nRF24L01 Send test packet
  const char text[] = "Test";
  bool success = radio.write(&text, sizeof(text));
  MySerial2.println(success ? "📡 nRF24L01 send success." : "⚠️ nRF24L01 send failed.");

  delay(1000);
}
