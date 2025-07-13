#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <MPU9250_asukiaaa.h>
#include <SoftwareSerial.h>

// === nRF24L01 Setup ===
RF24 radio(9, 10); // CE, CSN

// === MPU9250 Setup ===
MPU9250_asukiaaa mpu;

// === GPS Setup ===
SoftwareSerial gpsSerial(2, 3); // RX, TX

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println(F("=== Initializing Components ==="));

  // --- nRF24L01 ---
  if (!radio.begin()) {
    Serial.println(F("❌ nRF24L01 not detected"));
  } else {
    Serial.println(F("✅ nRF24L01 communication OK"));
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
    radio.setRetries(5, 5);
    radio.openWritingPipe(0xF0F0F0F0E1LL);  // Dummy pipe
    radio.stopListening(); // TX mode
  }

  // --- MPU9250 ---
  Wire.begin();
  mpu.setWire(&Wire);
  mpu.beginAccel();
  mpu.beginGyro();
  mpu.beginMag();

  Serial.println(F("✅ MPU9250 init complete"));

  // --- GPS ---
  gpsSerial.begin(9600);
  Serial.println(F("✅ GPS serial started"));
}

void loop() {
  Serial.println(F("\n--- Status ---"));

  // --- nRF24L01 Test Write ---
  bool tx_result = radio.write("ping", 4);
  Serial.print("nRF24L01 Write: ");
  Serial.println(tx_result ? "✅ ACK received" : "❌ No ACK (expected if no receiver)");

  bool tx_ok, tx_fail, rx_ready;
  radio.whatHappened(tx_ok, tx_fail, rx_ready);
  Serial.print("TX OK: "); Serial.println(tx_ok);
  Serial.print("TX FAIL: "); Serial.println(tx_fail);
  Serial.print("RX Ready: "); Serial.println(rx_ready);

  // --- MPU9250 Read ---
  mpu.accelUpdate();
  mpu.gyroUpdate();
  mpu.magUpdate();

  Serial.print("Accel X: "); Serial.print(mpu.accelX(), 2);
  Serial.print(" Y: "); Serial.print(mpu.accelY(), 2);
  Serial.print(" Z: "); Serial.println(mpu.accelZ(), 2);

  Serial.print("Gyro X: "); Serial.print(mpu.gyroX(), 2);
  Serial.print(" Y: "); Serial.print(mpu.gyroY(), 2);
  Serial.print(" Z: "); Serial.println(mpu.gyroZ(), 2);

  Serial.print("Mag X: "); Serial.print(mpu.magX(), 2);
  Serial.print(" Y: "); Serial.print(mpu.magY(), 2);
  Serial.print(" Z: "); Serial.println(mpu.magZ(), 2);

  // --- GPS Read ---
  Serial.print("GPS: ");
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    Serial.write(c);
  }

  delay(1000);
}
