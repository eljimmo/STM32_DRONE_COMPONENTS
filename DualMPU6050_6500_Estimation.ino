#include <Wire.h>
#include <math.h>

// ---- I2C addresses ----
#define MPU6050_ADDR 0x68   // AD0 = GND
#define MPU6500_ADDR 0x69   // AD0 = 3V3

// ---- Common registers (MPU60x0/65x0 family) ----
#define REG_PWR_MGMT_1   0x6B
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H  0x43
#define REG_WHO_AM_I     0x75

// ---- Config ----
const uint8_t GYRO_FS_SEL  = 0b11;  // ±2000 dps
const uint8_t ACC_FS_SEL   = 0b01;  // ±4 g
const uint8_t DLPF_CFG     = 0b011; // ~44 Hz bandwidth

float gyroScale;   // dps/LSB
float accelScale;  // g/LSB

// Bias (per-IMU gyro)
float gBias1x=0, gBias1y=0, gBias1z=0; // for MPU6050
float gBias2x=0, gBias2y=0, gBias2z=0; // for MPU6500

// Complementary filter state (radians)
float roll=0, pitch=0, yaw=0;
float alpha = 0.98f;

unsigned long lastMicros=0;

// ---------- Struct for IMU readings ----------
struct Imu {
  float ax, ay, az; // g
  float gx, gy, gz; // dps
};

// ---------- Helpers ----------
void i2cWrite(uint8_t addr, uint8_t reg, uint8_t val){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void i2cRead(uint8_t addr, uint8_t reg, uint8_t n, uint8_t* dst){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)addr, (int)n, (int)true);
  for(int i=0;i<n;i++) dst[i]=Wire.read();
}

int16_t mk16(uint8_t hi, uint8_t lo){
  return (int16_t)((hi<<8) | lo);
}

bool initMPU(uint8_t addr){
  // wake up, use X-gyro clock
  i2cWrite(addr, REG_PWR_MGMT_1, 0x01);
  delay(10);

  // Sample rate 1 kHz (with DLPF enabled)
  i2cWrite(addr, REG_SMPLRT_DIV, 0x00);
  i2cWrite(addr, REG_CONFIG, DLPF_CFG);
  i2cWrite(addr, REG_GYRO_CONFIG,  (GYRO_FS_SEL<<3));
  i2cWrite(addr, REG_ACCEL_CONFIG, (ACC_FS_SEL<<3));
  delay(10);

  // Basic sanity check
  uint8_t who=0; i2cRead(addr, REG_WHO_AM_I, 1, &who);
  return (who!=0x00 && who!=0xFF);
}

void computeScales(){
  // Gyro LSB/dps: 131, 65.5, 32.8, 16.4 for ±250/500/1000/2000
  // Acc  LSB/g  : 16384, 8192, 4096, 2048 for ±2/4/8/16
  switch (GYRO_FS_SEL) {
    case 0: gyroScale = 1.0f/131.0f; break;
    case 1: gyroScale = 1.0f/65.5f;  break;
    case 2: gyroScale = 1.0f/32.8f;  break;
    default: gyroScale = 1.0f/16.4f; break;
  }
  switch (ACC_FS_SEL) {
    case 0: accelScale = 1.0f/16384.0f; break;
    case 1: accelScale = 1.0f/8192.0f;  break;
    case 2: accelScale = 1.0f/4096.0f;  break;
    default: accelScale = 1.0f/2048.0f; break;
  }
}

Imu readIMU(uint8_t addr){
  uint8_t b[14];
  i2cRead(addr, REG_ACCEL_XOUT_H, 14, b);

  int16_t ax=mk16(b[0],b[1]);
  int16_t ay=mk16(b[2],b[3]);
  int16_t az=mk16(b[4],b[5]);
  int16_t gx=mk16(b[8],b[9]);
  int16_t gy=mk16(b[10],b[11]);
  int16_t gz=mk16(b[12],b[13]);

  Imu r;
  r.ax = ax * accelScale;
  r.ay = ay * accelScale;
  r.az = az * accelScale;
  r.gx = gx * gyroScale;
  r.gy = gy * gyroScale;
  r.gz = gz * gyroScale;
  return r;
}

void scanI2C(){
  Serial.println("I2C scan:");
  for(uint8_t a=1;a<127;a++){
    Wire.beginTransmission(a);
    if(Wire.endTransmission()==0){
      Serial.print("  found 0x"); Serial.println(a, HEX);
    }
  }
}

void calibrateGyros(){
  const int N=2000; // ~2s at ~1kHz reads
  long s1x=0,s1y=0,s1z=0, s2x=0,s2y=0,s2z=0;

  for(int i=0;i<N;i++){
    Imu m1=readIMU(MPU6050_ADDR);
    Imu m2=readIMU(MPU6500_ADDR);

    // Sum in "raw-equivalent" units (divide by scale) for stability
    s1x += (long)(m1.gx/gyroScale);
    s1y += (long)(m1.gy/gyroScale);
    s1z += (long)(m1.gz/gyroScale);
    s2x += (long)(m2.gx/gyroScale);
    s2y += (long)(m2.gy/gyroScale);
    s2z += (long)(m2.gz/gyroScale);
    delay(1);
  }

  // Convert averages back to dps
  gBias1x = (s1x/(float)N)*gyroScale;
  gBias1y = (s1y/(float)N)*gyroScale;
  gBias1z = (s1z/(float)N)*gyroScale;

  gBias2x = (s2x/(float)N)*gyroScale;
  gBias2y = (s2y/(float)N)*gyroScale;
  gBias2z = (s2z/(float)N)*gyroScale;
}

// ---------- Arduino ----------
void setup(){
  Serial.begin(115200);
  delay(100);

  // Explicitly bind Wire to I2C1 pins on Blackpill
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin();
  Wire.setClock(400000); // Fast-mode

  computeScales();

  scanI2C(); // Expect 0x68 and 0x69

  bool ok1 = initMPU(MPU6050_ADDR);
  bool ok2 = initMPU(MPU6500_ADDR);
  if(!ok1 || !ok2){
    Serial.println("IMU init failed. Check wiring & AD0 (6050->GND=0x68, 6500->3V3=0x69).");
    while(1) delay(1000);
  }

  Serial.println("Keep still: calibrating gyros (~2s)...");
  calibrateGyros();
  Serial.println("Calibration done.");

  // Initialize attitude from averaged accel
  Imu m1=readIMU(MPU6050_ADDR), m2=readIMU(MPU6500_ADDR);
  float ax=0.5f*(m1.ax+m2.ax), ay=0.5f*(m1.ay+m2.ay), az=0.5f*(m1.az+m2.az);
  roll  = atan2f(ay, az);
  pitch = atanf(-ax / sqrtf(ay*ay + az*az));
  yaw   = 0.0f;

  lastMicros = micros();
}

void loop(){
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  if (dt <= 0) dt = 1e-6f;
  lastMicros = now;

  Imu m1=readIMU(MPU6050_ADDR), m2=readIMU(MPU6500_ADDR);

  // Remove each IMU's gyro bias
  m1.gx -= gBias1x;  m1.gy -= gBias1y;  m1.gz -= gBias1z;
  m2.gx -= gBias2x;  m2.gy -= gBias2y;  m2.gz -= gBias2z;

  // Quick & dirty average (equal weights)
  float gx = 0.5f*(m1.gx + m2.gx); // dps
  float gy = 0.5f*(m1.gy + m2.gy);
  float gz = 0.5f*(m1.gz + m2.gz);
  float ax = 0.5f*(m1.ax + m2.ax); // g
  float ay = 0.5f*(m1.ay + m2.ay);
  float az = 0.5f*(m1.az + m2.az);

  // Integrate gyro (convert dps->rad/s)
  float wx = gx * (PI/180.0f);
  float wy = gy * (PI/180.0f);
  float wz = gz * (PI/180.0f);
  roll  += wx * dt;
  pitch += wy * dt;
  yaw   += wz * dt;

  // Accel-only roll/pitch (guard az)
  float azSafe = (fabsf(az) < 1e-3f) ? (az >= 0 ? 1e-3f : -1e-3f) : az;
  float rollAcc  = atan2f(ay, azSafe);
  float pitchAcc = atanf(-ax / sqrtf(ay*ay + az*az + 1e-6f));

  // Complementary fuse
  roll  = alpha * roll  + (1.0f - alpha) * rollAcc;
  pitch = alpha * pitch + (1.0f - alpha) * pitchAcc;
  // yaw left as gyro-only (no mag yet)

  // Print ~100 Hz
  static uint32_t lastPrint=0;
  if (millis() - lastPrint >= 10) {
    lastPrint = millis();
    Serial.print("RPY(deg): ");
    Serial.print(roll*180.0f/PI, 2);  Serial.print(", ");
    Serial.print(pitch*180.0f/PI, 2); Serial.print(", ");
    Serial.print(yaw*180.0f/PI, 2);
    Serial.print(" | GyroAvg(dps): ");
    Serial.print(gx,2); Serial.print(", ");
    Serial.print(gy,2); Serial.print(", ");
    Serial.print(gz,2);
    Serial.print(" | AccAvg(g): ");
    Serial.print(ax,3); Serial.print(", ");
    Serial.print(ay,3); Serial.print(", ");
    Serial.println(az,3);
  }
}
