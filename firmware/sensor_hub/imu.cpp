/**
 * ATLAS AGV - IMU Implementation
 * 
 * MPU-6050 + HMC5883L + BMP180 driver implementation
 */

#include "imu.h"

// ============================================
// MPU-6050 REGISTERS
// ============================================
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_TEMP_OUT_H   0x41
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_WHO_AM_I     0x75

// ============================================
// HMC5883L REGISTERS
// ============================================
#define HMC5883L_CONFIG_A    0x00
#define HMC5883L_CONFIG_B    0x01
#define HMC5883L_MODE        0x02
#define HMC5883L_DATA_X_MSB  0x03

// ============================================
// GLOBAL INSTANCE
// ============================================
IMU imu;

// ============================================
// CONSTRUCTOR
// ============================================
IMU::IMU() {
  gyroOffsetX = 0;
  gyroOffsetY = 0;
  gyroOffsetZ = 0;
  accelOffsetX = 0;
  accelOffsetY = 0;
  accelOffsetZ = 0;
  magOffsetX = 0;
  magOffsetY = 0;
  magOffsetZ = 0;
  
  data.valid = false;
  data.gyroIntegralZ = 0;
}

// ============================================
// INITIALIZATION
// ============================================
bool IMU::begin() {
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C hızı
  
  delay(100);
  
  // MPU-6050 bağlantı testi
  uint8_t whoami = readByte(MPU6050_ADDR, MPU6050_WHO_AM_I);
  if (whoami != 0x68) {
    Serial.println(F("IMU: MPU6050 not found!"));
    return false;
  }
  
  // MPU-6050'yi uyandır
  writeByte(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
  delay(100);
  
  // Gyroscope config: ±250°/s
  writeByte(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x00);
  
  // Accelerometer config: ±2g
  writeByte(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x00);
  
  // HMC5883L konfigürasyon
  writeByte(HMC5883L_ADDR, HMC5883L_CONFIG_A, 0x70);  // 8 sample avg, 15Hz
  writeByte(HMC5883L_ADDR, HMC5883L_CONFIG_B, 0x20);  // Gain = 1.3
  writeByte(HMC5883L_ADDR, HMC5883L_MODE, 0x00);      // Continuous mode
  
  delay(100);
  
  data.valid = true;
  Serial.println(F("IMU: Initialized successfully"));
  return true;
}

// ============================================
// CALIBRATION
// ============================================
void IMU::calibrate() {
  Serial.println(F("IMU: Calibrating... Keep robot still!"));
  
  const int samples = 100;
  long sumGX = 0, sumGY = 0, sumGZ = 0;
  long sumAX = 0, sumAY = 0, sumAZ = 0;
  
  for (int i = 0; i < samples; i++) {
    readGyro();
    readAccel();
    
    sumGX += rawGyroX;
    sumGY += rawGyroY;
    sumGZ += rawGyroZ;
    sumAX += rawAccelX;
    sumAY += rawAccelY;
    sumAZ += rawAccelZ;
    
    delay(10);
  }
  
  // Gyro offset hesapla
  gyroOffsetX = sumGX / samples / IMU_GYRO_SCALE;
  gyroOffsetY = sumGY / samples / IMU_GYRO_SCALE;
  gyroOffsetZ = sumGZ / samples / IMU_GYRO_SCALE;
  
  // Accel offset hesapla (Z ekseninde gravity var)
  accelOffsetX = sumAX / samples / IMU_ACCEL_SCALE;
  accelOffsetY = sumAY / samples / IMU_ACCEL_SCALE;
  accelOffsetZ = (sumAZ / samples / IMU_ACCEL_SCALE) - 1.0;
  
  Serial.print(F("IMU: Gyro offsets: "));
  Serial.print(gyroOffsetX, 2); Serial.print(F(", "));
  Serial.print(gyroOffsetY, 2); Serial.print(F(", "));
  Serial.println(gyroOffsetZ, 2);
  
  Serial.println(F("IMU: Calibration complete!"));
}

// ============================================
// UPDATE
// ============================================
void IMU::update(float dt) {
  if (!data.valid) return;
  
  // Tüm sensörleri oku
  readGyro();
  readAccel();
  readMag();
  readTemp();
  
  // Pitch ve Roll hesapla (accelerometer'dan)
  data.pitch = atan2(data.accelY, sqrt(data.accelX * data.accelX + data.accelZ * data.accelZ));
  data.roll = atan2(-data.accelX, data.accelZ);
  
  // Heading hesapla (magnetometer'dan - tilt compensated)
  float magXComp = data.magX * cos(data.pitch) + data.magZ * sin(data.pitch);
  float magYComp = data.magX * sin(data.roll) * sin(data.pitch) + 
                   data.magY * cos(data.roll) - 
                   data.magZ * sin(data.roll) * cos(data.pitch);
  data.heading = atan2(magYComp, magXComp);
  
  // Gyro Z eksenini entegre et (robot theta için)
  data.gyroIntegralZ += data.gyroZ * dt * PI / 180.0;  // deg/s'den rad'a
  
  // Normalize (-π ile +π arası)
  while (data.gyroIntegralZ > PI) data.gyroIntegralZ -= 2.0 * PI;
  while (data.gyroIntegralZ < -PI) data.gyroIntegralZ += 2.0 * PI;
}

// ============================================
// RESET
// ============================================
void IMU::reset() {
  data.gyroIntegralZ = 0;
}

// ============================================
// READ GYROSCOPE
// ============================================
void IMU::readGyro() {
  uint8_t buffer[6];
  readBytes(MPU6050_ADDR, MPU6050_GYRO_XOUT_H, 6, buffer);
  
  rawGyroX = (buffer[0] << 8) | buffer[1];
  rawGyroY = (buffer[2] << 8) | buffer[3];
  rawGyroZ = (buffer[4] << 8) | buffer[5];
  
  // Scale ve offset uygula
  data.gyroX = (rawGyroX / IMU_GYRO_SCALE) - gyroOffsetX;
  data.gyroY = (rawGyroY / IMU_GYRO_SCALE) - gyroOffsetY;
  data.gyroZ = (rawGyroZ / IMU_GYRO_SCALE) - gyroOffsetZ;
}

// ============================================
// READ ACCELEROMETER
// ============================================
void IMU::readAccel() {
  uint8_t buffer[6];
  readBytes(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 6, buffer);
  
  rawAccelX = (buffer[0] << 8) | buffer[1];
  rawAccelY = (buffer[2] << 8) | buffer[3];
  rawAccelZ = (buffer[4] << 8) | buffer[5];
  
  // Scale ve offset uygula
  data.accelX = (rawAccelX / IMU_ACCEL_SCALE) - accelOffsetX;
  data.accelY = (rawAccelY / IMU_ACCEL_SCALE) - accelOffsetY;
  data.accelZ = (rawAccelZ / IMU_ACCEL_SCALE) - accelOffsetZ;
}

// ============================================
// READ MAGNETOMETER
// ============================================
void IMU::readMag() {
  uint8_t buffer[6];
  readBytes(HMC5883L_ADDR, HMC5883L_DATA_X_MSB, 6, buffer);
  
  // HMC5883L data order: X, Z, Y (dikkat!)
  rawMagX = (buffer[0] << 8) | buffer[1];
  rawMagZ = (buffer[2] << 8) | buffer[3];
  rawMagY = (buffer[4] << 8) | buffer[5];
  
  // Scale ve offset uygula
  data.magX = (rawMagX * IMU_MAG_SCALE) - magOffsetX;
  data.magY = (rawMagY * IMU_MAG_SCALE) - magOffsetY;
  data.magZ = (rawMagZ * IMU_MAG_SCALE) - magOffsetZ;
}

// ============================================
// READ TEMPERATURE
// ============================================
void IMU::readTemp() {
  uint8_t buffer[2];
  readBytes(MPU6050_ADDR, MPU6050_TEMP_OUT_H, 2, buffer);
  
  rawTemp = (buffer[0] << 8) | buffer[1];
  data.temperature = (rawTemp / 340.0) + 36.53;
}

// ============================================
// DATA ACCESS
// ============================================
IMUData IMU::getData() {
  return data;
}

// ============================================
// CALIBRATION SETTERS
// ============================================
void IMU::setGyroOffsets(float x, float y, float z) {
  gyroOffsetX = x;
  gyroOffsetY = y;
  gyroOffsetZ = z;
}

void IMU::setAccelOffsets(float x, float y, float z) {
  accelOffsetX = x;
  accelOffsetY = y;
  accelOffsetZ = z;
}

void IMU::setMagOffsets(float x, float y, float z) {
  magOffsetX = x;
  magOffsetY = y;
  magOffsetZ = z;
}

// ============================================
// I2C HELPERS
// ============================================
void IMU::writeByte(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t IMU::readByte(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)1);
  return Wire.read();
}

void IMU::readBytes(uint8_t address, uint8_t reg, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(address, count);
  
  for (uint8_t i = 0; i < count; i++) {
    dest[i] = Wire.read();
  }
}