/**
 * ATLAS AGV - IMU Interface Header
 * 
 * 10-DOF IMU sensör interface'i
 * MPU-6050 (Gyro + Accel) + HMC5883L (Mag) + BMP180 (Baro)
 */

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// ============================================
// IMU DATA STRUCTURE
// ============================================
struct IMUData {
  // Gyroscope (deg/s)
  float gyroX;
  float gyroY;
  float gyroZ;
  
  // Accelerometer (g - gravity units)
  float accelX;
  float accelY;
  float accelZ;
  
  // Magnetometer (uT - microTesla)
  float magX;
  float magY;
  float magZ;
  
  // Temperature (°C)
  float temperature;
  
  // Calculated orientation (radians)
  float pitch;       // X-axis rotation
  float roll;        // Y-axis rotation
  float heading;     // Compass heading (yaw)
  
  // Integrated gyro angle (for robot theta)
  float gyroIntegralZ;
  
  // Status
  bool valid;
};

// ============================================
// IMU CLASS
// ============================================
class IMU {
public:
  /**
   * Constructor
   */
  IMU();
  
  /**
   * Initialize IMU sensors
   * @return true if successful
   */
  bool begin();
  
  /**
   * Calibrate IMU (gyro and accel offsets)
   * Robot must be stationary!
   */
  void calibrate();
  
  /**
   * Update sensor readings
   * @param dt Delta time in seconds
   */
  void update(float dt);
  
  /**
   * Reset integrated angles
   */
  void reset();
  
  /**
   * Get IMU data
   * @return IMUData structure with all sensor values
   */
  IMUData getData();
  
  /**
   * Read gyroscope
   */
  void readGyro();
  
  /**
   * Read accelerometer
   */
  void readAccel();
  
  /**
   * Read magnetometer
   */
  void readMag();
  
  /**
   * Read temperature
   */
  void readTemp();
  
  /**
   * Set calibration offsets manually
   */
  void setGyroOffsets(float x, float y, float z);
  void setAccelOffsets(float x, float y, float z);
  void setMagOffsets(float x, float y, float z);
  
private:
  IMUData data;
  
  // Raw sensor values
  int16_t rawGyroX, rawGyroY, rawGyroZ;
  int16_t rawAccelX, rawAccelY, rawAccelZ;
  int16_t rawMagX, rawMagY, rawMagZ;
  int16_t rawTemp;
  
  // Calibration offsets
  float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
  float accelOffsetX, accelOffsetY, accelOffsetZ;
  float magOffsetX, magOffsetY, magOffsetZ;
  
  // I2C communication helpers
  void writeByte(uint8_t address, uint8_t reg, uint8_t value);
  uint8_t readByte(uint8_t address, uint8_t reg);
  void readBytes(uint8_t address, uint8_t reg, uint8_t count, uint8_t* dest);
};

// ============================================
// GLOBAL IMU INSTANCE
// ============================================
extern IMU imu;

#endif // IMU_H