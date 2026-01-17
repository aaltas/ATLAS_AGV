/**
 * ATLAS AGV - Sensor Hub Config
 * 
 * IMU sensör parametreleri ve I2C adresleri
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================
// I2C ADDRESSES
// ============================================
const uint8_t MPU6050_ADDR = 0x68;      // MPU-6050 (Gyro + Accel)
const uint8_t HMC5883L_ADDR = 0x1E;     // HMC5883L (Magnetometer)
const uint8_t BMP180_ADDR = 0x77;       // BMP180 (Barometer)

// ============================================
// IMU CALIBRATION PARAMETERS
// ============================================
// MPU-6050 Gyroscope scale (±250°/s seçildi)
const float IMU_GYRO_SCALE = 131.0;     // LSB/(deg/s)

// MPU-6050 Accelerometer scale (±2g seçildi)
const float IMU_ACCEL_SCALE = 16384.0;  // LSB/g

// HMC5883L Magnetometer scale
const float IMU_MAG_SCALE = 0.92;       // mG/LSB

// ============================================
// TIMING PARAMETERS
// ============================================
const unsigned long SENSOR_UPDATE_INTERVAL = 50;  // 20Hz sensor update
const long SERIAL_BAUD = 115200;

// ============================================
// SENSOR FUSION PARAMETERS (opsiyonel)
// ============================================
const float GYRO_WEIGHT = 0.98;                   // Complementary filter
const float ACCEL_WEIGHT = 0.02;

#endif // CONFIG_H