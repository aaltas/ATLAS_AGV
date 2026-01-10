/**
 * ATLAS AGV - Sensors Header
 * 
 * Tüm sensörlerin ortak interface'i
 * Gelecekte eklenecek sensörler için placeholder
 */

#ifndef SENSORS_H
#define SENSORS_H

#include "imu.h"

// ============================================
// SENSOR TYPES (Gelecek için)
// ============================================
// Bu dosya gelecekte şu sensörler eklendiğinde genişletilecek:
// - Lidar (RPLidar A1)
// - Ultrasonic sensörler
// - Kamera modülü
// - GPS modülü
// - Voltage/Current sensörler

// ============================================
// SENSOR MANAGER CLASS (Gelecek için)
// ============================================
/*
class SensorManager {
public:
  SensorManager();
  
  void begin();
  void update();
  
  // Sensor access
  IMU* getIMU();
  // Lidar* getLidar();
  // Camera* getCamera();
  
private:
  IMU imu;
  // Lidar lidar;
  // Camera camera;
};
*/

#endif // SENSORS_H