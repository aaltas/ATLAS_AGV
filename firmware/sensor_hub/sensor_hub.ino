/**
 * ATLAS AGV - SENSOR HUB
 * 
 * 10-DOF IMU sensör yönetimi
 * MPU-6050 + HMC5883L + BMP180
 */

#include <Wire.h>
#include "config.h"
#include "imu.h"
#include "sensors.h"

// ============================================
// GLOBAL VARIABLES
// ============================================
unsigned long prevTime = 0;
String inputString = "";
bool stringComplete = false;

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) delay(10);
  
  Serial.println(F("SENSOR_HUB: Initializing..."));
  
  // IMU başlat
  if (imu.begin()) {
    delay(1000);
    
    // Otomatik kalibrasyon
    Serial.println(F("SENSOR_HUB: Starting calibration..."));
    //imu.calibrate();
    
    Serial.println(F("SENSOR_HUB:READY"));
  } else {
    Serial.println(F("SENSOR_HUB:ERROR"));
    Serial.println(F("Check I2C connections!"));
  }
  
  inputString.reserve(32);
  prevTime = millis();
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  unsigned long currentTime = millis();
  
  // 20Hz IMU update
  if (currentTime - prevTime >= SENSOR_UPDATE_INTERVAL) {
    float deltaTime = (currentTime - prevTime) / 1000.0;
    
    // IMU verilerini güncelle
    imu.update(deltaTime);
    
    // Sensör verilerini yayınla
    publishSensorData();
    
    prevTime = currentTime;
  }
  
  // Serial komut işleme
  if (stringComplete) {
    parseCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
}

// ============================================
// SERIAL EVENT
// ============================================
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

// ============================================
// COMMAND PARSING
// ============================================
void parseCommand(String cmd) {
  cmd.trim();
  
  if (cmd == F("STATUS")) {
    // Detaylı IMU verilerini yazdır
    printIMUData();
  }
  else if (cmd == F("CALIBRATE")) {
    // Yeniden kalibrasyon
    imu.calibrate();
    Serial.println(F("ACK"));
  }
  else if (cmd == F("RESET")) {
    // Entegre açıları sıfırla
    imu.reset();
    Serial.println(F("ACK"));
  }
  else if (cmd == F("INFO")) {
    // Sensör bilgileri
    printSensorInfo();
  }
}

// ============================================
// PUBLISH SENSOR DATA
// ============================================
void publishSensorData() {
  // Format: IMU,gx,gy,gz,ax,ay,az,mx,my,mz,temp
  IMUData data = imu.getData();
  
  if (!data.valid) {
    return;
  }
  
  Serial.print(F("IMU,"));
  Serial.print(data.gyroX, 2);
  Serial.print(F(","));
  Serial.print(data.gyroY, 2);
  Serial.print(F(","));
  Serial.print(data.gyroZ, 2);
  Serial.print(F(","));
  Serial.print(data.accelX, 3);
  Serial.print(F(","));
  Serial.print(data.accelY, 3);
  Serial.print(F(","));
  Serial.print(data.accelZ, 3);
  Serial.print(F(","));
  Serial.print(data.magX, 2);
  Serial.print(F(","));
  Serial.print(data.magY, 2);
  Serial.print(F(","));
  Serial.print(data.magZ, 2);
  Serial.print(F(","));
  Serial.println(data.temperature, 1);
}

// ============================================
// PRINT DETAILED IMU DATA
// ============================================
void printIMUData() {
  IMUData data = imu.getData();
  
  Serial.println(F("\n=== IMU DATA ==="));
  
  // Gyroscope
  Serial.print(F("Gyro (deg/s): X="));
  Serial.print(data.gyroX, 2);
  Serial.print(F(" Y="));
  Serial.print(data.gyroY, 2);
  Serial.print(F(" Z="));
  Serial.println(data.gyroZ, 2);
  
  // Accelerometer
  Serial.print(F("Accel (g): X="));
  Serial.print(data.accelX, 3);
  Serial.print(F(" Y="));
  Serial.print(data.accelY, 3);
  Serial.print(F(" Z="));
  Serial.println(data.accelZ, 3);
  
  // Magnetometer
  Serial.print(F("Mag (uT): X="));
  Serial.print(data.magX, 2);
  Serial.print(F(" Y="));
  Serial.print(data.magY, 2);
  Serial.print(F(" Z="));
  Serial.println(data.magZ, 2);
  
  // Temperature
  Serial.print(F("Temp: "));
  Serial.print(data.temperature, 1);
  Serial.println(F(" C"));
  
  // Calculated orientation
  Serial.print(F("Heading: "));
  Serial.print(data.heading * 180 / PI, 1);
  Serial.println(F(" deg"));
  
  Serial.print(F("Pitch: "));
  Serial.print(data.pitch * 180 / PI, 1);
  Serial.println(F(" deg"));
  
  Serial.print(F("Roll: "));
  Serial.print(data.roll * 180 / PI, 1);
  Serial.println(F(" deg"));
  
  Serial.print(F("Gyro Integral Z: "));
  Serial.print(data.gyroIntegralZ * 180 / PI, 1);
  Serial.println(F(" deg"));
  
  Serial.println();
}

// ============================================
// PRINT SENSOR INFO
// ============================================
void printSensorInfo() {
  Serial.println(F("\n=== SENSOR INFO ==="));
  Serial.println(F("10-DOF IMU Sensor"));
  Serial.println(F("MPU-6050: Gyro + Accel"));
  Serial.println(F("HMC5883L: Magnetometer"));
  Serial.println(F("BMP180: Barometer"));
  Serial.println();
  Serial.print(F("I2C Addresses: MPU=0x"));
  Serial.print(MPU6050_ADDR, HEX);
  Serial.print(F(" HMC=0x"));
  Serial.print(HMC5883L_ADDR, HEX);
  Serial.print(F(" BMP=0x"));
  Serial.println(BMP180_ADDR, HEX);
  Serial.println();
  Serial.print(F("Update Rate: "));
  Serial.print(1000 / SENSOR_UPDATE_INTERVAL);
  Serial.println(F(" Hz"));
  Serial.println();
}