/**
 * ATLAS AGV - Motor Controller Config
 * 
 * Tüm kalibrasyon parametreleri ve pin tanımlamaları
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================
// PIN TANIMLARI
// ============================================
#define LEFT_PWM 10
#define LEFT_IN1 8
#define LEFT_IN2 9
#define RIGHT_PWM 11
#define RIGHT_IN1 12
#define RIGHT_IN2 13
#define LEFT_ENC_A A1
#define LEFT_ENC_B A0
#define RIGHT_ENC_A A2
#define RIGHT_ENC_B A3

// ============================================
// KALİBRE EDİLMİŞ PARAMETRELER
// ============================================
// Enkoder ve tekerlek
const float COUNTS_PER_WHEEL_REV = 435.0;  // Ölçülen gerçek değer
const float WHEEL_DIAMETER = 0.080;         // 80mm
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;
const float METERS_PER_COUNT = WHEEL_CIRCUMFERENCE / COUNTS_PER_WHEEL_REV;
const float WHEEL_BASE = 0.21;              // 210mm

// Motor kalibrasyon faktörleri (düz gidiş için)
const float LEFT_SPEED_FACTOR = 1.0;
const float RIGHT_SPEED_FACTOR = 1.07;      // Sağ motor %7 hızlandırıldı

// Hız limitleri
const int MAX_PWM = 100;                    // Güç kaynağı limiti
const int MIN_PWM = 70;                     // Minimum hareket eşiği
const float MAX_LINEAR_SPEED = 0.5;         // m/s
const float MAX_ANGULAR_SPEED = 2.0;        // rad/s

// Enkoder ve motor yön düzeltmeleri
const int LEFT_ENCODER_DIR = 1;
const int RIGHT_ENCODER_DIR = 1;
const int LEFT_MOTOR_DIR = -1;
const int RIGHT_MOTOR_DIR = -1;

// ============================================
// TIMING PARAMETERS
// ============================================
const unsigned long UPDATE_INTERVAL = 50;   // 20Hz odometry
const unsigned long CMD_TIMEOUT = 500;      // 500ms command timeout
const long SERIAL_BAUD = 115200;

#endif // CONFIG_H