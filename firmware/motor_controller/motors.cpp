/**
 * ATLAS AGV - Motors Implementation
 * 
 * Motor kontrolü implementasyonu
 */

#include "motors.h"

/**
 * Constructor - Başlangıç değerlerini ayarla
 */
Motors::Motors() {
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
}

/**
 * Motor pinlerini başlat
 * Tüm motor pinlerini OUTPUT olarak ayarla ve motorları durdur
 */
void Motors::begin() {
  // Sol motor pinleri
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  
  // Sağ motor pinleri
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  
  // Başlangıçta motorları durdur
  stop();
}

/**
 * Motor hızlarını ayarla
 * Kalibrasyon faktörlerini uygula, limit kontrolü yap
 * 
 * @param leftPWM Sol motor PWM (-255 ile +255)
 * @param rightPWM Sağ motor PWM (-255 ile +255)
 */
void Motors::setSpeed(int leftPWM, int rightPWM) {
  // Kalibrasyon faktörü uygula (Arduino'da kaldırıldı, ROS tarafında uygulanacak)
  // leftPWM = (int)(leftPWM * LEFT_SPEED_FACTOR);
  // rightPWM = (int)(rightPWM * RIGHT_SPEED_FACTOR);
  
  // MAX_PWM ile limit uygula
  leftPWM = constrain(leftPWM, -MAX_PWM, MAX_PWM);
  rightPWM = constrain(rightPWM, -MAX_PWM, MAX_PWM);
  
  // Minimum eşik kontrolü
  // MIN_PWM'in altındaki değerler motor hareket ettiremez
  if (abs(leftPWM) > 0 && abs(leftPWM) < MIN_PWM) {
    leftPWM = (leftPWM > 0) ? MIN_PWM : -MIN_PWM;
  }
  if (abs(rightPWM) > 0 && abs(rightPWM) < MIN_PWM) {
    rightPWM = (rightPWM > 0) ? MIN_PWM : -MIN_PWM;
  }
  
  // Motorları çalıştır
  setLeftMotor(leftPWM);
  setRightMotor(rightPWM);
}

/**
 * Motorları durdur
 * Her iki motoru da PWM=0 yap
 */
void Motors::stop() {
  setLeftMotor(0);
  setRightMotor(0);
}

/**
 * Sol motor hızını al
 * @return Mevcut sol motor PWM değeri
 */
int Motors::getLeftSpeed() {
  return currentLeftSpeed;
}

/**
 * Sağ motor hızını al
 * @return Mevcut sağ motor PWM değeri
 */
int Motors::getRightSpeed() {
  return currentRightSpeed;
}

/**
 * Sol motoru kontrol et (Private)
 * L298N H-Bridge sürücü ile motor kontrolü
 * 
 * @param speed PWM değeri (-255 ile +255)
 */
void Motors::setLeftMotor(int speed) {
  currentLeftSpeed = speed;
  
  // Motor yön düzeltmesi uygula
  speed *= LEFT_MOTOR_DIR;
  
  if (speed > 0) {
    // İleri yön
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
    analogWrite(LEFT_PWM, abs(speed));
  } 
  else if (speed < 0) {
    // Geri yön
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
    analogWrite(LEFT_PWM, abs(speed));
  } 
  else {
    // Dur
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
    analogWrite(LEFT_PWM, 0);
  }
}

/**
 * Sağ motoru kontrol et (Private)
 * L298N H-Bridge sürücü ile motor kontrolü
 * 
 * @param speed PWM değeri (-255 ile +255)
 */
void Motors::setRightMotor(int speed) {
  currentRightSpeed = speed;
  
  // Motor yön düzeltmesi uygula
  speed *= RIGHT_MOTOR_DIR;
  
  if (speed > 0) {
    // İleri yön
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
    analogWrite(RIGHT_PWM, abs(speed));
  } 
  else if (speed < 0) {
    // Geri yön
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
    analogWrite(RIGHT_PWM, abs(speed));
  } 
  else {
    // Dur
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
    analogWrite(RIGHT_PWM, 0);
  }
}