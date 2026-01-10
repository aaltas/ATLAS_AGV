/**
 * ATLAS AGV - Motors Header
 * 
 * Motor kontrolü için sınıf tanımı
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "config.h"

/**
 * Motors sınıfı
 * 
 * 2 motorlu differential drive sistemi için
 * L298N motor sürücü ile çalışır
 */
class Motors {
public:
  /**
   * Constructor
   */
  Motors();
  
  /**
   * Motor pinlerini başlat
   */
  void begin();
  
  /**
   * Motor hızlarını ayarla
   * @param leftPWM Sol motor PWM (-255 ile +255)
   * @param rightPWM Sağ motor PWM (-255 ile +255)
   */
  void setSpeed(int leftPWM, int rightPWM);
  
  /**
   * Motorları durdur
   */
  void stop();
  
  /**
   * Sol motor hızını al
   * @return Mevcut sol motor PWM değeri
   */
  int getLeftSpeed();
  
  /**
   * Sağ motor hızını al
   * @return Mevcut sağ motor PWM değeri
   */
  int getRightSpeed();
  
private:
  int currentLeftSpeed;   // Mevcut sol motor hızı
  int currentRightSpeed;  // Mevcut sağ motor hızı
  
  /**
   * Sol motoru kontrol et
   * @param speed PWM değeri (yön düzeltmesi uygulanmış)
   */
  void setLeftMotor(int speed);
  
  /**
   * Sağ motoru kontrol et
   * @param speed PWM değeri (yön düzeltmesi uygulanmış)
   */
  void setRightMotor(int speed);
};

#endif // MOTORS_H