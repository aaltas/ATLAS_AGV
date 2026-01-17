"""
RPLidar A1 Motor Control
Orin Nano Pin 33 PWM Control
"""

import Jetson.GPIO as GPIO
import time
from . import config

class RPLidarControl:
    """RPLidar A1 motor kontrolü"""
    
    def __init__(self, motor_pin=None, pwm_freq=None, duty_cycle=None):
        self.motor_pin = motor_pin or config.MOTOR_PIN
        self.pwm_freq = pwm_freq or config.MOTOR_PWM_FREQ
        self.duty_cycle = duty_cycle or config.MOTOR_DUTY_CYCLE
        
        self.pwm = None
        self.is_running = False
        
        # GPIO setup
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
    
    def start(self):
        """Motor başlat"""
        if self.is_running:
            print("Motor zaten çalışıyor")
            return
        
        try:
            GPIO.setup(self.motor_pin, GPIO.OUT, initial=GPIO.LOW)
            self.pwm = GPIO.PWM(self.motor_pin, self.pwm_freq)
            self.pwm.start(self.duty_cycle)
            
            self.is_running = True
            
            print(f"✓ RPLidar motor başlatıldı")
            print(f"  Pin: {self.motor_pin}")
            print(f"  PWM: {self.duty_cycle}% @ {self.pwm_freq}Hz")
            
            # Isınma süresi
            print(f"  Motor ısınıyor ({config.MOTOR_WARMUP_TIME}s)...")
            time.sleep(config.MOTOR_WARMUP_TIME)
            print("  ✓ Motor hazır")
            
        except Exception as e:
            print(f"✗ Motor başlatma hatası: {e}")
            self.cleanup()
            raise
    
    def stop(self):
        if not self.is_running:
            return
        
        try:
            if self.pwm:
                self.pwm.stop()
            
            # GPIO.output sadece pin hala OUTPUT modundaysa çalışır
            try:
                GPIO.output(self.motor_pin, GPIO.LOW)
            except:
                pass  # Zaten temizlenmiş, sorun değil
            
            self.is_running = False
            print("✓ Motor durduruldu")
        
        except Exception as e:
            print(f"⚠ Motor durdurma uyarısı: {e}")
    
    def set_speed(self, duty_cycle):
        """Motor hızını değiştir (0-100)"""
        if not 0 <= duty_cycle <= 100:
            raise ValueError("Duty cycle 0-100 arasında olmalı")
        
        self.duty_cycle = duty_cycle
        
        if self.is_running and self.pwm:
            self.pwm.ChangeDutyCycle(duty_cycle)
            print(f"✓ Motor hızı değiştirildi: %{duty_cycle}")
    
    def cleanup(self):
        """GPIO temizle"""
        try:
            if self.pwm:
                self.pwm.stop()
            GPIO.cleanup(self.motor_pin)
        except:
            pass
    
    def __enter__(self):
        """Context manager - with bloğu için"""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager - çıkış"""
        self.stop()
        self.cleanup()