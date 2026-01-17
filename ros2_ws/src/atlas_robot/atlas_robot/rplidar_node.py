#!/usr/bin/env python3
"""
RPLidar A1 ROS 2 Node
Atlas Robot için entegre edilmiş
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rplidar import RPLidar, RPLidarException
import Jetson.GPIO as GPIO
import math
import time
import os

class RPLidarNode(Node):
    def __init__(self):
        super().__init__('rplidar_node')
        
        # Parametreler
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('motor_pin', 33)
        self.declare_parameter('motor_pwm', 95)
        self.declare_parameter('angle_compensate', True)
        self.declare_parameter('scan_frequency', 10.0)
        
        # Parametreleri al
        self.port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('serial_baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.motor_pin = self.get_parameter('motor_pin').value
        self.motor_pwm = self.get_parameter('motor_pwm').value
        self.angle_compensate = self.get_parameter('angle_compensate').value
        
        # Sudo kontrolü
        if os.geteuid() != 0:
            self.get_logger().error('Bu node sudo ile çalıştırılmalı!')
            raise RuntimeError('Sudo gerekli')
        
        # Publisher
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # Motor başlat
        self.get_logger().info('RPLidar motor başlatılıyor...')
        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.motor_pin, GPIO.OUT, initial=GPIO.LOW)
            self.pwm = GPIO.PWM(self.motor_pin, 50)
            self.pwm.start(self.motor_pwm)
            self.get_logger().info(f'Motor başlatıldı (Pin {self.motor_pin}, PWM {self.motor_pwm}%)')
            time.sleep(3)  # Motor ısınma süresi
        except Exception as e:
            self.get_logger().error(f'Motor başlatma hatası: {e}')
            raise
        
        # Lidar bağlan
        self.get_logger().info(f'RPLidar bağlanılıyor: {self.port}')
        try:
            self.lidar = RPLidar(self.port, baudrate=self.baudrate, timeout=3)
            
            # Buffer temizle
            if hasattr(self.lidar, '_serial'):
                self.lidar._serial.reset_input_buffer()
            
            # Cihaz bilgileri
            info = self.lidar.get_info()
            health = self.lidar.get_health()
            
            self.get_logger().info(f'Model: {info["model"]}')
            self.get_logger().info(f'Firmware: {info["firmware"][0]}.{info["firmware"][1]}')
            self.get_logger().info(f'Sağlık: {health[0]}')
            
            if health[0] != 'Good':
                self.get_logger().warn('Sağlık durumu iyi değil!')
        
        except Exception as e:
            self.get_logger().error(f'Lidar bağlantı hatası: {e}')
            self.cleanup_motor()
            raise
        
        # Scan generator
        self.scan_generator = self.lidar.iter_scans(max_buf_meas=500)
        
        # Timer ile sürekli tarama
        self.scan_timer = self.create_timer(0.05, self.scan_callback)  # 20Hz
        
        self.get_logger().info('✓ RPLidar node hazır!')
    
    def scan_callback(self):
        """Tarama verisi al ve yayınla"""
        try:
            # Bir tarama al
            scan = next(self.scan_generator)
            
            # LaserScan mesajı oluştur
            msg = LaserScan()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            
            msg.angle_min = 0.0
            msg.angle_max = 2.0 * math.pi
            msg.angle_increment = math.radians(1.0)  # 1 derece
            msg.time_increment = 0.0
            msg.scan_time = 0.1
            msg.range_min = 0.15  # 150mm
            msg.range_max = 12.0  # 12m
            
            # 360 nokta için dizi hazırla
            num_points = 360
            ranges = [float('inf')] * num_points
            intensities = [0.0] * num_points
            
            # Scan verisini işle
            for quality, angle, distance in scan:
                if quality < 10:  # Düşük kalite filtrele
                    continue
                
                # Açıyı normalize et (0-360)
                angle_deg = int(angle) % 360
                
                # Mesafeyi metre'ye çevir
                distance_m = distance / 1000.0
                
                # Geçerli aralıkta mı?
                if msg.range_min <= distance_m <= msg.range_max:
                    ranges[angle_deg] = distance_m
                    intensities[angle_deg] = float(quality)
            
            msg.ranges = ranges
            msg.intensities = intensities
            
            # Yayınla
            self.scan_pub.publish(msg)
        
        except StopIteration:
            # Generator bitti, yeniden başlat
            self.get_logger().warn('Scan generator sonu, yeniden başlatılıyor...')
            self.scan_generator = self.lidar.iter_scans(max_buf_meas=500)
        
        except RPLidarException as e:
            self.get_logger().error(f'RPLidar hatası: {e}')
        
        except Exception as e:
            self.get_logger().error(f'Scan callback hatası: {e}')
    
    def cleanup_motor(self):
        """Motor temizliği"""
        try:
            if hasattr(self, 'pwm'):
                self.pwm.stop()
            GPIO.cleanup(self.motor_pin)
        except:
            pass
    
    def destroy_node(self):
        """Node kapatılırken"""
        self.get_logger().info('RPLidar node kapatılıyor...')
        
        # Lidar kapat
        try:
            self.lidar.stop()
            self.lidar.disconnect()
            self.get_logger().info('✓ Lidar kapatıldı')
        except:
            pass
        
        # Motor kapat
        self.cleanup_motor()
        self.get_logger().info('✓ Motor kapatıldı')
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RPLidarNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()