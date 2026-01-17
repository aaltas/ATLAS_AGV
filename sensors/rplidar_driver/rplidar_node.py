"""
RPLidar A1 Node
Motor + Data Reading
"""

import time
from rplidar import RPLidar, RPLidarException  
from . import config
from .rplidar_control import RPLidarControl

class RPLidarNode:
    """RPLidar A1 tam kontrol"""
    
    def __init__(self):
        self.motor = RPLidarControl()
        self.lidar = None
        self.is_running = False
        
        # İstatistikler
        self.scan_count = 0
        self.point_count = 0
        self.valid_points = 0
        self.start_time = None
    
    def start(self):
        """Sistemi başlat"""
        print("=" * 60)
        print("RPLidar A1 Node Başlatılıyor")
        print("=" * 60)
        
        # Motor başlat
        print("\n[1/2] Motor başlatılıyor...")
        self.motor.start()
        
        # Lidar bağlan
        print("\n[2/2] RPLidar bağlantısı...")
        try:
            self.lidar = RPLidar(
                config.SERIAL_PORT,
                baudrate=config.SERIAL_BAUD,
                timeout=config.SERIAL_TIMEOUT
            )
            
            # Buffer temizle
            if hasattr(self.lidar, '_serial'):
                self.lidar._serial.reset_input_buffer()
            
            # Cihaz bilgileri
            info = self.lidar.get_info()
            health = self.lidar.get_health()
            
            print(f"  ✓ Bağlantı başarılı")
            print(f"  Model: {info['model']}")
            print(f"  Firmware: {info['firmware'][0]}.{info['firmware'][1]}")
            print(f"  Serial: {info['serialnumber']}")
            print(f"  Sağlık: {health[0]}")
            
            if health[0] != 'Good':
                print(f"  ⚠ Uyarı: Sağlık durumu iyi değil!")
            
            self.is_running = True
            self.start_time = time.time()
            
            print("\n" + "=" * 60)
            print("✓ RPLidar Node hazır!")
            print("=" * 60)
            
        except Exception as e:
            print(f"  ✗ Bağlantı hatası: {e}")
            self.motor.stop()
            self.motor.cleanup()
            raise
    
    def read_scans(self, callback=None, max_scans=None):
        """
        Tarama verilerini oku
        
        Args:
            callback: Her nokta için çağrılacak fonksiyon (quality, angle, distance)
            max_scans: Maksimum tur sayısı (None = sonsuz)
        """
        if not self.is_running:
            raise RuntimeError("Node başlatılmamış!")
        
        print("\nVeri okuma başlıyor...")
        print("  → Ctrl+C ile durdurun\n")
        
        try:
            for scan in self.lidar.iter_scans(max_buf_meas=config.MAX_BUFFER_MEAS):
                self.scan_count += 1
                
                for quality, angle, distance in scan:
                    self.point_count += 1
                    
                    # Filtreleme
                    if (distance >= config.MIN_DISTANCE and 
                        distance <= config.MAX_DISTANCE and
                        quality >= config.MIN_QUALITY):
                        
                        self.valid_points += 1
                        
                        # Callback çağır
                        if callback:
                            callback(quality, angle, distance)
                
                # Maksimum tur kontrolü
                if max_scans and self.scan_count >= max_scans:
                    break
        
        except KeyboardInterrupt:
            print("\n\n⚠ Kullanıcı tarafından durduruldu")
        
        except RPLidarException as e:
            print(f"\n✗ RPLidar hatası: {e}")
        
        except Exception as e:
            print(f"\n✗ Beklenmeyen hata: {e}")
    
    def get_stats(self):
        """İstatistikleri döndür"""
        elapsed = time.time() - self.start_time if self.start_time else 0
        
        return {
            'scan_count': self.scan_count,
            'point_count': self.point_count,
            'valid_points': self.valid_points,
            'elapsed_time': elapsed,
            'points_per_second': self.point_count / elapsed if elapsed > 0 else 0,
            'scans_per_second': self.scan_count / elapsed if elapsed > 0 else 0,
        }
    
    def print_stats(self):
        """İstatistikleri yazdır"""
        stats = self.get_stats()
        
        print("\n" + "=" * 60)
        print("İSTATİSTİKLER")
        print("=" * 60)
        print(f"Süre:              {stats['elapsed_time']:.1f} saniye")
        print(f"Tur sayısı:        {stats['scan_count']}")
        print(f"Toplam nokta:      {stats['point_count']}")
        print(f"Geçerli nokta:     {stats['valid_points']}")
        print(f"Veri hızı:         {stats['points_per_second']:.0f} nokta/sn")
        print(f"Tarama hızı:       {stats['scans_per_second']:.2f} Hz")
        print("=" * 60)
    
    def stop(self):
        """Sistemi kapat"""
        print("\n" + "=" * 60)
        print("RPLidar Node Kapatılıyor")
        print("=" * 60)
        
        # İstatistikleri göster
        if self.point_count > 0:
            self.print_stats()
        
        # Lidar kapat
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.disconnect()
                print("✓ RPLidar bağlantısı kapatıldı")
            except:
                pass
        
        # Motor kapat
        self.motor.stop()
        self.motor.cleanup()
        
        self.is_running = False
        print("✓ Node kapatıldı\n")
    
    def __enter__(self):
        """Context manager"""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager çıkış"""
        self.stop()