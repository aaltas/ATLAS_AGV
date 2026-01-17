#!/usr/bin/env python3
import sys
import os

# Değişen klasör adı
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rplidar_driver import RPLidarNode  # Klasör adı değişti

def main():
    if os.geteuid() != 0:
        print("❌ sudo ile çalıştırın!")
        sys.exit(1)
    
    node = RPLidarNode()
    
    try:
        node.start()
        
        def print_point(q, a, d):
            print(f"[{q:3d}] - [{a:6.2f}°] - [{d:7.1f} mm]")
        
        node.read_scans(callback=print_point, max_scans=10)
        
    finally:
        node.stop()

if __name__ == '__main__':
    main()