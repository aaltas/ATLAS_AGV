"""
RPLidar A1 Configuration
"""

# Hardware Pinleri
MOTOR_PIN = 33              # Orin Nano Physical Pin 33
MOTOR_PWM_FREQ = 50         # 50 Hz PWM
MOTOR_DUTY_CYCLE = 95       # %95 PWM

# Serial Port
SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 3

# Motor Timing
MOTOR_WARMUP_TIME = 3       # Saniye

# Scan Settings
MAX_BUFFER_MEAS = 500       # Maksimum buffer

# Data Filtering
MIN_DISTANCE = 150          # mm
MAX_DISTANCE = 12000        # mm
MIN_QUALITY = 10            # Minimum kalite eşiği

# Performance
SCAN_RATE_MIN = 5           # Hz (minimum)
SCAN_RATE_MAX = 10          # Hz (maksimum)
POINTS_PER_SCAN_MIN = 300   # Minimum nokta/tur
POINTS_PER_SCAN_MAX = 400   # Maksimum nokta/tur