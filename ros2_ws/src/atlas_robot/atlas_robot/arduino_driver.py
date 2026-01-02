#!/usr/bin/env python3
"""
Arduino Driver Node
Handles serial communication with Arduino
Publishes odometry and accepts velocity commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
import serial
import math
import threading

class ArduinoDriver(Node):
    def __init__(self):
        super().__init__('arduino_driver')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base', 0.21)
        self.declare_parameter('max_speed', 0.5)  # m/s
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value
        
        # Serial connection
        try:
            self.serial = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f'Connected to Arduino on {self.serial_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            raise
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.status_pub = self.create_publisher(String, 'arduino_status', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Start serial read thread
        self.running = True
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        self.get_logger().info('Arduino Driver Node started')
    
    def cmd_vel_callback(self, msg):
        """Convert Twist to differential drive commands"""
        linear = msg.linear.x  # m/s
        angular = msg.angular.z  # rad/s
        
        # PURE ROTATION (Tank Turn)
        if abs(linear) < 0.01 and abs(angular) > 0.01:
            # Sadece dönüş - tank modu
            # Angular hızı direkt PWM'e çevir
            rotation_speed = angular * 100  # rad/s -> PWM faktörü
            
            left_pwm = -int(rotation_speed)  # Sol geri
            right_pwm = int(rotation_speed)  # Sağ ileri
            
            # Limit
            max_pwm = 180
            min_pwm = 100
            
            left_pwm = max(-max_pwm, min(max_pwm, left_pwm))
            right_pwm = max(-max_pwm, min(max_pwm, right_pwm))
            
            # Minimum eşik
            if abs(left_pwm) < min_pwm:
                left_pwm = -min_pwm if left_pwm < 0 else min_pwm
            if abs(right_pwm) < min_pwm:
                right_pwm = -min_pwm if right_pwm < 0 else min_pwm
        
        else:
            # Normal hareket veya kavisli hareket
            angular_gain = 2.0
            angular = angular * angular_gain
            
            # Differential drive kinematics
            v_left = linear - (angular * self.wheel_base / 2.0)
            v_right = linear + (angular * self.wheel_base / 2.0)
            
            # Convert m/s to PWM
            left_pwm = int((v_left / self.max_speed) * 255)
            right_pwm = int((v_right / self.max_speed) * 255)
            
            # Limit
            max_pwm = 180
            min_pwm = 90
            
            left_pwm = max(-max_pwm, min(max_pwm, left_pwm))
            right_pwm = max(-max_pwm, min(max_pwm, right_pwm))
            
            # Minimum eşik
            if abs(left_pwm) > 0 and abs(left_pwm) < min_pwm:
                left_pwm = min_pwm if left_pwm > 0 else -min_pwm
            if abs(right_pwm) > 0 and abs(right_pwm) < min_pwm:
                right_pwm = min_pwm if right_pwm > 0 else -min_pwm
        
        # Send command
        cmd = f'CMD,{left_pwm},{right_pwm}\n'
        try:
            self.serial.write(cmd.encode())
            # Debug
            self.get_logger().debug(f'CMD: L={left_pwm}, R={right_pwm}, lin={linear:.2f}, ang={angular:.2f}')
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')
    
    def read_serial(self):
        """Read serial data in separate thread"""
        while self.running and rclpy.ok():
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8').strip()
                    self.parse_message(line)
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
    
    def parse_message(self, line):
        """Parse incoming serial messages"""
        if line.startswith('ODO,'):
            self.parse_odometry(line)
        elif line.startswith('STATUS,'):
            self.parse_status(line)
        elif line == 'ACK':
            self.get_logger().debug('ACK received')
        elif line == 'READY':
            self.get_logger().info('Arduino ready')
    
    def parse_odometry(self, line):
        """Parse odometry message: ODO,x,y,theta,vl,vr,encL,encR"""
        try:
            parts = line.split(',')
            if len(parts) != 8:
                return
            
            x = float(parts[1])
            y = float(parts[2])
            theta = float(parts[3])
            vl = float(parts[4])
            vr = float(parts[5])
            
            # Create Odometry message
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            
            # Position
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0.0
            
            # Orientation (quaternion from yaw)
            quat = self.euler_to_quaternion(0, 0, theta)
            odom.pose.pose.orientation = quat
            
            # Velocity
            linear_vel = (vl + vr) / 2.0
            angular_vel = (vr - vl) / self.wheel_base
            
            odom.twist.twist.linear.x = linear_vel
            odom.twist.twist.angular.z = angular_vel
            
            # Publish
            self.odom_pub.publish(odom)
            
            # Broadcast TF
            self.broadcast_tf(x, y, theta)
            
        except Exception as e:
            self.get_logger().error(f'Odometry parse error: {e}')
    
    def parse_status(self, line):
        """Parse status message: STATUS,left,right"""
        msg = String()
        msg.data = line
        self.status_pub.publish(msg)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
             math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
             math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
             math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
             math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        quat = Quaternion()
        quat.x = qx
        quat.y = qy
        quat.z = qz
        quat.w = qw
        return quat
    
    def broadcast_tf(self, x, y, theta):
        """Broadcast odom -> base_link transform"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        
        t.transform.rotation = self.euler_to_quaternion(0, 0, theta)
        
        self.tf_broadcaster.sendTransform(t)
    
    def destroy_node(self):
        """Cleanup"""
        self.running = False
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.write(b'STOP\n')
            self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoDriver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()