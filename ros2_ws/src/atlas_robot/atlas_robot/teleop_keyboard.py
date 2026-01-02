#!/usr/bin/env python3
"""
Teleop Keyboard Node - Timeout based movement
If no key pressed within timeout, robot stops
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select
import threading
import time

INSTRUCTIONS = """
---------------------------
Atlas Robot - Keyboard Control
---------------------------
Hold keys to move (stops when released):
   w
 a s d

w : Forward
s : Backward
a : Turn left
d : Turn right

q : Increase speed (+0.05)
e : Decrease speed (-0.05)
CTRL-C to quit
---------------------------
Movement stops automatically if no key pressed for 0.2s
"""

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.speed = 0.3
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.last_key_time = time.time()
        self.timeout = 0.2  # Stop if no key for 200ms
        
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Publisher thread
        self.running = True
        self.pub_thread = threading.Thread(target=self.publish_loop)
        self.pub_thread.daemon = True
        self.pub_thread.start()
        
        print(INSTRUCTIONS)
        print(f"Speed: {self.speed:.2f} m/s | Hold keys to move...")
    
    def publish_loop(self):
        """Publish velocity at 20Hz and check timeout"""
        while self.running and rclpy.ok():
            # Check timeout - stop if no key pressed recently
            if time.time() - self.last_key_time > self.timeout:
                self.linear_vel = 0.0
                self.angular_vel = 0.0
            
            twist = Twist()
            twist.linear.x = self.linear_vel
            twist.angular.z = self.angular_vel
            self.pub.publish(twist)
            
            time.sleep(0.05)  # 20Hz
    
    def get_key_nonblocking(self, timeout=0.01):
        """Non-blocking key read with timeout"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def print_status(self):
        status = ""
        if self.linear_vel > 0:
            status = "FORWARD"
        elif self.linear_vel < 0:
            status = "BACKWARD"
        elif self.angular_vel > 0:
            status = "LEFT"
        elif self.angular_vel < 0:
            status = "RIGHT"
        else:
            status = "STOPPED"
        
        print(f"\r[{status:8s}] Speed: {self.speed:.2f} | Lin: {self.linear_vel:.2f} | Ang: {self.angular_vel:.2f}    ", end='', flush=True)
    
    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key_nonblocking(timeout=0.05)
                
                if key:
                    self.last_key_time = time.time()  # Update last key time
                    
                    if key == 'w':
                        self.linear_vel = self.speed
                        self.angular_vel = 0.0
                        
                    elif key == 's':
                        self.linear_vel = -self.speed
                        self.angular_vel = 0.0
                        
                    elif key == 'a':
                        self.linear_vel = 0.0
                        self.angular_vel = self.speed * 2.0
                        
                    elif key == 'd':
                        self.linear_vel = 0.0
                        self.angular_vel = -self.speed * 2.0
                        
                    elif key == ' ' or key == 'x':
                        self.linear_vel = 0.0
                        self.angular_vel = 0.0
                        
                    elif key == 'q':
                        self.speed = min(0.5, self.speed + 0.05)
                        print(f"\nSpeed increased: {self.speed:.2f} m/s")
                        
                    elif key == 'e':
                        self.speed = max(0.05, self.speed - 0.05)
                        print(f"\nSpeed decreased: {self.speed:.2f} m/s")
                        
                    elif key == '\x03':  # Ctrl-C
                        break
                    
                    self.print_status()
                
        except Exception as e:
            print(f"\nError: {e}")
            
        finally:
            self.running = False
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            time.sleep(0.2)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("\n\nStopped.")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    
    try:
        node.run()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()