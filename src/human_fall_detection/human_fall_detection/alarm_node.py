#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess
import threading
import time
import os

class AlarmNode(Node):
    def __init__(self):
        super().__init__('alarm_node')
        
        # Declare parameters
        self.declare_parameter('play_sound', True)
        self.declare_parameter('sound_file', '/usr/share/sounds/freedesktop/stereo/alarm-clock-elapsed.oga')
        self.declare_parameter('message', 'ALERT: Human fall detected!')
        self.declare_parameter('cooldown_time', 10.0)  # Time in seconds between consecutive alarms
        
        # Get parameters
        self.play_sound = self.get_parameter('play_sound').value
        self.sound_file = self.get_parameter('sound_file').value
        self.message = self.get_parameter('message').value
        self.cooldown_time = self.get_parameter('cooldown_time').value
        
        # Check if the sound file exists
        if self.play_sound and not os.path.exists(self.sound_file):
            self.get_logger().warning(f'Sound file not found: {self.sound_file}')
            self.get_logger().warning('Falling back to default system alert sound')
            self.sound_file = '/usr/share/sounds/freedesktop/stereo/bell.oga'
            
            # Check if the fallback exists
            if not os.path.exists(self.sound_file):
                self.get_logger().warning('Fallback sound file not found, disabling sound')
                self.play_sound = False
        
        # Subscriber for fall detection alerts
        self.fall_subscription = self.create_subscription(
            Bool,
            'human/fall_detected',
            self.fall_callback,
            10
        )
        
        # Alarm state
        self.alarm_active = False
        self.last_alarm_time = self.get_clock().now()
        
        self.get_logger().info('Alarm node initialized')
    
    def fall_callback(self, msg):
        # If fall detected (True) and not in cooldown period
        if msg.data and not self.alarm_active:
            current_time = self.get_clock().now()
            time_diff = (current_time - self.last_alarm_time).nanoseconds / 1e9
            
            if time_diff >= self.cooldown_time:
                self.trigger_alarm()
                self.last_alarm_time = current_time
    
    def trigger_alarm(self):
        self.alarm_active = True
        
        # Display message
        self.get_logger().error(self.message)
        
        # Play sound in a separate thread to avoid blocking
        if self.play_sound:
            sound_thread = threading.Thread(target=self.play_alarm_sound)
            sound_thread.daemon = True
            sound_thread.start()
        
        # Set a timer to reset the alarm state after cooldown
        timer = threading.Timer(self.cooldown_time, self.reset_alarm)
        timer.daemon = True
        timer.start()
    
    def play_alarm_sound(self):
        try:
            # Try to play sound using paplay, which is commonly available on Ubuntu
            subprocess.run(['paplay', self.sound_file], check=True)
        except (subprocess.SubprocessError, FileNotFoundError):
            try:
                # Fallback to aplay if paplay is not available
                subprocess.run(['aplay', self.sound_file], check=True)
            except (subprocess.SubprocessError, FileNotFoundError):
                self.get_logger().error('Failed to play alarm sound')
    
    def reset_alarm(self):
        self.alarm_active = False
        self.get_logger().info('Alarm reset, ready for new alerts')

def main(args=None):
    rclpy.init(args=args)
    node = AlarmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()