#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Declare parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        
        # Get parameters
        self.camera_id = self.get_parameter('camera_id').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        
        # Publisher for camera frames
        self.image_publisher = self.create_publisher(
            Image,
            'camera/image_raw',
            10
        )
        
        # OpenCV video capture
        self.cap = cv2.VideoCapture(self.camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        
        # Check if camera opened successfully
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')
            return
        
        # Timer for frame capture
        timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # CV bridge for converting between OpenCV and ROS image formats
        self.bridge = CvBridge()
        
        self.get_logger().info('Camera node initialized')
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV image to ROS message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            # Publish the image
            self.image_publisher.publish(ros_image)
        else:
            self.get_logger().warning('Failed to capture frame')
    
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt detected. Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()