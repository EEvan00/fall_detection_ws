#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import json
import cv2
from ultralytics.utils.plotting import Annotator

class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')
        
        # Declare parameters
        self.declare_parameter('window_width', 640)
        self.declare_parameter('window_height', 480)
        self.declare_parameter('display_output', True)
        
        # Get parameters
        self.window_width = self.get_parameter('window_width').value
        self.window_height = self.get_parameter('window_height').value
        self.display_output = self.get_parameter('display_output').value
        
        # Subscriber for pose annotated image
        self.pose_image_subscription = self.create_subscription(
            Image,
            'pose/annotated_image',
            self.pose_image_callback,
            10
        )

        # Subscribe to furniture detections
        self.furniture_subscription = self.create_subscription(
            String,
            'furniture/detected',
            self.furniture_callback,
            10
        )

        # Subscriber for fall detection
        self.fall_subscription = self.create_subscription(
            Bool,
            'human/fall_detected',
            self.fall_callback,
            10
        )
        
        # CV bridge for converting between OpenCV and ROS image formats
        self.bridge = CvBridge()
        
        # Store the latest data
        self.latest_pose_image = None
        self.latest_furniture = []
        self.last_furniture_update_time = None
        self.latest_camera_image = None
        self.fall_detected = False
        self.fall_alert_time = None
        
        # Initialize timer for visualization
        timer_period = 0.033  # Update at approximately 30 Hz
        self.timer = self.create_timer(timer_period, self.visualize)
        
        self.get_logger().info('Visualization node initialized')
    
    def pose_image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            self.latest_pose_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error processing pose image: {str(e)}')

    def furniture_callback(self, msg):
        """Process furniture detection data"""
        try:
            # Parse furniture data
            self.latest_furniture = json.loads(msg.data)
            self.last_furniture_update_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().error(f'Error processing furniture data: {str(e)}')

    def camera_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image (as backup)
            self.latest_camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {str(e)}')

    def fall_callback(self, msg):
        """Callback for fall detection alerts"""
        if msg.data:  # If fall is detected
            self.fall_detected = True
            self.fall_alert_time = self.get_clock().now()
            self.get_logger().info('Fall alert received, updating visualization')
    
    def visualize(self):
        """Display pose visualization and add fall alert if needed"""
        # Check if fall alert should still be active (display for 5 seconds)
        if self.fall_detected and self.fall_alert_time is not None:
            current_time = self.get_clock().now()
            time_diff = (current_time - self.fall_alert_time).nanoseconds / 1e9
            if time_diff > 5.0:  # Reset fall alert after 5 seconds
                self.fall_detected = False
        
        # Use pose detection image if available, otherwise use camera image
        if self.latest_pose_image is not None:
            display_image = self.latest_pose_image.copy()
        elif self.latest_camera_image is not None:
            display_image = self.latest_camera_image.copy()
        else:
            # No images available yet
            return
        
        # Draw furniture detections
        self.draw_furniture(display_image)

        # Add fall alert message to the visualization if fall detected
        if self.fall_detected:
            # Add a red background at the top of the image for the alert
            alert_height = 40
            h, w = display_image.shape[:2]
            overlay = display_image.copy()
            cv2.rectangle(overlay, (0, 0), (w, alert_height), (0, 0, 255), -1)
            alpha = 0.6  # Transparency factor
            cv2.addWeighted(overlay, alpha, display_image, 1 - alpha, 0, display_image)
            
            # Add alert message text
            alert_msg = "ALERT: FALL DETECTED!"
            cv2.putText(display_image, alert_msg, 
                       (int(w/2 - 160), alert_height-10), cv2.FONT_HERSHEY_SIMPLEX, 
                       1.0, (255, 255, 255), 2)
        
        if self.last_furniture_update_time is not None:
            time_since_update = (self.get_clock().now() - self.last_furniture_update_time).nanoseconds / 1e9
            if time_since_update > 0.5:  # Clear if older than 0.5 second
                self.latest_furniture = []

        # Display the visualization
        if self.display_output:
            cv2.imshow('Fall Detection Visualization', display_image)
            cv2.waitKey(1)

    def draw_furniture(self, image):
        """Draw furniture bounding boxes on the image using YOLO11's Annotator"""
        # Create annotator object
        annotator = Annotator(image, line_width=2)
        
        # Define green color for all furniture
        green_color = (0, 255, 0)
        
        for furniture in self.latest_furniture:
            # Get furniture details
            class_name = furniture['class_name']
            bbox = furniture['bbox']
            confidence = furniture['confidence']
            
            # Create label
            label = f"{class_name} {confidence:.2f}"
            
            # Use annotator to draw box and label with fixed green color
            annotator.box_label(bbox, label, color=green_color)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()