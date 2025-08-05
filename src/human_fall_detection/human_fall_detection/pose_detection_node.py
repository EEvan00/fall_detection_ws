#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import json
import time
from std_msgs.msg import String
from ultralytics import YOLO

class PoseDetectionNode(Node):
    def __init__(self):
        super().__init__('pose_detection_node')
        
        # Declare parameters
        self.declare_parameter('model_path', 'yolo11n-pose.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('display_output', True)
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.display_output = self.get_parameter('display_output').value
        
        # FPS calculation variables
        self.prev_time = 0
        self.fps = 0

        # Initialize YOLO model
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f'Loaded YOLO11n model: {self.model_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO11n model: {str(e)}')
            return
        
        # Subscriber for camera frames
        self.image_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for pose data
        self.pose_publisher = self.create_publisher(
            String,
            'pose/keypoints',
            10
        )
        
        # Publisher for annotated image
        self.annotated_image_publisher = self.create_publisher(
            Image,
            'pose/annotated_image',
            10
        )
        
        # CV bridge for converting between OpenCV and ROS image formats
        self.bridge = CvBridge()
        
        self.get_logger().info('Pose detection node initialized')
    
    def image_callback(self, msg):
        try:
            # Start time
            start_time = time.time()

            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Process the image with YOLO11n pose detection
            results = self.model(cv_image, conf=self.confidence_threshold)
            
            # Process and publish results
            self.process_and_publish_results(results)

            # Publish annotated image
            if results and len(results) > 0:
                annotated_image = results[0].plot()

                # Calculate FPS
                end_time = time.time()
                processing_time = end_time - start_time
                self.fps = 1 / processing_time if processing_time > 0 else 0

                # Draw FPS on the image
                fps_text = f"FPS: {self.fps:.2f}"
                org = (10, 30) # Top-left corner coordinates
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 0.7
                color = (255, 255, 255) # white color
                thickness = 2
                cv2.putText(annotated_image, fps_text, org, font, fontScale, color, thickness, cv2.LINE_AA)

                # Convert back to ROS Image
                ros_annotated_image = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
                self.annotated_image_publisher.publish(ros_annotated_image)
            
            # Display the output if configured (not in use)
            if self.display_output:
                # Visualize the results on the image
                annotated_image = results[0].plot()
                cv2.imshow('YOLO11n Pose Detection', annotated_image)
                cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def process_and_publish_results(self, results):
        # Extract keypoints from results
        keypoints_data = []
        
        if results and len(results) > 0 and hasattr(results[0], 'keypoints') and results[0].keypoints is not None:
            keypoints = results[0].keypoints.data
            boxes = results[0].boxes.data
            
            # Extract data for each person detected
            for i, (kpts, box) in enumerate(zip(keypoints, boxes)):
                if box is not None and kpts is not None:
                    # Get bounding box coordinates and confidence
                    bbox = box.tolist()[:4]  # x1, y1, x2, y2
                    confidence = float(box.tolist()[4]) if len(box) > 4 else 1.0
                    
                    # Process keypoints
                    kpts_array = kpts.tolist()
                    
                    # Create data structure for this person
                    person_data = {
                        'person_id': i,
                        'bbox': bbox,
                        'confidence': confidence,
                        'keypoints': kpts_array
                    }
                    
                    keypoints_data.append(person_data)
        
        # Publish pose data as JSON string
        if keypoints_data:
            msg = String()
            msg.data = json.dumps(keypoints_data)
            self.pose_publisher.publish(msg)
    
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PoseDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()