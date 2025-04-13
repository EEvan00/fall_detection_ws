#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import json
from std_msgs.msg import String
from ultralytics import YOLO

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Declare parameters
        self.declare_parameter('model_path', 'yolo11n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('display_output', True)
        self.declare_parameter('furniture_classes', ['bed', 'couch', 'bench']) # From COCO dataset
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.display_output = self.get_parameter('display_output').value
        self.furniture_classes = self.get_parameter('furniture_classes').value
        
        # Initialize YOLO model
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f'Loaded YOLO11n object detection model: {self.model_path}')
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
        
        # Publisher for furniture detection data
        self.furniture_publisher = self.create_publisher(
            String,
            'furniture/detected',
            10
        )

        # CV bridge for converting between OpenCV and ROS image formats
        self.bridge = CvBridge()
        
        self.get_logger().info('Object detection node initialized')
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Process the image with YOLO11n object detection
            results = self.model(cv_image, conf=self.confidence_threshold)
            
            # Process and publish results
            self.process_and_publish_results(results)
            
            # Display the output if configured
            if self.display_output:
                # Visualize the results on the image
                annotated_image = results[0].plot()
                cv2.imshow('YOLO11n Furniture Detection', annotated_image)
                cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def process_and_publish_results(self, results):
        # Extract furniture objects from results
        furniture_data = []
        
        if results and len(results) > 0:
            # Get the first result
            result = results[0]
            
            # YOLO11n detection model returns boxes, class IDs, and confidences
            if hasattr(result, 'boxes') and result.boxes is not None:
                boxes = result.boxes
                
                # Iterate through detections
                for i, box in enumerate(boxes):
                    # Get class ID and name
                    cls_id = int(box.cls.item())
                    cls_name = result.names[cls_id]  # Get class name from the model's name mapping
                    
                    # Only include furniture classes we care about
                    if cls_name in self.furniture_classes:
                        confidence = float(box.conf.item())
                        bbox = box.xyxy[0].tolist()  # Get box coordinates [x1, y1, x2, y2]
                        
                        # Create data structure for this furniture object
                        object_data = {
                            'furniture_id': i,
                            'class_id': cls_id,
                            'class_name': cls_name,
                            'confidence': confidence,
                            'bbox': bbox  # [x1, y1, x2, y2]
                        }
                        
                        furniture_data.append(object_data)
        
        # Publish furniture data as JSON string
        if furniture_data:
            msg = String()
            msg.data = json.dumps(furniture_data)
            self.furniture_publisher.publish(msg)
            self.get_logger().debug(f'Published {len(furniture_data)} furniture objects')
    
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()