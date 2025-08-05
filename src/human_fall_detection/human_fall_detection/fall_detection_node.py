#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json
import numpy as np
import math
from enum import Enum

class PersonState(Enum):
    STANDING = 0
    SITTING = 1
    LYING = 2
    FALLEN = 3
    UNKNOWN = 4

class FallDetectionNode(Node):
    def __init__(self):
        super().__init__('fall_detection_node')
        
        # Declare parameters
        self.declare_parameter('fall_confirmation_frames', 5)    # Number of frames to confirm a fall
        self.declare_parameter('time_between_alerts', 10.0)      # Minimum time between alerts in seconds
        
        # Get parameters
        self.fall_confirmation_frames = self.get_parameter('fall_confirmation_frames').value
        self.time_between_alerts = self.get_parameter('time_between_alerts').value
        
        # Subscriber for pose data
        self.pose_subscription = self.create_subscription(
            String,
            'pose/keypoints',
            self.pose_callback,
            10
        )
        
        # Publisher for fall alerts
        self.fall_publisher = self.create_publisher(
            Bool,
            'human/fall_detected',
            10
        )

        # Subscribe to furniture detection
        self.furniture_subscription = self.create_subscription(
            String,
            'furniture/detected',
            self.furniture_callback,
            10
        )
        
        # Store detected furniture
        self.detected_furniture = []  # List of furniture bounding boxes
        
        # Fall detection state
        self.fall_frames_counter = {}  # Track consecutive fall frames for each person
        self.last_alert_time = self.get_clock().now()
        self.person_states = {}  # Keep track of each person's state
        
        self.get_logger().info('Fall detection node initialized')
    
    def pose_callback(self, msg):
        try:
            # Parse pose data from JSON
            pose_data = json.loads(msg.data)
            
            # Check if we have any pose data
            if not pose_data:
                return
            
            # Get current timestamp
            current_time = self.get_clock().now()
            
            # Process each person's pose
            for person in pose_data:
                person_id = person['person_id']
                keypoints = person['keypoints']
                person_bbox = person['bbox']
                
                # Analyze pose to detect falls
                person_state = self.analyze_pose(keypoints)
                
                # Update person state
                self.person_states[person_id] = person_state
                
                # Check if person is on furniture
                on_furniture = self.is_person_on_furniture(person_bbox)
                if on_furniture:
                    self.get_logger().debug(f'Person {person_id} is on furniture')
                    
                # Correct state if detected as FALLEN but is on furniture
                if person_state == PersonState.FALLEN and on_furniture:
                    person_state = PersonState.LYING
                    
                # Update person state in the dictionary
                self.person_states[person_id] = person_state

                # Check if person has fallen (static pose analysis) AND is NOT on furniture
                if person_state == PersonState.FALLEN and not on_furniture:
                    # Initialize counter if it doesn't exist
                    if person_id not in self.fall_frames_counter:
                        self.fall_frames_counter[person_id] = 0
                    
                    # Increment counter
                    self.fall_frames_counter[person_id] += 1
                    
                    # If counter exceeds threshold, publish fall alert
                    if self.fall_frames_counter[person_id] >= self.fall_confirmation_frames:
                        # Check if enough time has passed since last alert
                        time_diff = (current_time - self.last_alert_time).nanoseconds / 1e9
                        
                        if time_diff >= self.time_between_alerts:
                            self.publish_fall_alert()
                            self.last_alert_time = current_time
                            # Reset the counter after alert
                            self.fall_frames_counter[person_id] = 0
                            
                            # Log the fall detection with relevant details
                            self.get_logger().info(
                                f'Fall detected for person {person_id}: ' +
                                f'Pose={person_state.name}'
                            )
                else:
                    # Reset counter if person is not fallen or is on furniture
                    self.fall_frames_counter[person_id] = 0
                    
        except Exception as e:
            self.get_logger().error(f'Error processing pose data: {str(e)}')
    
    def analyze_pose(self, keypoints):
        # Keypoints structure in YOLOv8:
        # 0: nose, 1: left_eye, 2: right_eye, 3: left_ear, 4: right_ear, 
        # 5: left_shoulder, 6: right_shoulder, 7: left_elbow, 8: right_elbow,
        # 9: left_wrist, 10: right_wrist, 11: left_hip, 12: right_hip,
        # 13: left_knee, 14: right_knee, 15: left_ankle, 16: right_ankle
        
        # Check if we have the minimal required keypoints
        #if len(keypoints) < 17:
        #    return PersonState.UNKNOWN
        
        # Extract relevant keypoints
        # Each keypoint is [x, y, confidence]
        nose = keypoints[0]
        left_shoulder = keypoints[5]
        right_shoulder = keypoints[6]
        left_hip = keypoints[11]
        right_hip = keypoints[12]
        left_ankle = keypoints[15]
        right_ankle = keypoints[16]
        
        # Calculate the center points for more robust detection
        shoulders_center = [(left_shoulder[0] + right_shoulder[0]) / 2, 
                           (left_shoulder[1] + right_shoulder[1]) / 2]
        hips_center = [(left_hip[0] + right_hip[0]) / 2, 
                      (left_hip[1] + right_hip[1]) / 2]
        ankles_center = [(left_ankle[0] + right_ankle[0]) / 2, 
                        (left_ankle[1] + right_ankle[1]) / 2]
        
        # Confidence check for keypoints
        min_confidence = 0.3
        key_keypoints = [nose, left_shoulder, right_shoulder, left_hip, right_hip, left_ankle, right_ankle]
        if any(kp[2] < min_confidence for kp in key_keypoints):
            return PersonState.UNKNOWN
        
        # Calculate body orientation
        # Vertical axis (spine) orientation
        spine_vector = [hips_center[0] - shoulders_center[0], hips_center[1] - shoulders_center[1]]
        spine_length = math.sqrt(spine_vector[0]**2 + spine_vector[1]**2)
        
        # Normalize for stability
        if spine_length > 0:
            spine_vector = [spine_vector[0] / spine_length, spine_vector[1] / spine_length]
        
        # Vertical axis in image space [0, 1]
        vertical_axis = [0, 1]
        
        # Calculate the angle between spine and vertical axis
        dot_product = spine_vector[0] * vertical_axis[0] + spine_vector[1] * vertical_axis[1]
        angle = math.acos(max(-1, min(1, dot_product)))  # Clamp to avoid domain errors
        angle_degrees = math.degrees(angle)
        
        # Bounding box analysis
        # Calculate the height and width ratio of the person
        x_values = [kp[0] for kp in key_keypoints if kp[2] >= min_confidence]
        y_values = [kp[1] for kp in key_keypoints if kp[2] >= min_confidence]
        
        if not y_values or not x_values:
            return PersonState.UNKNOWN
        
        height = max(y_values) - min(y_values)
        width = max(x_values) - min(x_values)
        
        aspect_ratio = width / height if height > 0 else 0
        
        # Check vertical distance between body parts
        # In a fall, the head, hips, and feet are often at similar y-coordinates
        vert_dispersion = max(abs(nose[1] - hips_center[1]), abs(hips_center[1] - ankles_center[1]))
        normalized_dispersion = vert_dispersion / height if height > 0 else 0
        
        # Determine state based on all factors
        if angle_degrees < 27 and aspect_ratio < 0.8 and normalized_dispersion > 0.3:
            # Upright position
            return PersonState.STANDING
        elif angle_degrees < 35 and aspect_ratio < 1.0:
            # Sitting position (slightly angled)
            return PersonState.SITTING
        elif angle_degrees > 45 and aspect_ratio > 1.0:
            # Lying or fallen position
            return PersonState.FALLEN
        else:
            return PersonState.UNKNOWN
   
    def is_person_on_furniture(self, person_bbox):
        """Check if a person is on or near furniture"""
        if not self.detected_furniture:
            return False
        
        # Extract person bounding box
        p_x1, p_y1, p_x2, p_y2 = person_bbox
        
        # Check each furniture item
        for furniture in self.detected_furniture:
            f_x1, f_y1, f_x2, f_y2 = furniture['bbox']
            
            # Calculate overlap
            # Check horizontal overlap
            x_overlap = max(0, min(p_x2, f_x2) - max(p_x1, f_x1))
            # Check vertical overlap
            y_overlap = max(0, min(p_y2, f_y2) - max(p_y1, f_y1))
            
            # Calculate overlap area
            overlap_area = x_overlap * y_overlap
            person_area = (p_x2 - p_x1) * (p_y2 - p_y1)
            
            # If significant overlap (>88% of person) or person's bottom is on furniture
            if (overlap_area > 0.88 * person_area):
                return True
        
        return False
    def furniture_callback(self, msg):
            """Callback for furniture detection messages"""
            try:
                # Parse furniture data from JSON
                furniture_data = json.loads(msg.data)
                
                # Store the furniture data
                self.detected_furniture = furniture_data
                
                # Log detected furniture for debugging
                if furniture_data:
                    furniture_names = [item['class_name'] for item in furniture_data]
                    self.get_logger().debug(f'Detected furniture: {furniture_names}')
                    
            except Exception as e:
                self.get_logger().error(f'Error processing furniture data: {str(e)}')
      
    def publish_fall_alert(self):
        msg = Bool()
        msg.data = True
        self.fall_publisher.publish(msg)
        self.get_logger().warn('FALL DETECTED! Alert published.')

def main(args=None):
    rclpy.init(args=args)
    node = FallDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()