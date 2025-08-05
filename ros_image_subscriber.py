from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RosImageSubscriber(Node):
    def __init__(self):
        super().__init__('ros_image_subscriber')
        self.bridge = CvBridge()
        self.callback = None

        self.subscription = self.create_subscription(
            Image,
            '/fall_detection/image',
            self.listener_callback,
            10
        )

    def set_callback(self, callback):
        self.callback = callback

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.callback:
            self.callback(frame)
