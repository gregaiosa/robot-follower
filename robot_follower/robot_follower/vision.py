import rclpy
from ultralytics import YOLO
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class Vision(Node):
    def __init__(self):
        super().__init__("vision")
        self.bridge = CvBridge()
        # self.declare_parameter("model",
        #                        value="yolo26n.pt")
        self.declare_parameter("model",
                               value="best.pt")
        self.model = YOLO(self.get_parameter("model").get_parameter_value().string_value)

        self.declare_parameter("topic",
                               value="/image_raw/compressed")
        # self.declare_parameter("topic",
        #                        value="/j100_0076/sensors/camera_0/color/compressed")
        self.topic = self.get_parameter("topic").get_parameter_value().string_value


        self.create_subscription(CompressedImage, self.topic, self.yolo_callback, 10)
        self.pub = self.create_publisher(CompressedImage, 'new_image/compressed', 10)

    def yolo_callback(self, image):
        """Identify all the objects in the scene"""
        # Convert to OpenCV
        cv_image = self.bridge.compressed_imgmsg_to_cv2(image, desired_encoding='bgr8')
        # Run the model
        results = self.model(cv_image)
        # Get the result and draw it on an OpenCV image
        frame = results[0].plot()
        new_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        # publish
        self.pub.publish(new_msg)

def main():
    rclpy.init()
    node = Vision()
    rclpy.spin(node)
    rclpy.shutdown()