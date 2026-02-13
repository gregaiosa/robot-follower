import rclpy
from ultralytics import YOLO
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, RegionOfInterest, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster, TransformStamped
from robot_follower.led_control import LEDControl

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
        # self.declare_parameter("depth_topic",
        #                        value="/camera/camera/aligned_depth_to_color/image_raw")
        # self.declare_parameter("depth_topic",
        #                        value="/j100_0076/sensors/camera_0/depth/image")
        self.declare_parameter("depth_topic", value="/j100_0076/sensors/camera_0/depth/compressedDepth")
        # self.declare_parameter("topic",
        #                        value="/j100_0076/sensors/camera_0/color/compressed")
        self.topic = self.get_parameter("topic").get_parameter_value().string_value
        self.depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value

        self.create_subscription(CompressedImage, self.topic, self.yolo_callback, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.image_pub = self.create_publisher(CompressedImage, 'new_image/compressed', 10)
        self.coord_pub = self.create_publisher(RegionOfInterest, 'vision/target_roi', 10)

        self.broadcaster = TransformBroadcaster(self)
        self.latest_depth_array = None
        self.info_sub = self.create_subscription(
            CameraInfo, 
            '/j100_0076/sensors/camera_0/depth/camera_info',
            self.info_callback,
            10)
        self.info_pub = self.create_publisher(CameraInfo, 'new_image/camera_info', 10)
        self.intrinsics = None
        self.led_controller = LEDControl()

    def info_callback(self, msg):
        self.intrinsics = {
            'fx': msg.k[0],
            'fy': msg.k[4],
            'cx': msg.k[2],
            'cy': msg.k[5]
        }

    def yolo_callback(self, image):
        """Identify all the objects in the scene"""
        # Convert to OpenCV
        cv_image = self.bridge.compressed_imgmsg_to_cv2(image, desired_encoding='bgr8')
        # Run the model
        results = self.model.predict(cv_image, classes=[0], verbose=False)  # Only detect people (class 0)

        # Get the result and draw it on an OpenCV image
        frame = results[0].plot()

        # Output the coordinates of the bounding boxes 
        # [center_x, center_y, width, height]
        if len(results[0].boxes) > 0:  # Check if there are any detections
            # coord_msg.= results[0][0].boxes.xyxy[0].tolist()
            coord_msg = RegionOfInterest()
            bbox = results[0].boxes.xyxy[0].cpu().numpy()
            coord_msg.x_offset = int(bbox[0])
            coord_msg.y_offset = int(bbox[1])
            coord_msg.height = int(bbox[3] - bbox[1])
            coord_msg.width = int(bbox[2] - bbox[0])
            coord_msg.do_rectify = False
            self.coord_pub.publish(coord_msg)

            center_x = int(bbox[0] + (bbox[2] - bbox[0]) / 2)
            center_y = int(bbox[1] + (bbox[3] - bbox[1]) / 2)
            
            # self.get_logger().info(f"Latest Depth array: {self.latest_depth_array}; self.intrinsics: {self.intrinsics}")
            if self.latest_depth_array is not None and self.intrinsics is not None:
                depth_mm = self.latest_depth_array[center_y, center_x]
                
                if depth_mm > 0:
                    z_camera = depth_mm / 1000.0  # Convert to meters
                    x_camera = (center_x - self.intrinsics['cx']) * z_camera / self.intrinsics['fx']
                    y_camera = (center_y - self.intrinsics['cy']) * z_camera / self.intrinsics['fy']
                    self.get_logger().info(f"Depth at center: {z_camera:.2f} m")
                    tf_cam_person = TransformStamped()
                    tf_cam_person.header.stamp = self.get_clock().now().to_msg()
                    # tf_msg.header.frame_id = "camera_0_link"
                    tf_cam_person.header.frame_id = "camera_0_depth_optical_frame"
                    tf_cam_person.child_frame_id = "person"
                    tf_cam_person.transform.translation.x = x_camera
                    tf_cam_person.transform.translation.y = y_camera
                    tf_cam_person.transform.translation.z = z_camera
                    tf_cam_person.transform.rotation.x = 0.0
                    tf_cam_person.transform.rotation.y = 0.0
                    tf_cam_person.transform.rotation.z = 0.0
                    tf_cam_person.transform.rotation.w = 1.0
                    self.broadcaster.sendTransform(tf_cam_person)
                    self.led_controller.set_color(LEDControl.GREEN, blink_ms=500)
                else:
                    self.get_logger().warn("Depth value is zero, cannot determine distance.")
                    self.led_controller.set_color(LEDControl.RED, blink_ms=500)
            else:
                self.led_controller.set_color(LEDControl.RED, blink_ms=0)
            cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
        if self.intrinsics:
            info_msg = CameraInfo()
            info_msg.header = image.header
            self.info_pub.publish(info_msg)
        new_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        # publish
        self.image_pub.publish(new_msg)
        

    def depth_callback(self, depth_image):
        depth_image = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')
        self.latest_depth_array = np.array(depth_image, dtype=np.uint16)

def main():
    rclpy.init()
    node = Vision()
    rclpy.spin(node)
    rclpy.shutdown()
