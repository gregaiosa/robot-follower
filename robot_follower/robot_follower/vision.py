import rclpy
from ultralytics import YOLO
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, RegionOfInterest, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster, TransformStamped
from geometry_msgs.msg import PoseStamped
# from robot_follower.led_control import LedControl
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from enum import auto, Enum

class State(Enum):
    """An enumeration that controls behavior based on yolo model being run."""

    POSE = auto(),
    OBJECT = auto(),

class Vision(Node):
    def __init__(self):
        super().__init__("vision")
        self.bridge = CvBridge()
        # self.declare_parameter("model",
        #                        value="yolo26n.pt")
        self.declare_parameter("model",
                               value="best.pt")
        self.model = YOLO(self.get_parameter("model").get_parameter_value().string_value)

        # 1. Save the string value to a variable
        current_model = self.get_parameter("model").get_parameter_value().string_value
        
        # 2. Log it with quotes to see exactly what the node is receiving
        self.get_logger().info(f"Node received model string: '{current_model}'")

        self.declare_parameter("topic",
                               value="/image_raw/compressed")
        # self.declare_parameter("depth_topic",
        #                        value="/camera/camera/aligned_depth_to_color/image_raw")
        # self.declare_parameter("depth_topic",
        #                        value="/j100_0076/sensors/camera_0/depth/image")
        self.declare_parameter("depth_topic", value="/j100_0076/sensors/camera_0/depth/image")
        # self.declare_parameter("topic",
        #                        value="/j100_0076/sensors/camera_0/color/compressed")
        self.topic = self.get_parameter("topic").get_parameter_value().string_value
        self.depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value

        self.create_subscription(CompressedImage, self.topic, self.image_callback, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.image_pub = self.create_publisher(CompressedImage, 'new_image/compressed', 10)
        self.coord_pub = self.create_publisher(RegionOfInterest, 'vision/target_roi', 10)

        self.broadcaster = TransformBroadcaster(self)
        self.latest_depth_array = None
        self.info_sub = self.create_subscription(
            CameraInfo, 
            '/j100_0076/sensors/camera_0/depth/camera_info',
            self.info_callback,
            qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, 'new_image/camera_info', 10)
        self.intrinsics = None
        # self.led_controller = LedControl()
        self.person_tf = self.create_publisher(PoseStamped, 'person_pose', 10)

        if current_model.endswith("yolo26n-pose.pt") or current_model.endswith("yolo26n-pose.onnx"):
            self.state = State.POSE
        else:
            self.state = State.OBJECT
        self.get_logger().info(f"State is set to {self.state}")

        # Implement Exponential Moving Average for location values
        self.declare_parameter("ema_alpha", value=1.0)
        self.alpha = self.get_parameter("ema_alpha").get_parameter_value().double_value

        # Store the current filtered pose [x, y, z]
        self.ema_pose = None

        # Ensure a detection is consistent for a few frames before trusting it (helps with YOLO jitter)
        self.detection_streak = 0
        self.missed_frames = 0
        self.declare_parameter("required_streak", value=4)
        self.required_streak = self.get_parameter("required_streak").get_parameter_value().integer_value
        self.max_missed_frames = 3

    def info_callback(self, msg):
        self.intrinsics = {
            'fx': msg.k[0],
            'fy': msg.k[4],
            'cx': msg.k[2],
            'cy': msg.k[5]
        }

    def image_callback(self, image):
        """Identify all the objects in the scene"""
        # Convert to OpenCV
        cv_image = self.bridge.compressed_imgmsg_to_cv2(image, desired_encoding='bgr8')
        # Run the model
        if (self.state == State.POSE):
            results = self.model.predict(cv_image, conf = 0.6, imgsz=320)
        else:
            results = self.model.predict(cv_image, classes=[0], verbose=False, conf = 0.5) 
            
         # Only detect people (class 0)

        # Get the result and draw it on an OpenCV image
        frame = results[0].plot()

        # Output the coordinates of the bounding boxes 
        # [center_x, center_y, width, height]
        if len(results[0].boxes) > 0:  # Check if there are any detections
            # coord_msg.= results[0][0].boxes.xyxy[0].tolist()
            if self.state == State.OBJECT:
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

            else:
                center_x = 0
                center_y = 0
            
            
            self.get_logger().debug(f"Latest Depth array: {self.latest_depth_array}; self.intrinsics: {self.intrinsics}")
            if self.latest_depth_array is not None and self.intrinsics is not None:
                if self.state == State.OBJECT:
                    depth_mm = self.latest_depth_array[center_y, center_x]
                elif self.state == State.POSE:
                    min_dist = 1e9
                    best_x, best_y = 0, 0
                    
                    for result in results:
                        # Grab the keypoints for this specific person
                        # xy[0] extracts the array of keypoints for the first detected person
                        keypoints = result.keypoints.xy[0].cpu().numpy() 
                        
                        # Now iterate through each individual keypoint
                        for kp in keypoints:
                            X = int(kp[0])
                            Y = int(kp[1])
                            
                            # Skip keypoints YOLO couldn't see (it sets them to 0,0)
                            if X == 0 and Y == 0:
                                continue
                                
                            # Constrain X and Y to the actual image dimensions to prevent indexing errors
                            height, width = self.latest_depth_array.shape
                            X = max(0, min(X, width - 1))
                            Y = max(0, min(Y, height - 1))
                            
                            pixel_depth = self.latest_depth_array[Y, X]
                            
                            # Only update if the depth is valid (> 0) AND closer than previous
                            if 0 < pixel_depth < min_dist:
                                min_dist = pixel_depth
                                best_x = X
                                best_y = Y
                                # self.get_logger().info(f"New closest point found at ({X}, {Y}) with depth: {min_dist}")

                    # After checking all keypoints, assign the winning coordinates
                    if min_dist != 1e9:
                        center_x = best_x
                        center_y = best_y
                        depth_mm = min_dist
                    else:
                        # Fallback if no valid depth was found on any keypoint
                        depth_mm = 0
                
                if depth_mm > 0:

                    

                    # 1. Calculate the RAW 3D coordinates for this specific frame
                    raw_z = depth_mm / 1000.0  # Convert to meters
                    raw_x = (center_x - self.intrinsics['cx']) * raw_z / self.intrinsics['fx']
                    raw_y = (center_y - self.intrinsics['cy']) * raw_z / self.intrinsics['fy']

                    # 2. Apply the Exponential Moving Average
                    # if self.ema_pose is None:
                    #     # First valid frame: initialize the EMA with the raw readings
                    self.ema_pose = [raw_x, raw_y, raw_z]
                    # else:
                    #     # Subsequent frames: calculate the new EMA
                    #     self.ema_pose[0] = (self.alpha * raw_x) + ((1.0 - self.alpha) * self.ema_pose[0])
                    #     self.ema_pose[1] = (self.alpha * raw_y) + ((1.0 - self.alpha) * self.ema_pose[1])
                    #     self.ema_pose[2] = (self.alpha * raw_z) + ((1.0 - self.alpha) * self.ema_pose[2])

                    x_camera, y_camera, z_camera = self.ema_pose

                    self.get_logger().debug(f"Depth at center: {z_camera:.2f} m")
                    tf_cam_person = TransformStamped()
                    # tf_cam_person.header.stamp = image.header.stamp 
                    tf_cam_person.header.stamp = self.get_clock().now().to_msg()
                    # tf_msg.header.frame_id = "camera_0_link"
                    tf_cam_person.header.frame_id = "camera_0_depth_optical_frame"
                    tf_cam_person.child_frame_id = "person"
                    tf_cam_person.transform.translation.x = x_camera
                    tf_cam_person.transform.translation.y = y_camera
                    tf_cam_person.transform.translation.z = z_camera
                    tf_cam_person.transform.rotation.x = 0.0
                    tf_cam_person.transform.rotation.y = -.707
                    tf_cam_person.transform.rotation.z = 0.0
                    tf_cam_person.transform.rotation.w = .707
                    
                    person_msg = PoseStamped()
                    person_msg.header.stamp = tf_cam_person.header.stamp
                    person_msg.header.frame_id = tf_cam_person.header.frame_id
                    person_msg.pose.position.x = x_camera
                    person_msg.pose.position.y = y_camera
                    person_msg.pose.position.z = z_camera
                    person_msg.pose.orientation.x = 0.0
                    person_msg.pose.orientation.y = 0.0
                    person_msg.pose.orientation.z = 0.0
                    person_msg.pose.orientation.w = 1.0

                    if depth_mm > 0:
                        self.missed_frames = 0
                        self.detection_streak += 1

                        if self.detection_streak >= self.required_streak:
                            self.person_tf.publish(person_msg)
                            self.broadcaster.sendTransform(tf_cam_person)

                    else:
                        self.get_logger().warn("Depth value is zero, cannot determine distance.")
    
            cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
        else:
            self.missed_frames += 1
            # Reset streak on any missed frame
            if self.missed_frames > self.max_missed_frames:
                self.detection_streak = 0
                self.ema_pose = None

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
