import rclpy
from ultralytics import YOLO
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, RegionOfInterest, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster, TransformStamped
from geometry_msgs.msg import PoseStamped
# from robot_follower.led_control import LedControl
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from scipy.spatial.transform import Rotation as R
from enum import auto, Enum
from std_srvs.srv import SetBool

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

        camera_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
                )

        sensor_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
                )

        self.declare_parameter("model",
                               value="best.pt")
        self.model = YOLO(self.get_parameter("model").get_parameter_value().string_value)
        self.declare_parameter("hand_model", value="hand_pose.pt")
        hand_model_path = self.get_parameter("hand_model").get_parameter_value().string_value
        self.hand_model = YOLO(hand_model_path)

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
        # self.declare_parameter("depth_topic", value="/j100_0076/sensors/camera_0/depth/image")
        self.declare_parameter("depth_topic", value="/j100_0076/sensors/camera_0/aligned_depth_to_color/image")
        # self.declare_parameter("topic",
        #                        value="/j100_0076/sensors/camera_0/color/compressed")
        self.topic = self.get_parameter("topic").get_parameter_value().string_value
        self.depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value

        self.create_subscription(CompressedImage, self.topic, self.image_callback, camera_qos)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, sensor_qos)
        self.image_pub = self.create_publisher(CompressedImage, 'vision/image/compressed', 10)
        self.coord_pub = self.create_publisher(RegionOfInterest, 'vision/target_roi', 10)

        self.broadcaster = TransformBroadcaster(self)
        self.latest_depth_array = None
        # self.info_sub = self.create_subscription(
        #     CameraInfo, 
        #     '/j100_0076/sensors/camera_0/depth/camera_info',
        #     self.info_callback,
        #     qos_profile_sensor_data)
        self.info_sub = self.create_subscription(
                    CameraInfo, 
                    '/j100_0076/sensors/camera_0/aligned_depth_to_color/camera_info',
                    self.info_callback,
                    sensor_qos)
        self.info_pub = self.create_publisher(CameraInfo, 'vision/camera_info', 10)
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
        self.declare_parameter("required_streak", value=3)
        self.required_streak = self.get_parameter("required_streak").get_parameter_value().integer_value
        self.max_missed_frames = 3

        self.state_sub = self.create_subscription(String, 'control/state', self.state_callback, 10)
        self.control_state = "Waiting"
        
        # Set up service client to receive gesture commands from vision node
        self.gesture_client = self.create_client(SetBool, 'control/set_movement')

        # Ensure gesture happens for a few frames before changing the robot state
        self.gesture_history = []
        self.gesture_confirm_frames = 15
        self.last_sent_gesture = None
        self.gesture_control_active = True  # set False after go command received


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
                    valid_depths = []
                    
                    for result in results:
                        keypoints = result.keypoints.data[0].cpu().numpy()

                        # Get the crop box for the right arm 
                        box = self.get_hand_crop_box(keypoints, cv_image.shape, side="right", scale_factor=1.5)

                        if box and self.gesture_control_active:
                            x1, y1, x2, y2 = box
                            raw_crop = cv_image[y1:y2, x1:x2]
                            h_crop, w_crop = raw_crop.shape[:2]
                            max_side = max(h_crop, w_crop)
                            pad_top = (max_side - h_crop) // 2
                            pad_bottom = max_side - h_crop - pad_top
                            pad_left = (max_side - w_crop) // 2
                            pad_right = max_side - w_crop - pad_left
                            square_crop = cv2.copyMakeBorder(raw_crop, pad_top, pad_bottom, pad_left, pad_right, cv2.BORDER_CONSTANT, value=(114, 114, 114))
                            ready_crop = cv2.resize(square_crop, (256, 256))
                            
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                            hand_results = self.hand_model.predict(ready_crop, conf=0.15, imgsz=256, verbose=False)

                            if hand_results[0].keypoints is not None and len(hand_results[0].keypoints.data) > 0:
                                hand_kpts = hand_results[0].keypoints.data[0].cpu().numpy()
                                gesture_name = self.recognize_gesture(hand_kpts)
                                label = f"Gesture: {gesture_name}"
                                cv2.putText(frame, label, (x1, y1 - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                                if gesture_name in ("Stop", "Go"):
                                    self.gesture_history.append(gesture_name)
                                else:
                                    self.gesture_history = []

                                # Keep only the last N frames
                                self.gesture_history = self.gesture_history[-self.gesture_confirm_frames:]

                                # Check if the last N frames are all the same gesture
                                if len(self.gesture_history) == self.gesture_confirm_frames:
                                    confirmed = self.gesture_history[0]
                                    if all(g == confirmed for g in self.gesture_history):
                                        movement = (confirmed == "Go")
                                        if confirmed != self.last_sent_gesture:
                                            self.last_sent_gesture = confirmed
                                            self._send_gesture_command(movement)
                                            if movement:  # Go confirmed
                                                # self.gesture_control_active = False
                                                self.gesture_history = []
                                                self.last_sent_gesture = None
                                                self.get_logger().info("Go gesture confirmed. Hand model deactivated.")

                                # Show confirmation progress on frame
                                filled = len(self.gesture_history)
                                bar_color = (0, 255, 0) if filled == self.gesture_confirm_frames else (0, 165, 255)
                                cv2.rectangle(frame, (x1, y2 + 4), 
                                              (x1 + int((filled / self.gesture_confirm_frames) * (x2 - x1)), y2 + 12),
                                              bar_color, -1)

                        # Collect all valid keypoint depths instead of taking minimum
                        for kp in keypoints:
                            X = int(kp[0])
                            Y = int(kp[1])
                            if X == 0 and Y == 0:
                                continue
                            height, width = self.latest_depth_array.shape
                            X = max(0, min(X, width - 1))
                            Y = max(0, min(Y, height - 1))
                            pixel_depth = self.latest_depth_array[Y, X]
                            if pixel_depth > 0:
                                valid_depths.append((pixel_depth, X, Y))

                    # Take the median depth reading instead of the minimum
                    if len(valid_depths) >= 3:
                        valid_depths.sort(key=lambda d: d[0])
                        median_idx = len(valid_depths) // 2
                        depth_mm, center_x, center_y = valid_depths[median_idx]
                    elif len(valid_depths) > 0:
                        # Fewer than 3 keypoints visible — fall back to median of what we have
                        valid_depths.sort(key=lambda d: d[0])
                        median_idx = len(valid_depths) // 2
                        depth_mm, center_x, center_y = valid_depths[median_idx]
                    else:
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
                    # tf_cam_person.header.frame_id = "camera_0_depth_optical_frame"
                    tf_cam_person.header.frame_id = "camera_0_color_optical_frame"
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
                self.gesture_history = []
                self.last_sent_gesture = None

        text = self.control_state
        pos = (10, 30)
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 1
        thickness = 2

        # Black outline
        cv2.putText(frame, text, pos, font, scale, (0, 0, 0), thickness + 4)
        # Green text on top
        cv2.putText(frame, text, pos, font, scale, (0, 255, 0), thickness)

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

    def get_hand_crop_box(self, keypoints, img_shape, side="right", scale_factor=2.5):
        """
        Calculates a dynamic bounding box for a hand based on elbow and wrist keypoints.
        
        Args:
            keypoints: The keypoints array from YOLO results (shape: [17, 3] for x, y, conf).
            img_shape: Tuple of (height, width) of the original image.
            side: "right" or "left" arm.
            scale_factor: Multiplier for the forearm length to determine box size.
            
        Returns:
            Tuple of (x1, y1, x2, y2) for the bounding box, or None if keypoints are hidden.
        """
        # 1. Map COCO indices for the chosen arm
        if side == "right":
            elbow_idx, wrist_idx = 8, 10
        else:
            elbow_idx, wrist_idx = 7, 9

        # 2. Extract x, y, and confidence score
        elbow = keypoints[elbow_idx]
        wrist = keypoints[wrist_idx]

        # 3. Check if the model is confident it actually sees the arm
        if elbow[2] < 0.5 or wrist[2] < 0.5:
            return None  # Arm is likely occluded or out of frame

        # 4. Calculate the distance (forearm length)
        dx = wrist[0] - elbow[0]
        dy = wrist[1] - elbow[1]
        arm_length = np.sqrt(dx**2 + dy**2)

        # 5. Determine the box size
        box_size = arm_length * scale_factor
        half_size = box_size / 2

        # 6. Shift the center slightly forward from the wrist 
        # (The hand is usually an extension of the arm, not exactly ON the wrist)
        if arm_length > 0:
            center_x = wrist[0] + (dx / arm_length) * (arm_length * 0.2)
            center_y = wrist[1] + (dy / arm_length) * (arm_length * 0.2)
        else:
            center_x, center_y = wrist[0], wrist[1]

        # 7. Calculate the corners of the bounding box
        x1 = int(center_x - half_size)
        y1 = int(center_y - half_size)
        x2 = int(center_x + half_size)
        y2 = int(center_y + half_size)

        # Find the largest side to force a perfect square
        box_width = x2 - x1
        box_height = y2 - y1
        max_side = max(box_width, box_height)
        
        # Re-center the square perfectly over the wrist
        x1_sq = int(center_x - (max_side / 2))
        y1_sq = int(center_y - (max_side / 2))
        x2_sq = int(center_x + (max_side / 2))
        y2_sq = int(center_y + (max_side / 2))

        # 8. Clamp the coordinates to the image dimensions (prevents crashing if hand is at the edge)
        h, w = img_shape[:2]
        x1 = max(0, x1_sq)
        y1 = max(0, y1_sq)
        x2 = min(w, x2_sq)
        y2 = min(h, y2_sq)

        # 9. Verify the box is valid
        if x2 <= x1 or y2 <= y1:
            return None

        return (x1, y1, x2, y2)

    def recognize_gesture(self, keypoints):
        """
        Determines the gesture based on 21 hand keypoints.
        Assumes standard 21-point layout (0=wrist, 4=thumb tip, 8=index tip, 12=middle tip, 16=ring tip, 20=pinky tip).
        """
        # Make sure we actually have 21 points
        if len(keypoints) < 21:
            return "Unknown"

        # Extract X,Y coordinates (ignore confidence score for the math)
        wrist = keypoints[0][:2]
        
        # Indices for the base knuckle (MCP) and tip of each finger
        # (Index, Middle, Ring, Pinky)
        fingers = [(5, 8), (9, 12), (13, 16), (17, 20)]
        
        extended_fingers = 0
        
        for base_idx, tip_idx in fingers:
            base_kpt = keypoints[base_idx][:2]
            tip_kpt = keypoints[tip_idx][:2]
            
            # Calculate Euclidean distances from the wrist
            dist_to_base = np.linalg.norm(wrist - base_kpt)
            dist_to_tip = np.linalg.norm(wrist - tip_kpt)
            
            # If the tip is much further from the wrist than the base knuckle, it's extended
            if dist_to_tip > dist_to_base * 1.3: 
                extended_fingers += 1

        # Classify based on how many fingers are up
        if extended_fingers >= 1:
            return "Stop"
        elif extended_fingers == 0:
            return "Go"
        # elif extended_fingers == 1 or extended_fingers == 2:
        #     # You can get fancy here later (e.g., checking if specifically the index finger is up)
        #     return "Pointing"
        else:
            return "Unknown"

    def _send_gesture_command(self, movement_allowed: bool):
        if not self.gesture_client.service_is_ready():
            self.get_logger().warn("Control service not ready, skipping gesture command.")
            return
        request = SetBool.Request()
        request.data = movement_allowed
        future = self.gesture_client.call_async(request)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"Gesture command sent: {'GO' if movement_allowed else 'STOP'}"
                )
            )
    
    def state_callback(self, msg):
        self.control_state = msg.data

def main():
    rclpy.init()
    node = Vision()
    rclpy.spin(node)
    rclpy.shutdown()
