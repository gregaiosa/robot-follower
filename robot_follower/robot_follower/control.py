import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, Spin
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
import math
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from visualization_msgs.msg import Marker
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from ament_index_python.packages import get_package_share_directory
import os
from robot_follower.led_control import LedControl
from action_msgs.msg import GoalStatus

class Control(Node):
    def __init__(self):
        super().__init__("control")
        
        # Action Clients for Driving and Spinning
        self.nav_client = ActionClient(self, NavigateToPose, '/j100_0076/navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, '/j100_0076/spin')
        
        # Publishers and Subscribers
        self.goal_update_pub = self.create_publisher(PoseStamped, '/j100_0076/goal_update', 1)
        self.person_sub = self.create_subscription(PoseStamped, '/person_pose', self.person_callback, 10)
        self.reset_service = self.create_service(Empty, 'reset', self.reset_callback)
        
        # State tracking variables
        self.goal_sent = False
        self.is_spinning = False
        self.spin_goal_handle = None
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.follow_distance = 1.0
        self.backup_distance = 0.5

        # Handle for watchdog timer
        self.goal_handle = None
        # Timer for watchdog event
        self.watchdog_timer = self.create_timer(0.2, self.watchdog_callback)

        # Variables for progress tracking
        self.last_known_person_odom_x = None
        self.last_known_person_odom_y = None
        self.progress_check_interval = 1.0  # check progress every 1 second
        self.min_progress_distance = 0.2    # must close 20cm per second to count as progress
        self.last_robot_x = None
        self.last_robot_y = None
        self.missing_timeout = 6.0          # raise the hard timeout as a fallback
        self.no_progress_timeout = 2.0      # spin sooner if not making progress
        self.lost_and_no_progress_time = None
        self.last_seen_time = None  # None means "never seen yet"

        # Remember the last know y value (sign) in robot's base_link frame
        self.last_known_y = 0.0

        # Deadband Tracking
        self.last_goal_x = None
        self.last_goal_y = None
        self.deadband_radius = 0.4  # Require 25cm of movement to update the goal

        self.last_goal_update_time = None
        self.goal_update_min_interval = 0.3  # max ~3Hz goal updates
        
        # EMA Tracking in Odom Frame ---
        self.alpha = 0.5  # Smoothing factor (0.0 to 1.0)
        self.ema_person_x = None
        self.ema_person_y = None

        # Create marker to show person position in the map
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(Marker, 'person_marker', markerQoS)

        # Controls the LED to indicate state (Green = Target Visible, Yellow = Spinning, Red = Lost)
        self.led_controller = LedControl()

        # Declare behavior tree parameter
        self.declare_parameter("behavior_tree", value="")
        self.bt_path = self.get_parameter("behavior_tree").get_parameter_value().string_value

        self.last_nav_goal_time = None
        self.nav_goal_min_interval = 2.0  # don't send new NavigateToPose faster than 2Hz

    def person_callback(self, msg):
        self.last_seen_time = self.get_clock().now()
        msg.header.stamp = self.get_clock().now().to_msg()
        marker_pose = None
        self.led_controller.set_color(LedControl.GREEN, blink_ms=0)


        try:
            # Transform the person's pose from the camera's optical frame to the 'odom' frame
            target_frame = 'odom'
            transform_to_odom = self.tf_buffer.lookup_transform(
                target_frame, 
                msg.header.frame_id, 
                rclpy.time.Time()
            )
            person_odom = tf2_geometry_msgs.do_transform_pose_stamped(msg, transform_to_odom)

            # Transform to base_link to determine Left vs. Right ---
            transform_to_base_link = self.tf_buffer.lookup_transform(
                'base_link', 
                msg.header.frame_id, 
                rclpy.time.Time()
            )
            person_base_link = tf2_geometry_msgs.do_transform_pose_stamped(msg, transform_to_base_link)
            
            # Save the Y coordinate. Positive = Left, Negative = Right.
            self.last_known_y = person_base_link.pose.position.y
            
            # Get the robot's current position in the 'odom' frame
            robot_transform = self.tf_buffer.lookup_transform(
                target_frame, 
                'base_link', 
                rclpy.time.Time()
            )
            robot_x = robot_transform.transform.translation.x
            robot_y = robot_transform.transform.translation.y
            
            # Apply EMA to the ODOM coordinates
            raw_person_x = person_odom.pose.position.x
            raw_person_y = person_odom.pose.position.y
            
        
            if self.ema_person_x is None:
                self.ema_person_x = raw_person_x
                self.ema_person_y = raw_person_y
            else:
                self.ema_person_x = (self.alpha * raw_person_x) + ((1.0 - self.alpha) * self.ema_person_x)
                self.ema_person_y = (self.alpha * raw_person_y) + ((1.0 - self.alpha) * self.ema_person_y)

            self.last_known_person_odom_x = self.ema_person_x
            self.last_known_person_odom_y = self.ema_person_y
            
            # Create a dedicated pose for the marker (person's actual location)
            marker_pose = PoseStamped()
            marker_pose.header = person_odom.header
            marker_pose.pose.position.x = self.ema_person_x
            marker_pose.pose.position.y = self.ema_person_y
            marker_pose.pose.position.z = person_odom.pose.position.z
            marker_pose.pose.orientation.x = 0.0
            marker_pose.pose.orientation.y = 0.0
            marker_pose.pose.orientation.z = 0.0
            marker_pose.pose.orientation.w = 1.0

            # Publish marker immediately so visualization is smooth regardless of nav logic
            self.marker_publisher(marker_pose)

            # Calculate distance using the SMOOTHED coordinates
            dx = self.ema_person_x - robot_x
            dy = self.ema_person_y - robot_y
            distance = math.hypot(dx, dy)

            self.get_logger().info(f"Current true distance is {distance:.2f}m.")
            if distance < self.follow_distance:
                if distance < self.backup_distance:
                    self.get_logger().warn(f"Too close ({distance:.2f}m)! Backing up...")
                else:
                    self.get_logger().info(f"Robot within {self.follow_distance}m, not moving.")
                    self.get_logger().info(f"Current true distance is {distance:.2f}m.")
                    
                    # Stop the robot if it's currently moving
                    if self.goal_handle and self.goal_sent:
                        self.get_logger().info("Target close. Cancelling navigation goal...")
                        self.goal_handle.cancel_goal_async()
                        self.goal_handle = None
                        self.goal_sent = False
                    return 
                
            # Calculate the new point exactly follow_distance in front of the person
            ratio = (distance - self.follow_distance) / distance
            
            new_goal_x = robot_x + (dx * ratio)
            new_goal_y = robot_y + (dy * ratio)

            goal_distance_from_robot = math.hypot(new_goal_x - robot_x, new_goal_y - robot_y)
            if goal_distance_from_robot < 0.3:
            # Goal is too close, just stop navigation and wait
                if self.goal_handle and self.goal_sent:
                    self.goal_handle.cancel_goal_async()
                    self.goal_handle = None
                    self.goal_sent = False
                return

            # Deadband Logic (Don't spam MPPI if the goal hasn't moved much) ---
            if self.goal_sent and self.last_goal_x is not None:
                goal_shift = math.hypot(new_goal_x - self.last_goal_x, new_goal_y - self.last_goal_y)
                if goal_shift < self.deadband_radius:
                    return  # Ignore small jitters, let MPPI keep its current optimized path
            
            # Save the new valid goal
            self.last_goal_x = new_goal_x
            self.last_goal_y = new_goal_y
            
            # Update the transformed message with the new calculated point
            person_odom.pose.position.x = new_goal_x
            person_odom.pose.position.y = new_goal_y
            
            # Force the robot to rotate to face the person
            # yaw = math.atan2(dy, dx)
            # person_odom.pose.orientation.z = math.sin(yaw / 2.0)
            # person_odom.pose.orientation.w = math.cos(yaw / 2.0)
            # Replace the yaw calculation with:
            person_odom.pose.orientation.x = 0.0
            person_odom.pose.orientation.y = 0.0
            person_odom.pose.orientation.z = 0.0
            person_odom.pose.orientation.w = 1.0

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # Cancel the spin if YOLO finds you again
        if self.is_spinning and self.spin_goal_handle:
            self.get_logger().info("Target re-acquired! Canceling spin search...")
            self.spin_goal_handle.cancel_goal_async()
            self.is_spinning = False
            self.spin_goal_handle = None

        # Use person_odom (the new target) to send to the action server!
        if not self.goal_sent:
            now = self.get_clock().now()
            if self.last_nav_goal_time is not None:
                dt = (now - self.last_nav_goal_time).nanoseconds / 1e9
                if dt < self.nav_goal_min_interval:
                    return
            self.get_logger().info("Person detected! Starting pursuit...")
            self.last_nav_goal_time = now
            self.send_nav_goal(person_odom)
            self.goal_sent = True
        else:
            now = self.get_clock().now()
            if self.last_goal_update_time is not None:
                dt = (now - self.last_goal_update_time).nanoseconds / 1e9
                if dt < self.goal_update_min_interval:
                    return
            self.last_goal_update_time = now
            person_odom.header.stamp = now.to_msg()
            self.goal_update_pub.publish(person_odom)

    def send_nav_goal(self, initial_pose):
        if not self.nav_client.server_is_ready():
            self.get_logger().warn("Nav server not ready, skipping goal.")
            self.goal_sent = False
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = initial_pose
        
        goal_msg.behavior_tree = self.bt_path
        
        self._send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.nav_response_callback)

    def nav_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.goal_sent = False
            return
            
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        result = future.result()
        self.goal_sent = False
        self.goal_handle = None

        # Only reset EMA on genuine completion/abort, not cancellation
        if result.status != GoalStatus.STATUS_CANCELED:
            self.ema_person_x = None
            self.ema_person_y = None
            self.last_goal_x = None
            self.last_goal_y = None

    def send_spin_goal(self):
        if not self.spin_client.server_is_ready():
            self.get_logger().warn("Spin server not ready, skipping spin.")
            return
            
        goal_msg = Spin.Goal()
        
        # If last_known_y is positive (left), multiply by 1.0 (Spin CCW)
        # If last_known_y is negative (right), multiply by -1.0 (Spin CW)
        spin_direction = 1.0 if self.last_known_y >= 0 else -1.0
        
        # Spin 6.28 radians (360 degrees) in the correct direction
        goal_msg.target_yaw = spin_direction * 6.28  
        
        self.led_controller.set_color(LedControl.YELLOW, blink_ms=0)
        self.is_spinning = True
        self._send_spin_future = self.spin_client.send_goal_async(goal_msg)
        self._send_spin_future.add_done_callback(self.spin_response_callback)

    def spin_response_callback(self, future):
        self.spin_goal_handle = future.result()
        if not self.spin_goal_handle.accepted:
            self.is_spinning = False
            return
            
        self.spin_result_future = self.spin_goal_handle.get_result_async()
        self.spin_result_future.add_done_callback(self.spin_result_callback)

    def spin_result_callback(self, future):
        # This only triggers if the spin finishes WITHOUT finding you
        self.is_spinning = False
        self.spin_goal_handle = None
        self.get_logger().info("Spin sweep complete. Waiting for target to reappear...")
        self.last_seen_time = self.get_clock().now()  # prevent immediate re-trigger
        self.led_controller.set_color(LedControl.RED, blink_ms=500)

    def reset_callback(self, request, response):
        self.goal_sent = False
        self.is_spinning = False
        self.ema_person_x = None
        self.ema_person_y = None
        self.last_goal_x = None
        self.last_goal_y = None
        return response

    def watchdog_callback(self):
        if self.is_spinning:
            return

        if self.last_seen_time is None:
            return

        time_since_seen = (self.get_clock().now() - self.last_seen_time).nanoseconds / 1e9

        if time_since_seen <= 0.5:
            # Recently saw person, reset progress tracking
            self.lost_and_no_progress_time = None
            return

        # Person is lost — check if the robot is making progress toward last known position
        if self.last_known_person_odom_x is None:
            # No known position to navigate toward, spin immediately
            if time_since_seen > self.missing_timeout:
                self._trigger_spin()
            return

        try:
            robot_transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time()
            )
            robot_x = robot_transform.transform.translation.x
            robot_y = robot_transform.transform.translation.y

            dist_to_last_known = math.hypot(
                self.last_known_person_odom_x - robot_x,
                self.last_known_person_odom_y - robot_y
            )

            # If robot is already close to last known position, spin — 
            # we've arrived but still don't see them
            if dist_to_last_known < 0.5:
                self.get_logger().info("Reached last known position, no target found. Spinning...")
                self._trigger_spin()
                return

            # Check if robot is closing the distance (making progress)
            making_progress = False
            if self.last_robot_x is not None:
                dist_moved_toward_target = math.hypot(
                    robot_x - self.last_robot_x,
                    robot_y - self.last_robot_y
                )
                if dist_moved_toward_target > self.min_progress_distance:
                    making_progress = True
                    self.lost_and_no_progress_time = None  # reset no-progress clock

            self.last_robot_x = robot_x
            self.last_robot_y = robot_y

            if not making_progress:
                if self.lost_and_no_progress_time is None:
                    self.lost_and_no_progress_time = self.get_clock().now()
                else:
                    no_progress_duration = (
                        self.get_clock().now() - self.lost_and_no_progress_time
                    ).nanoseconds / 1e9
                    if no_progress_duration > self.no_progress_timeout:
                        self.get_logger().warn(
                            f"Lost target and no progress for {no_progress_duration:.1f}s. Spinning..."
                        )
                        self._trigger_spin()
                        return

            # Hard fallback — spin regardless after long enough
            if time_since_seen > self.missing_timeout:
                self.get_logger().warn(f"Hard timeout reached ({time_since_seen:.1f}s). Spinning...")
                self._trigger_spin()

        except Exception as e:
            self.get_logger().warn(f"Watchdog TF lookup failed: {e}")
            if time_since_seen > self.missing_timeout:
                self._trigger_spin()


    def _trigger_spin(self):
        if self.goal_handle and self.goal_sent:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

        self.goal_sent = False
        self.ema_person_x = None
        self.ema_person_y = None
        self.last_goal_x = None
        self.last_goal_y = None
        self.lost_and_no_progress_time = None
        self.last_robot_x = None
        self.last_robot_y = None
        self.send_spin_goal()

    def marker_publisher(self, person_odom):
        marker = Marker()
        marker.header.frame_id = person_odom.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "person marker"
        marker.id = 0
        
        # Robustly check for mesh file, fallback to cylinder if missing
        try:
            share_dir = get_package_share_directory('robot_follower')
            mesh_path = os.path.join(share_dir, 'models', 'low_poly_person.stl')
            
            if os.path.exists(mesh_path):
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = "package://robot_follower/models/low_poly_person.stl"
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
            else:
                self.get_logger().warn(f"Mesh not found at {mesh_path}. Using Cylinder fallback.")
                marker.type = Marker.CYLINDER
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 1.8
        except Exception as e:
            self.get_logger().warn(f"Could not resolve mesh path: {e}")
            marker.type = Marker.CYLINDER
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 1.8

        marker.action = Marker.ADD
        marker.pose = person_odom.pose
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = Control()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()