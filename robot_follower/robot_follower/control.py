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

class Control(Node):
    def __init__(self):
        super().__init__("control")
        
        # Action Clients for Driving and Spinning
        self.nav_client = ActionClient(self, NavigateToPose, '/j100_0076/navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, '/j100_0076/spin')
        
        # Publishers and Subscribers
        self.goal_update_pub = self.create_publisher(PoseStamped, '/j100_0076/goal_update', 10)
        self.person_sub = self.create_subscription(PoseStamped, '/person_pose', self.person_callback, 10)
        self.reset_service = self.create_service(Empty, 'reset', self.reset_callback)
        
        # State tracking variables
        self.goal_sent = False
        self.is_spinning = False
        self.spin_goal_handle = None
        
        # Timestamp to track the exact moment YOLO last saw you
        self.last_seen_time = self.get_clock().now()
        self.missing_timeout = 2.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.follow_distance = 1.0

        # Handle for watchdog timer
        self.goal_handle = None
        # Timer for watchdog event
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_callback)

        # Remember the last know y value (sign) in robot's base_link frame
        self.last_known_y = 0.0

    def person_callback(self, msg):
        self.last_seen_time = self.get_clock().now()
        msg.header.stamp = self.get_clock().now().to_msg()

        try:
            # 1. Transform the person's pose from the camera's optical frame to the 'odom' frame
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
            
            # 2. Get the robot's current position in the 'odom' frame
            robot_transform = self.tf_buffer.lookup_transform(
                target_frame, 
                'base_link', 
                rclpy.time.Time()
            )
            robot_x = robot_transform.transform.translation.x
            robot_y = robot_transform.transform.translation.y
            
            # 3. Now we can safely use X and Y because they represent the flat floor!
            person_x = person_odom.pose.position.x
            person_y = person_odom.pose.position.y
            
            dx = person_x - robot_x
            dy = person_y - robot_y
            distance = math.hypot(dx, dy)
            
            if distance < self.follow_distance:
                self.get_logger().info(f"Robot within {self.follow_distance}m, not moving.")
                self.get_logger().info(f"Current true distance is {distance:.2f}m.")
                # self.goal_handle.cancel_goal_async()
                return 
                
            # Calculate the new point exactly follow_distance in front of the person
            ratio = (distance - self.follow_distance) / distance
            
            # Update the transformed message with the new calculated point
            person_odom.pose.position.x = robot_x + (dx * ratio)
            person_odom.pose.position.y = robot_y + (dy * ratio)
            
            # Force the robot to rotate to face the person
            yaw = math.atan2(dy, dx)
            person_odom.pose.orientation.z = math.sin(yaw / 2.0)
            person_odom.pose.orientation.w = math.cos(yaw / 2.0)
            
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # Cancel the spin if YOLO finds you again
        if self.is_spinning and self.spin_goal_handle:
            self.get_logger().info("Target re-acquired! Canceling spin search...")
            self.spin_goal_handle.cancel_goal_async()
            self.is_spinning = False
            self.spin_goal_handle = None

        # IMPORTANT: Use person_odom (the new target) to send to the action server!
        if not self.goal_sent:
            self.get_logger().info("Person detected! Starting pursuit...")
            self.send_nav_goal(person_odom)
            self.goal_sent = True
        else:
            self.goal_update_pub.publish(person_odom)
            self.get_logger().info("Path updated!")

    def send_nav_goal(self, initial_pose):
        self.nav_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = initial_pose
        
        goal_msg.behavior_tree = "/home/robot/nav2_ws/src/navigation2/nav2_bt_navigator/behavior_trees/follow_target.xml" 
        
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
        # The goal finished normally, was aborted, or was cancelled by our watchdog
        self.goal_sent = False
        self.goal_handle = None

    def send_spin_goal(self):
        self.spin_client.wait_for_server()
        goal_msg = Spin.Goal()
        
        # If last_known_y is positive (left), multiply by 1.0 (Spin CCW)
        # If last_known_y is negative (right), multiply by -1.0 (Spin CW)
        spin_direction = 1.0 if self.last_known_y >= 0 else -1.0
        
        # Spin 6.28 radians (360 degrees) in the correct direction
        goal_msg.target_yaw = spin_direction * 6.28  
        
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

    async def reset_callback(self, request, response):
        self.goal_sent = False
        self.is_spinning = False
        return response

    def watchdog_callback(self):
        # Only run the check if we are actively following and NOT already spinning
        if self.goal_sent and not self.is_spinning:
            time_since_seen = (self.get_clock().now() - self.last_seen_time).nanoseconds / 1e9
            
            if time_since_seen > self.missing_timeout:
                self.get_logger().warn(f"Target lost for {time_since_seen:.1f}s! Hijacking Nav2...")
                
                # 1. Kill the active navigation goal immediately
                if self.goal_handle:
                    self.goal_handle.cancel_goal_async()
                    self.goal_handle = None
                
                # 2. Reset our state so we don't keep firing the watchdog
                self.goal_sent = False 
                
                # 3. Start the spin search
                self.send_spin_goal()


def main():
    rclpy.init()
    node = Control()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()