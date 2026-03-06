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

        self.follow_distance = 0.8

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
            # self.get_logger().info("Path updated!")

    def send_nav_goal(self, initial_pose):
        self.nav_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = initial_pose
        
        goal_msg.behavior_tree = "/home/robot/nav2_ws/src/navigation2/nav2_bt_navigator/behavior_trees/follow_target.xml" 
        
        self._send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.nav_response_callback)

    def nav_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.goal_sent = False
            return
            
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        # The robot has reached the ghost coordinate
        self.goal_sent = False
        
        # Calculate how long it has been since YOLO last published a message
        time_since_seen = (self.get_clock().now() - self.last_seen_time).nanoseconds / 1e9
        
        # If it has been more than missing_timeout, you are truly missing. Start spinning!
        if time_since_seen > self.missing_timeout:
            self.get_logger().info(f"Target lost for {time_since_seen:.1f}s. Initiating radar sweep...")
            self.send_spin_goal()

    def send_spin_goal(self):
        self.spin_client.wait_for_server()
        goal_msg = Spin.Goal()
        # Spin 6.28 radians (a full 360-degree sweep)
        goal_msg.target_yaw = 6.28  
        
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

def main():
    rclpy.init()
    node = Control()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()