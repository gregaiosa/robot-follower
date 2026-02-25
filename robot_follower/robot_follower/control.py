import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowObject
from geometry_msgs.msg import PoseStamped

class Control(Node):
    def __init__(self):
        super().__init__("control")
        
        # Create an Action Client to talk to the Nav2 following server
        self.action_client = ActionClient(self, FollowObject, '/j100_0076/follow_object')
        
        # Listen to the vision node to know when a person is in frame
        self.person_sub = self.create_subscription(PoseStamped, 'person_pose', self.person_callback, 10)
        
        self.goal_sent = False

    def person_callback(self, msg):
        # If we see a person and haven't sent the follow goal yet, send it!
        if not self.goal_sent:
            self.get_logger().info("Person detected! Triggering Nav2 FollowObject Action...")
            self.send_goal('person')
            self.goal_sent = True

    def send_goal(self, target_frame):
        # Wait for the action server to be ready
        self.get_logger().info("Waiting for /j100_0076/follow_object action server...")
        self.action_client.wait_for_server()
        
        # Construct the goal message using the correct attribute
        goal_msg = FollowObject.Goal()
        goal_msg.tracked_frame = target_frame  # FIX: changed from target_id to tracked_frame
        
        # Send the goal asynchronously
        self.get_logger().info(f"Sending goal to follow TF frame: {target_frame}")
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Follow goal rejected by the server.')
            self.goal_sent = False  # allow it to try again
            return
            
        self.get_logger().info('Follow goal accepted! The robot should now be following you.')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        self.get_logger().info(f'Follow action ended with status code: {status}')

        self.get_logger().info('Ready to track a new person.')
        self.goal_sent = False

def main():
    rclpy.init()
    node = Control()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()