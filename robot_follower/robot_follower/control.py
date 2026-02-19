# This node needs to do a transform lookup from person frame to the map frame, 
# and then publish a goal pose message to something to follow the person. 
# It should also publish the person's position in the map frame as a PoseStamped message.

import rclpy
from rclpy.node import Node
import numpy as np
from tf2_ros import buffer, TransformBroadcaster, TransformStamped, TransformListener, TransformException
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
from nav_msgs.msg import Goals
import copy
from scipy.spatial.transform import Rotation as R
import numpy as np


class Control(Node):
    def __init__(self):
        super().__init__("control")
        self.tf_buffer = buffer.Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.person_sub = self.create_subscription(PoseStamped, 'person_pose', self.person_callback, 10)

    def person_callback(self, msg):
        try:
            # Look up the transform from the camera frame to the map frame
            now = msg.header.stamp
            if not self.tf_buffer.can_transform('map', msg.header.frame_id, now, timeout=rclpy.duration.Duration(seconds=1.0)):
                self.get_logger().warn(f"Cannot transform from {msg.header.frame_id} to map frame at time {now}")
                return
            transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, now)
            # Transform the person's position to the map frame
            person_position = self.tf_buffer.transform(msg, 'map')
            r_current = R.from_quat([transform.transform.rotation.x, 
                                            transform.transform.rotation.y, 
                                            transform.transform.rotation.z, 
                                            transform.transform.rotation.w])

            r_rot = R.from_euler('z', 90, degrees=True)
            r_goal = r_current * r_rot
            goal_quat = r_goal.as_quat()
            
            # Publish the person's position as a goal pose
            goal_pose = PoseStamped()
            goal_pose = copy.deepcopy(person_position)
            goal_pose.header.frame_id = 'map'
            
            # Set the orientation to face forward (might need to be adjusted)
            goal_pose.pose.orientation.x = goal_quat[0]
            goal_pose.pose.orientation.y = goal_quat[1]
            goal_pose.pose.orientation.z = goal_quat[2]
            goal_pose.pose.orientation.w = goal_quat[3]
            distance = np.sqrt(person_position.pose.position.x**2 + person_position.pose.position.y**2)
            if distance > 0.3: # Only publish a goal if the person is more than 0.3 meters away
                self.goal_pub.publish(goal_pose)
        except TransformException as e:
            self.get_logger().warn(f"Could not transform person position to map frame: {e}")

def main():
    rclpy.init()
    node = Control()
    rclpy.spin(node)
    rclpy.shutdown()         


