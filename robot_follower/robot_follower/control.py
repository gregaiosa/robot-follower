# This node needs to do a transform lookup from person frame to the map frame, 
# and then publish a goal pose message to something to follow the person. 
# It should also publish the person's position in the map frame as a PoseStamped message.

import rclpy
from rclpy.node import Node
import numpy as np
from tf2_ros import buffer, TransformBroadcaster, TransformStamped, TransformListener, TransformException
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Goals
import copy

class Control(Node):
    def __init__(self):
        super().__init__("control")
        self.tf_buffer = buffer.Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.person_sub = self.create_subscription(PoseStamped, 'person_position', self.person_callback, 10)

    def person_callback(self, msg):
        try:
            # Look up the transform from the camera frame to the map frame
            now = msg.header.stamp
            self.tf_buffer.can_transform('map', msg.header.frame_id, now, timeout=rclpy.duration.Duration(seconds=1.0))
            transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, now)
            # Transform the person's position to the map frame
            person_position = self.tf_buffer.transform(msg, 'map')
            # Publish the person's position as a goal pose
            goal_pose = PoseStamped()
            goal_pose = copy.deepcopy(person_position)
            goal_pose.header.frame_id = 'map'
            self.goal_pub.publish(goal_pose)
        except TransformException as e:
            self.get_logger().warn(f"Could not transform person position to map frame: {e}")
            


