import math
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose

class TargetSwitcher(Node):
    def __init__(self):
        super().__init__('target_switcher')
        
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Store poses for turtles
        self.turtle1_pose = None
        self.turtle3_pose = None
        
        # Parameters for carrot rotation
        self.declare_parameter('carrot1.radius', 2.0)
        self.declare_parameter('carrot1.direction', 1)
        self.declare_parameter('carrot2.radius', 2.0)
        self.declare_parameter('carrot2.direction', 1)
        
        # Timer for rotation
        self.start_time = self.get_clock().now()
        
        # Subscribe to turtle poses
        self.subscription1 = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.turtle1_pose_callback,
            10)
            
        self.subscription3 = self.create_subscription(
            Pose,
            '/turtle3/pose', 
            self.turtle3_pose_callback,
            10)
        
        # Timer to publish transforms
        self.timer = self.create_timer(0.1, self.broadcast_transforms)
        
    def turtle1_pose_callback(self, msg):
        self.turtle1_pose = msg
        
    def turtle3_pose_callback(self, msg):
        self.turtle3_pose = msg
        
    def broadcast_transforms(self):
        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds / 1e9
        
        # Broadcast carrot1 (rotating around turtle1)
        if self.turtle1_pose is not None:
            radius1 = self.get_parameter('carrot1.radius').get_parameter_value().double_value
            direction1 = self.get_parameter('carrot1.direction').get_parameter_value().integer_value
            
            # Validate direction parameter
            if direction1 not in [1, -1]:
                direction1 = 1
            
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = 'turtle1'
            t.child_frame_id = 'carrot1'
            
            # Calculate rotating position
            angle = direction1 * elapsed_time
            t.transform.translation.x = radius1 * math.cos(angle)
            t.transform.translation.y = radius1 * math.sin(angle)
            t.transform.translation.z = 0.0
            
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)
            
        # Broadcast carrot2 (rotating around turtle3)
        if self.turtle3_pose is not None:
            radius2 = self.get_parameter('carrot2.radius').get_parameter_value().double_value
            direction2 = self.get_parameter('carrot2.direction').get_parameter_value().integer_value
            
            # Validate direction parameter
            if direction2 not in [1, -1]:
                direction2 = 1
            
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = 'turtle3'
            t.child_frame_id = 'carrot2'
            
            # Calculate rotating position
            angle = direction2 * elapsed_time
            t.transform.translation.x = radius2 * math.cos(angle)
            t.transform.translation.y = radius2 * math.sin(angle)
            t.transform.translation.z = 0.0
            
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)
            
        # Broadcast static_target (fixed position in world)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'static_target'
        t.transform.translation.x = 8.0
        t.transform.translation.y = 2.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = TargetSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()