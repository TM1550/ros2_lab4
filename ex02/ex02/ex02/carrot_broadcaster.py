import math
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

class CarrotBroadcaster(Node):
    def __init__(self):
        super().__init__('carrot_broadcaster')
        
        # Declare parameters
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('direction_of_rotation', 1)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)
        
        self.start_time = self.get_clock().now()
        
    def broadcast_timer_callback(self):
        # Get parameters
        radius = self.get_parameter('radius').get_parameter_value().double_value
        direction = self.get_parameter('direction_of_rotation').get_parameter_value().integer_value
        
        # Validate direction parameter
        if direction not in [1, -1]:
            self.get_logger().warn('Direction must be 1 or -1. Using default 1.')
            direction = 1
        
        # Calculate angle based on time for rotation
        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds / 1e9  # Convert to seconds
        angle = direction * elapsed_time  # Rotate at 1 radian per second
        
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'carrot1'
        
        # Calculate carrot position relative to turtle1
        t.transform.translation.x = radius * math.cos(angle)
        t.transform.translation.y = radius * math.sin(angle)
        t.transform.translation.z = 0.0
        
        # Carrot orientation (facing forward)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = CarrotBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()