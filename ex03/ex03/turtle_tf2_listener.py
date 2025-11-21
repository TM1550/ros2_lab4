import math
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from turtlesim.srv import Spawn

class TurtleTf2Listener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_listener')
        
        self.declare_parameter('delay', 5.0)
        self.delay = self.get_parameter('delay').get_parameter_value().double_value
        
        self.target_frame = self.declare_parameter('target_frame', 'turtle1').get_parameter_value().string_value
        self.source_frame = self.declare_parameter('source_frame', 'turtle2').get_parameter_value().string_value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.publisher = self.create_publisher(Twist, f'/{self.source_frame}/cmd_vel', 1)
        self.timer = self.create_timer(1.0, self.on_timer)
        
        self.spawn_turtle2()

    def spawn_turtle2(self):
        self.client = self.create_client(Spawn, 'spawn')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        request = Spawn.Request()
        request.x = 4.0
        request.y = 2.0
        request.theta = 0.0
        request.name = self.source_frame
        
        future = self.client.call_async(request)
        future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Spawned turtle {response.name}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def on_timer(self):
        from_frame_rel = self.target_frame
        to_frame_rel = self.source_frame
        
        try:
            when = self.get_clock().now() - rclpy.time.Duration(seconds=self.delay)
            trans = self.tf_buffer.lookup_transform_full(
                target_frame=to_frame_rel,
                target_time=rclpy.time.Time(),
                source_frame=from_frame_rel,
                source_time=when,
                fixed_frame='world',
                timeout=rclpy.time.Duration(seconds=0.05))
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {from_frame_rel} to {to_frame_rel}: {ex}')
            return

        msg = Twist()
        scale_rotation_rate = 1.0
        scale_forward_speed = 0.5
        
        msg.angular.z = scale_rotation_rate * math.atan2(
            trans.transform.translation.y,
            trans.transform.translation.x)
        
        msg.linear.x = scale_forward_speed * math.sqrt(
            trans.transform.translation.x ** 2 +
            trans.transform.translation.y ** 2)

        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = TurtleTf2Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()