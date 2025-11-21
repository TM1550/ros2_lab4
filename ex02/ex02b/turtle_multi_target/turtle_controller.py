import math
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from turtlesim.srv import Spawn
from std_msgs.msg import String

# Временное решение - используем стандартные сообщения вместо пользовательских
from std_msgs.msg import Float64MultiArray, String as StringMsg

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        # Declare parameters
        self.declare_parameter('switch_threshold', 1.0)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create turtle2 velocity publisher
        self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)
        
        # Publisher for current target info - используем стандартные сообщения
        self.target_name_publisher = self.create_publisher(StringMsg, '/current_target_name', 10)
        self.target_pose_publisher = self.create_publisher(Float64MultiArray, '/current_target_pose', 10)
        
        # Subscription for manual target switching
        self.switch_subscription = self.create_subscription(
            String,
            '/switch_target',
            self.switch_target_callback,
            10)

        # Target management
        self.targets = ['carrot1', 'carrot2', 'static_target']
        self.current_target_index = 0
        self.current_target = self.targets[self.current_target_index]
        
        # Service for spawning turtle
        self.spawner = self.create_client(Spawn, 'spawn')
        self.turtle_spawning_service_ready = False
        self.turtle_spawned = False

        # Call on_timer function every second
        self.timer = self.create_timer(0.1, self.on_timer)
        
        # Flag to prevent rapid switching
        self.can_switch = True
        
        self.get_logger().info(f"Starting with target: {self.current_target}")

    def switch_target_callback(self, msg):
        if msg.data == 'next' and self.can_switch:
            self.switch_to_next_target()
            # Prevent rapid switching
            self.can_switch = False
            # Reset flag after delay
            self.create_timer(0.5, self.reset_switch_flag)

    def reset_switch_flag(self):
        self.can_switch = True

    def switch_to_next_target(self):
        self.current_target_index = (self.current_target_index + 1) % len(self.targets)
        self.current_target = self.targets[self.current_target_index]
        self.get_logger().info(f"Switched to target: {self.current_target}")

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.current_target
        to_frame_rel = 'turtle2'

        if self.turtle_spawning_service_ready:
            if self.turtle_spawned:
                try:
                    # Get the transform between turtle2 and current target
                    t = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        rclpy.time.Time())
                except TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                    return

                # Calculate distance to target for auto-switching
                distance = math.sqrt(
                    t.transform.translation.x ** 2 +
                    t.transform.translation.y ** 2)
                
                # Check if we should auto-switch
                switch_threshold = self.get_parameter('switch_threshold').get_parameter_value().double_value
                if distance < switch_threshold and self.can_switch:
                    self.switch_to_next_target()
                    # Prevent rapid switching
                    self.can_switch = False
                    self.create_timer(0.5, self.reset_switch_flag)

                # Control logic for turtle2
                msg = Twist()
                scale_rotation_rate = 1.0
                msg.angular.z = scale_rotation_rate * math.atan2(
                    t.transform.translation.y,
                    t.transform.translation.x)

                scale_forward_speed = 0.5
                msg.linear.x = scale_forward_speed * distance

                self.publisher.publish(msg)
                
                # Publish current target information using standard messages
                # Target name
                name_msg = StringMsg()
                name_msg.data = self.current_target
                self.target_name_publisher.publish(name_msg)
                
                # Target pose and distance
                pose_msg = Float64MultiArray()
                pose_msg.data = [
                    t.transform.translation.x,  # target_x
                    t.transform.translation.y,  # target_y
                    distance                    # distance_to_target
                ]
                self.target_pose_publisher.publish(pose_msg)
                
            else:
                if self.result.done():
                    self.get_logger().info(
                        f'Successfully spawned {self.result.result().name}')
                    self.turtle_spawned = True
                else:
                    self.get_logger().info('Spawn is not finished')
        else:
            if self.spawner.service_is_ready():
                # Initialize request with turtle name and coordinates
                request = Spawn.Request()
                request.name = 'turtle2'
                request.x = float(4)
                request.y = float(2)
                request.theta = float(0)
                # Call request
                self.result = self.spawner.call_async(request)
                self.turtle_spawning_service_ready = True
            else:
                # Check if the service is ready
                self.get_logger().info('Service is not ready')

def main():
    rclpy.init()
    node = TurtleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()