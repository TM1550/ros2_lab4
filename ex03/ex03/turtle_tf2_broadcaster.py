import math
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TurtleTf2Broadcaster(Node):

    def __init__(self):
        super().__init__('turtle_tf2_broadcaster')
        self.turtlename = self.declare_parameter('turtlename', 'turtle1').get_parameter_value().string_value
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.subscribe_pose()

    def subscribe_pose(self):
        topic_name = f'/{self.turtlename}/pose'
        subscription = self.create_subscription(
            Pose,
            topic_name,
            self.handle_turtle_pose,
            1)
        return subscription

    def handle_turtle_pose(self, msg):
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename
        
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        
        q = self.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci * ck
        cs = ci * sk
        sc = si * ck
        ss = si * sk

        q = [
            cj * sc - sj * cs,
            cj * ss + sj * cc,
            cj * cs - sj * sc,
            cj * cc + sj * ss,
        ]
        return q

def main():
    rclpy.init()
    node = TurtleTf2Broadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()