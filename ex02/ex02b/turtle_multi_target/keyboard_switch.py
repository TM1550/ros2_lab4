import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import termios
import tty
import sys
import select

class KeyboardSwitch(Node):
    def __init__(self):
        super().__init__('keyboard_switch')
        self.publisher = self.create_publisher(String, '/switch_target', 10)
        
        # Set up terminal for non-blocking input
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info("Keyboard switch node started")
        self.get_logger().info("Press 'n' to switch to next target, 'q' to quit")
        
        self.timer = self.create_timer(0.1, self.check_keyboard)
        
    def check_keyboard(self):
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)
            if key == 'n':
                msg = String()
                msg.data = 'next'
                self.publisher.publish(msg)
                self.get_logger().info("Manual switch command sent")
            elif key == 'q':
                self.cleanup()
                rclpy.shutdown()
                return
                
    def cleanup(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

def main():
    # Set terminal to non-blocking mode
    tty.setraw(sys.stdin.fileno())
    
    rclpy.init()
    node = KeyboardSwitch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()