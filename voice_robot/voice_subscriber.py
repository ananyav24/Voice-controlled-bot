import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class VoiceSubscriber(Node):
    def __init__(self):
        super().__init__('voice_subscriber')
        self.subscription = self.create_subscription(
            String,
            'voice_cmd',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("VoiceSubscriber started and listening for commands...")

    def listener_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f"Received command: {command}")
        twist = Twist()

        if command == "forward":
            twist.linear.x = 0.2
        elif command == "backward":
            twist.linear.x = -0.2
        elif command == "left":
            twist.angular.z = 0.5
        elif command == "right":
            twist.angular.z = -0.5
        elif command == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            self.get_logger().info("Command not recognized")

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

