import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class VelocitySubscriber(Node):

    def __init__(self):
        super().__init__('velocity_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'commands/velocity',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Twist, 'commands/velocity_lost',10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.linear = Vector3()
        self.linear.x = 0.0
        self.linear.y = 0.0
        self.linear.z = 0.0

        self.angular = Vector3()
        self.angular.x = 0.0
        self.angular.y = 0.0
        self.angular.z = 0.0

    def listener_callback(self, msg):
        self.linear = msg.linear
        self.angular = msg.angular

        return

    def timer_callback(self):
        msg = Twist()
        msg.linear = self.linear
        msg.angular = self.angular
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = VelocitySubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
