import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class VelocitySubscriber(Node):

    def __init__(self):
        super().__init__('velocity_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'commands/velocity',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'some_vel',10)
	self.timer = self.create_timer(0.1, self.timer_callback)
	self.data = None

    def listener_callback(self, msg):
	self.data = msg.data
	return

    def timer_callback(self):
	msg = Twist()
	msg.data = self.data
	self.publisher_.publish(msg)

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
