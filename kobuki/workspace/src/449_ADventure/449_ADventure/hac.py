import rclpy
from rclpy.node import Node
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from example_interfaces.srv import SetBool


def linear_vel(t, k=1):
    return max(k*t, 1) 

class VictimForwardMove(Node):

    def __init__(self):
        super().__init__('victim_move')
        self.service_ = self.create_service(
            SetBool, "move_forward", self.move)
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.pose = None
        self.publisher = self.create_publisher(Twist, 'commands/velocity_lost',10)
        self.msg = Twist()
        self.linear = Vector3()
        self.linear.x = 0.0
        self.linear.y = 0.0
        self.linear.z = 0.0

        self.angular = Vector3()
        self.angular.x = 0.0
        self.angular.y = 0.0
        self.angular.z = 0.0
        self.msg.angular = self.angular
        
    def move(self, request, response):
        if not request.data :
            return response
        
        while abs(self.pose.x - 0.5) > 0.001:
            self.linear.x = linear_vel(self.pose.x - 0.5)
            self.msg.linear = self.linear
            self.publisher.publish(self.msg)
        else:
            self.linear.x = 0
            self.msg.linear = self.linear
            self.publisher.publish(self.msg)
        while abs(self.orientation - 3.14) > 0.01:
            self.angular.z = linear_vel(self.orientation - 3.14)
            self.msg.angular = self.angular
            self.publisher.publish(self.msg)
        else:
            self.angular.z = 0
            self.msg.angular = self.angular
            self.publisher.publish(self.msg)
        return response

    def listener_callback(self, msg):
        self.pose = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.orientation = yaw
        return


def main(args=None):
    rclpy.init(args=args)

    move = VictimForwardMove()

    rclpy.spin(move)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()