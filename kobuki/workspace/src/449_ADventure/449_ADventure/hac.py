import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from example_interfaces.srv import SetBool
from std_msgs.msg import Empty

import numpy as np


def linear_vel(t, k=1):
    return max(min(k*t, 1.) ,-1)

class VictimForwardMove(Node):

    def __init__(self):
        super().__init__('victim_move')
        self.service_ = self.create_service(
            SetBool, "move_forward", self.move)
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom_lost',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.pose = None
        self.publisher = self.create_publisher(Twist, 'commands/velocity_lost',10)
        self.zero_odom_publisher = self.create_publisher(Empty, '/commands/reset_odometry_lost',1)
        
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
        
        self.enabled_ = False
        self.forward = True
        self.first_time = True
        
    def move(self, request, response):
        self.enabled_ = request.data 
        return response

    def listener_callback(self, msg):
        if self.first_time:
            self.zero_odom_publisher.publish(Empty())
            return
        print("GOT",msg.pose.pose.position.x)
        self.pose = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.orientation = yaw
        if not self.enabled_:
            return
        
        
        if  self.forward:
            if abs(self.pose.x - (0.5 + self.first_pose.x)) > 0.01:
                print(self.pose.x)
                self.linear.x = float(linear_vel(0.5 - self.pose.x+ self.first_pose.x, 0.6))
                self.msg.linear = self.linear
                self.publisher.publish(self.msg)
            else:
                # self.linear.x = 0.
                # self.msg.linear = self.linear
                # self.publisher.publish(self.msg)
                self.forward = False
        if not self.forward :
            goal = np.pi             
            if abs(self.orientation - (goal + self.first_orientation)) > 0.05:
                print(self.orientation)
                self.angular.z = float(linear_vel(-self.orientation + goal + self.first_orientation, 0.7))
                self.msg.angular = self.angular
                self.publisher.publish(self.msg)
            else:
                # self.angular.z = 0.
                # self.msg.angular = self.angular
                # self.publisher.publish(self.msg) 
                self.enabled_ = False
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