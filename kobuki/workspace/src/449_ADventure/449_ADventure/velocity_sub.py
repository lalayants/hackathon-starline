
import rclpy
from rclpy.node import Node
import threading
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from collections import deque
from example_interfaces.srv import SetBool



class VelocitySubscriber(Node):

    def __init__(self):
        
        super().__init__('velocity_subscriber')
        
        self.vel_queue = deque()
        self.cli = self.create_client(SetBool, "toggle_stabilization")

        self.subscription = self.create_subscription(
            Twist,
            'commands/velocity',
            self.listener_callback,
            10)
        self.subscription  # prevent unusekld variable warning
        self.publisher = self.create_publisher(Twist, 'commands/velocity_lost',10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        self.read = False
        self.repeat = False
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            print('service not available, waiting again...')
        self.req = SetBool.Request()

    
    def clean_queue(self):
        self.vel_queue = deque()
    
    def read(self):
        self.read = True
    
    def not_read(self):
        self.read = False

    def repeat(self):
        self.repeat = True
    
    def not_repeat(self):
        self.repeat = False

    def listener_callback(self, msg):
        if self.read:
            self.vel_queue.append(msg)
    
   #     self.file_w.write(f'{msg.linear.x} {msg.linear.y} {msg.linear.z}')
    #    self.file_w.write(f' {msg.angular.x} {msg.angular.y} {msg.angular.z}\n')
     #   print('callback listen')
        return

    def timer_callback(self):
        if not self.repeat:
            return
        if len(self.vel_queue) == 0:
            return 
        vel = self.vel_queue.popleft()
        msg = Twist()
        msg.linear = vel.linear
        msg.angular = vel.angular
        self.publisher.publish(msg)
        return 

def main(args=None):
    rclpy.init(args=args)

    vel_sub = VelocitySubscriber()
    rclpy.spin(vel_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vel_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
