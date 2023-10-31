
import rclpy
from rclpy.node import Node
import threading
from geometry_msgs.msg import Twist, PoseStamped
from geometry_msgs.msg import Vector3
from collections import deque
from example_interfaces.srv import SetBool
import time

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import numpy as np



class HeroToHome(Node):

    def __init__(self):
        
        super().__init__('hero_to_home')
        
        self.vel_queue = deque()
        self.cli_read = self.create_client(SetBool, "lost/read")
        self.cli_repeat = self.create_client(SetBool, "lost/repeat")
        self.cli_move_away = self.create_client(SetBool, "move_hero")
        self.service_ = self.create_service(
            SetBool, "move_to_home", self.move)
        self.enabled_ = False
        self.published = False

        
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose',10)
    
        
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        

    def move(self, request, response):
        self.enabled_ = request.data 
        return response

    

    def timer_callback(self):
        if not self.enabled_:
            return
        if  not self.published:
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.x = 0.0
            msg.pose.position.y = -0.6
            msg.pose.position.z = 0.0
            
            req = SetBool.Request()
            req.data = True
            self.cli_read.call_async(req) 
            self.publisher.publish(msg)
            self.published = True
            time.sleep(3)
            req = SetBool.Request()
            req.data = True
            self.cli_repeat.call_async(req)
        
        try:
            t = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time())
            if np.sqrt(t.transform.x **2 + t.transform.y ** 2) < 0.1:
                req = SetBool.Request()
                req.data = False
                self.cli_read.call_async(req)
                time.sleep(2)
                req = SetBool.Request()
                req.data = True
                self.cli_move_away.call_async(req)
                
                
                self.enabled_ = False
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform map to base_link: {ex}')
            return
        
        
        
        return 

def main(args=None):
    rclpy.init(args=args)

    vel_sub = HeroToHome()
    rclpy.spin(vel_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vel_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
