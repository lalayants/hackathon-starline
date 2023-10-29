import time
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from aruco_detector import ArucoDetector


CENTER_MARKERS = [0, 1, 2]
LEFT_MARKERS = [3, 4]
RIGHT_MARKERS = [5, 6]

class ArucoController(Node):

    def __init__(self):
        super().__init__('ibvs_controller')
        self.cap = cv2.VideoCapture(1)
        self.bridge = CvBridge()
        self.aruco_detector = ArucoDetector()

        self.publisher_saver = self.create_publisher(Twist, 'commands_saver/velocity', 1)
        self.publisher_lost = self.create_publisher(Twist, 'commands_lost/velocity', 1)
        self.subscription = self.create_subscription(
            Image,
            'camera/color',
            self.control_callback,
            1)
        
        self.msg_saver = Twist()
        self.msg_lost = Twist()
        
        print("Node started!")

    def control_callback(self, msg): 
        print("Got image!") 
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        ref_point = int(frame.shape[1] / 2), int(frame.shape[0])

        res, corners, ids = self.aruco_detector.detect(frame)
        if res:
            if set(ids) == set(CENTER_MARKERS):
                print("Center aruco are found!")
                self.msg_saver.angular.z = 0.1 * (ref_point[0] - corners[1,:,0].mean())
            elif bool(set(ids) & set(LEFT_MARKERS)):
                print("Left aruco are found!")
                self.msg_lost.angular.z = -1
            elif bool(set(ids) & set(RIGHT_MARKERS)):
                print("Right aruco are found!")
                self.msg_lost.angular.z = 1
            else:
                print("Aruco are not found! Rotating lost robot...")
                self.msg_lost.angular.z = 1

        # self.msg.angular.z = self.current_dir*60.
        # self.msg.linear.x = 0.


        self.publisher_lost.publish(self.msg_lost)
        self.publisher_saver.publish(self.msg_saver)
        
        # Zeroing velocities
        self.msg_lost.angular.z = 0
        self.msg_saver.angular.z = 0
        self.msg_lost.linear.x = 0
        self.msg_saver.linear.z = 0


def main(args=None):
    rclpy.init(args=args)

    aruco_controller = ArucoController()

    rclpy.spin(aruco_controller)

    aruco_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()