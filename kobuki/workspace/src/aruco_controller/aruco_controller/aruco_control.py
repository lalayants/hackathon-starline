import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
# from aruco_detector import ArucoDetector


CENTER_MARKER = 1
LEFT_MARKERS = [3, 4]
RIGHT_MARKERS = [5, 6]

class ArucoController(Node):

    def __init__(self):
        super().__init__('aruco_control')
        self.bridge = CvBridge()
        # self.aruco_detector = ArucoDetector()

        # self.publisher_saver = self.create_publisher(Twist, 'commands_saver/velocity', 1)
        # self.publisher_lost = self.create_publisher(Twist, 'commands_lost/velocity', 1)
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.control_callback,
            qos_profile_sensor_data)
        self.i = 0
        # self.msg_saver = Twist()
        # self.msg_lost = Twist()
        # self.last_dir = 1 # left
        
        self.calib_mat = np.load("/workspace/calibration_matrix.npy")
        self.dist_mat = np.load("/workspace/distortion_coefficients.npy")
        print("Node started!")

    def control_callback(self, msg): 
        print("Got image!") 
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict)
        
        res, corners, ids = self.aruco_detector.get_info(frame)
        if res:
            if set(ids) & set([CENTER_MARKER]):
                print("Center aruco are found!")
                if len(corners) > 0:
                    for i in range(0, len(ids)):
                        # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                        rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, self.calib_mat,
                                                                                self.dist_mat)
                        print("ROT:",rvec)
                        print("TRANS:",tvec)


def main(args=None):
    rclpy.init(args=args)

    aruco_controller = ArucoController()

    rclpy.spin(aruco_controller)

    aruco_controller.out.release()
    aruco_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()