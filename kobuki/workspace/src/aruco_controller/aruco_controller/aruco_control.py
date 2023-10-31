import time
import rclpy
import math

from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, Range
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from scipy.spatial.transform import Rotation as R
from example_interfaces.srv import SetBool

CENTER_MARKERS = [1]
LEFT_MARKERS = [2]
RIGHT_MARKERS = [3]
BACK_MARKERS = [4]

def dist(ar):
    return 42.93206 * ar**-0.478856

class ArucoController(Node):

    def __init__(self):
        super().__init__('aruco_control')
        self.bridge = CvBridge()
        # self.aruco_detector = ArucoDetector()

        self.publisher_saver = self.create_publisher(Twist, 'commands/velocity', 1)
        self.publisher_lost = self.create_publisher(Twist, 'commands/velocity_lost', 1)
        self.publisher_distance = self.create_publisher(Range, 'distance_aruco', 1)
        
        self.publisher_found = self.create_publisher(Empty, 'aruco_found', 1)
        self.service_ = self.create_service(
            SetBool, "toggle_stabilization", self.toggle_stabilization)
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.control_callback,
            qos_profile_sensor_data)
        self.msg_saver = Twist()
        self.msg_lost = Twist()
        self.msg_dist = Range()

        self.activated_ = False
        
        self.calib_mat = np.load("/workspace/calibration_matrix.npy")
        self.dist_mat = np.load("/workspace/distortion_coefficients.npy")
        print("Node started!")
        
    def toggle_stabilization(self, request, response):
        self.activated_ = request.data
        response.success = True
        if self.activated_:
            response.message = "Stabilization activated"
        else:
            response.message = "Stabilization deactivated"
        return response

    def control_callback(self, msg): 
        
        print("Got image!") 
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        
        
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict)
        
        if len(corners) > 0:
            print(f"Found {len(corners)} markers: {list(np.unique(ids))}")
            self.publisher_found.publish(Empty())
            
            if not self.activated_:
                return
            for i in range(0, len(ids)):
                # print(corners[i])
                if bool(set(ids[i]) & set(CENTER_MARKERS)):
                    print("Center aruco is found!")
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, self.calib_mat,
                                                                            self.dist_mat)
                    r = R.from_rotvec(rvec[0,0])
                    print("Rotated: ",r.as_euler("xyz")[1])
                    err_finder_center = ((frame.shape[1]/2 - 35) - np.mean(np.array(corners[i])[:,1]))
                    err_lost_orient = r.as_euler("xyz")[1]
                    
                    self.msg_saver.angular.z = err_finder_center * 0.03
                    self.msg_lost.angular.z = err_lost_orient * 3
                    if abs(err_lost_orient) < 0.05 and abs(err_finder_center) < 5:
                        rect = cv2.minAreaRect(corners[i])
                        box = cv2.boxPoints(rect)
                        area = cv2.contourArea(box)
                        distance = dist(area)
                        self.msg_dist.range = distance
                        self.publisher_distance.publish(self.msg_dist)
                        print("Distance:",distance)
                        lost_linear_speed = -(0.5 - distance) * 2
                        self.msg_lost.linear.x = min(0.4, abs(lost_linear_speed)) * (1 if lost_linear_speed > 0 else -1)
                        if abs(0.5 - distance) < 0.01:
                            print("Stabilized!")
                            self.activated_ = False
                            # TODO: call services
                            
                            

                    break
                elif bool(set(ids[i]) & set(RIGHT_MARKERS)):
                    print("Right aruco is found!")
                    self.msg_lost.angular.z = -1.
                    
                elif bool(set(ids[i]) & set(LEFT_MARKERS)):
                    print("Left aruco is found!")
                    self.msg_lost.angular.z = 1.
                    
                elif bool(set(ids[i]) & set(BACK_MARKERS)):
                    print("Back aruco is found!")
                    self.msg_lost.angular.z = 1.
                    
                else:
                    print("Unknown marker found!")
        else:
            print("No markers found")
            return
        
        self.publisher_lost.publish(self.msg_lost)
        self.publisher_saver.publish(self.msg_saver)
        self.msg_lost.linear.z = 0.
        self.msg_lost.linear.x = 0.
        self.msg_saver.angular.z = 0.

def main(args=None):
    rclpy.init(args=args)

    aruco_controller = ArucoController()

    rclpy.spin(aruco_controller)

    aruco_controller.out.release()
    aruco_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()