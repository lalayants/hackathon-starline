import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from scipy.spatial.transform import Rotation as R


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
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.control_callback,
            qos_profile_sensor_data)
        self.i = 0
        self.msg_saver = Twist()
        self.msg_lost = Twist()
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
        
        if len(corners) > 0:
            print(f"Found {len(corners)} markers: {list(np.unique(ids))}")
            for i in range(0, len(ids)):
                # print(corners[i])
                if bool(set(ids[i]) & set(CENTER_MARKERS)):
                    print("Center aruco is found!")
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, self.calib_mat,
                                                                            self.dist_mat)
                    r = R.from_rotvec(rvec[0,0])
                    print("Rotated: ",r.as_euler("xyz")[1])
                    self.msg_lost.angular.z = r.as_euler("xyz")[1] * 3
                    self.msg_saver.angular.z = ((frame.shape[1]/2 - 50) - np.mean(np.array(corners[i])[:,1])) * 0.03
                    
                    rect = cv2.minAreaRect(corners[i])
                    box = cv2.boxPoints(rect)
                    area = cv2.contourArea(box)
                    distance = dist(area)
                    
                    # print("Area:", area)
                    # print("Distance:", dist(area))
                    break
                elif bool(set(ids[i]) & set(RIGHT_MARKERS)):
                    print("Right aruco is found!")
                    break
                elif bool(set(ids[i]) & set(LEFT_MARKERS)):
                    print("Left aruco is found!")
                    break
                elif bool(set(ids[i]) & set(BACK_MARKERS)):
                    print("Back aruco is found!")
                    break
                else:
                    print("Unknown marker found!")
        else:
            print("No markers found")
        
        self.publisher_lost.publish(self.msg_lost)
        self.msg_lost.angular.z = 0.
        self.publisher_saver.publish(self.msg_saver)
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