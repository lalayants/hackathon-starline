from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from explorer_interfaces.action import Wander
from nav2_msgs.action import NavigateToPose

from std_msgs.msg import Float32
from visualization_msgs.msg import MarkerArray

import rclpy
import math
from rclpy.action import ActionClient
from rclpy.node import Node

from rclpy.node import Node
from rcl_interfaces.srv import GetParameters


class Manager(Node):

    def __init__(self):
        super().__init__('manager')
        self._action_client_wanderer = ActionClient(self, Wander, 'wander')
        self.navigation_client = NavigationClient()
        self.trajectory_subscription = self.create_subscription(MarkerArray,'trajectory_node_list',self.trajectory_callback,10)
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.map_explored=0.01
        self.map_finished=False
        self.trajectory_distance=0.0
        self.trajectory_markers=MarkerArray()
        self.start_time=self.get_clock().now()


    def print_feedback(self):
        try:
            self.map_explored="{:.2f}".format(self.map_explored) #Crop to 2 decimals
            self.trajectory_distance=self.compute_distance_from_markers(self.trajectory_markers)
            self.trajectory_distance="{:.2f}".format(self.trajectory_distance) #Crop to 2 decimals
            time_now=self.get_clock().now()
            duration=str(int((time_now.nanoseconds-self.start_time.nanoseconds)/(10**9)))
            self.get_logger().info("Duration: %s s - Map: %s - Distance: %s m " %(duration, self.map_explored, self.trajectory_distance))
        except:
            pass

    def timer_callback(self):
        #Print feedback in terminal acording to timer_period
        if not self.map_finished:
            self.print_feedback()

    def trajectory_callback(self, msg):
        self.trajectory_markers=msg.markers 

    def compute_distance_from_markers(self, markers):
        trajectory_distance=0.0
        last_point=[0,0]
        try:
            for marker in self.trajectory_markers:
                marker_points=marker.points     
                for point in marker_points:
                    point=[point.x, point.y]
                    trajectory_distance+=math.dist(last_point, point)
                    last_point=point
            return trajectory_distance
        except:
            self.get_logger().warn("Trajectory not received yet")


    def goal_response_callback_wanderer(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Exploration goal rejected')
            return

        self.get_logger().info('Exploration goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback_wanderer)

    def feedback_callback_wanderer(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.sequence))

    def get_result_callback_wanderer(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.map_finished=True
            self.get_logger().info('MAP SUCCESSFULLY EXPLORED')
            self.print_feedback()
            #Return to home
            self.navigation_client.send_goal()
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

    def send_goal_wanderer(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client_wanderer.wait_for_server()

        goal_msg = Wander.Goal()
        goal_msg.map_completed_thres = 0.9

        self.get_logger().info('Sending wanderer goal request...')
        self.get_logger().info('Wandering until 90% map completed')

        self._send_goal_future = self._action_client_wanderer.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback_wanderer)

        self._send_goal_future.add_done_callback(self.goal_response_callback_wanderer)
    
class NavigationClient(Node):

    def __init__(self):
        super().__init__('navigation_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Exploration goal rejected')
            return

        self.get_logger().info('Navigation goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Arrived to home position')
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))
            
    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.orientation.w=1.0 #Home position

        self.get_logger().info('Returning to base...')

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)


def main(args=None):
    rclpy.init(args=args)

    manager = Manager()

    manager.send_goal_wanderer()
    rclpy.spin(manager)

if __name__ == '__main__':
    main()
