import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock
# Stores known frames and offers frame graph requests
from tf2_ros.buffer import Buffer 

# Easy way to request and receive coordinate frame transform information
from tf2_ros.transform_listener import TransformListener 

from tf2_ros import TransformException 
from std_msgs.msg import Empty
from example_interfaces.srv import SetBool
import time

class Grid():

    def __init__(self, n_width:int = 6, n_length:int = 5, cell_size:float = 1.0):
        self.origin_pose = [0.0, 0.0]
        self.zero_cell_pose = (0.8, 2.2)
        self.n_width = n_width
        self.n_length = n_length
        self.cell_size = cell_size
        self.grid = [self.zero_cell_pose]

        self.create_grid()

    def create_grid(self):
        for i in range(self.n_length):
            for j in range(self.n_width):
                if i == 0 and j == 0:
                    continue
                y = -j*self.cell_size + self.zero_cell_pose[1]
                x = i*self.cell_size + self.zero_cell_pose[0]
                self.grid.append((x, y))

    def get_nearest_cell(self, robot_pose):
        pass        
            



class GoalPublisher(Node):

    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        timer_period = 0.5  # seconds
        self.path = (2, 1, 0, 6, 12, 7, 6, 0, 1, 2, 3, 4, 5, 11, 10, 9, 15, 16, 17, 22, 28, 29, 27, 26, 25, 26, 27, 28, 22, 16, 15, 21, 20, 14, 8, 2)
        self.goal = 0
        self.grid = Grid()
        self.is_reached = False
        self.time_stamp = Clock().now()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.subscription = self.create_subscription(
            Empty,
            '/aruco_found',
            self.aruco_callback,
            1)
        self.n_found = 0
        self.last_found = time.time()
        
        self.cli = self.create_client(SetBool, "toggle_stabilization")
    
    def aruco_callback(self, msg): 
        if time.time() - self.last_found > 1:
            self.n_found = 1
        
        if self.n_found == 10:
            self.req = SetBool.Request()
            self.req.data = True
            self.cli.call_async(self.req)
            self.destroy_node()
        
        self.last_found = time.time()
            
    
    def get_robot_pose(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                        'map',
                        'base_link',
                        now)
            
            return (trans.transform.translation.x, trans.transform.translation.y)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base_link to map: {ex}')
            return None


    def timer_callback(self):
        pose = self.get_robot_pose()
        if pose is None:
            return 
        
        goal_pose = self.grid.grid[self.path[self.goal]]
        error = ((pose[0] - goal_pose[0])**2 + (pose[1] - goal_pose[1])**2)**(0.5) 
        print(pose, goal_pose, error)
        if error < 0.4:
            if self.goal < len(self.path) - 1: 
                self.goal +=1

        pose = PoseStamped()

        pose.header.frame_id = 'map'
        pose.header.stamp = self.time_stamp.to_msg()

        pose.pose.position.x = goal_pose[0]
        pose.pose.position.y = goal_pose[1]
        pose.pose.position.z = 0.0

        # quaternion = tf.transformations.quaternion_from_euler(
        #     0, 0, -math.radians(wp[0].transform.rotation.yaw))
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        # msg.poses.append(pose)
        self.publisher_.publish(pose)
        self.get_logger().info('Publishing')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = GoalPublisher()

    rclpy.spin(minimal_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
