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
from nav_msgs.msg import Path

import time

class Grid():

    def __init__(self, n_width:int = 6, n_length:int = 5, cell_size:float = 1.0, stage=2):
        self.stage2_poses = [13, 18, 19, 22, 23, 24, 25, 26, 27, 28, 29]
        self.origin_pose = [0.0, 0.0]
        self.zero_cell_pose = (0.8, 2.2)
        self.n_width = n_width
        self.n_length = n_length
        self.cell_size = cell_size
        self.grid = [self.zero_cell_pose]
        self.used = [0]
        self.pred = [-1]
        self.restrictions = [[]]
        self.stage = stage
        self.create_grid()

    def create_grid(self):
        for i in range(self.n_length):
            for j in range(self.n_width):
                if i == 0 and j == 0:
                    continue
                y = -j*self.cell_size + self.zero_cell_pose[1]
                x = i*self.cell_size + self.zero_cell_pose[0]
                self.grid.append((x, y))
                id = i*self.n_width + j
                is_used = 0
                if self.stage == 1 and id in self.stage2_poses:
                    is_used = 1
                self.used.append(is_used)
                self.pred.append(-1)
                self.restrictions.append([])


    def get_nearest_cell(self, robot_pose):
        pass        
            



class GoalPublisher(Node):

    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        timer_period = 0.5  # seconds
        # self.path = (2, 1, 0, 6, 12, 7, 6, 0, 1, 2, 3, 4, 5, 11, 10, 9, 15, 16, 17, 22, 28, 29, 27, 26, 25, 26, 27, 28, 22, 16, 15, 21, 20, 14, 8, 2)
        self.path = (2, 1)
        self.current = -1
        self.goal = 2
        self.grid = Grid(stage=2)
        self.grid.pred[self.goal] = -2
        self.is_reached = False
        self.last_goal_update = time.time()
        self.time_stamp = Clock().now()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.subscription = self.create_subscription(
            Empty,
            '/aruco_found',
            self.aruco_callback,
            1)
        
        self.path_subscription = self.create_subscription(
            Path,
            '/received_global_plan',
            self.path_callback,
            1)
        self.n_found = 0
        self.last_found = time.time()
        self.path_length = 0
        self.path_was_updated = True
        self.cli = self.create_client(SetBool, "toggle_stabilization")
    
    def path_callback(self, msg):
        self.path_length = 0
        for i in range(1, len(msg.poses)):
            self.path_length += ((msg.poses[i].pose.position.x - msg.poses[i - 1].pose.position.x)**2 + (msg.poses[i].pose.position.y - msg.poses[i - 1].pose.position.y)**2)**0.5
            self.path_was_updated = True
        #print("PATH LENGTH:", path_length)

    def aruco_callback(self, msg): 
        print(time.time(),self.last_found)
        if time.time() - self.last_found > 1:
            self.n_found = 1
        else:
            self.n_found += 1
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
        new_goal = False

        if (time.time() - self.last_goal_update) > 180:
            print(f'Cannot reach {self.goal} because of timeout')
            self.grid.restrictions[self.current].append(self.goal)
            self.grid.restrictions[self.goal].append(self.current)
            new_goal = True

        if self.path_was_updated and self.path_length > 1.7:
            self.grid.restrictions[self.current].append(self.goal)
            self.grid.restrictions[self.goal].append(self.current)
            new_goal = True
            print(f'Goal {self.goal} is not neighbour of {self.current}')

        goal_pose = self.grid.grid[self.goal]
        error = ((pose[0] - goal_pose[0])**2 + (pose[1] - goal_pose[1])**2)**(0.5) 
        
        if error < 0.3 and not new_goal:
            new_goal = True
            if self.grid.pred[self.goal] == -1:
                self.grid.pred[self.goal] = self.current
            self.grid.used[self.goal] = 1
            self.current = self.goal
            
            print('Reached goal: ', self.goal)

        if new_goal:
            print('Finding new goal...')
            flag = True
            
            # print(self.goal % self.grid.n_width)
            if self.current % self.grid.n_width > 0:
                possible_goal = self.current - 1
                if self.grid.used[possible_goal] == 0 and not (possible_goal in self.grid.restrictions[self.current]):
                    # Has left neigbour
                    self.goal = possible_goal
                    flag = False
            if (self.current < self.grid.n_length * (self.grid.n_width - 1) - 1):
                possible_goal = self.current + self.grid.n_width  
                if self.grid.used[possible_goal] == 0 and flag and not (possible_goal in self.grid.restrictions[self.current]):
                    # Has up neigbour
                    self.goal = possible_goal
                    flag = False
            if self.current % self.grid.n_width < self.grid.n_width - 1:
                possible_goal = self.current + 1
                if self.grid.used[possible_goal] == 0 and flag and not (possible_goal in self.grid.restrictions[self.current]):
                    # Has right neigbour
                    self.goal = possible_goal
                    flag = False
            if self.current >= self.grid.n_width:
                possible_goal = self.current - self.grid.n_width
                if self.grid.used[possible_goal] == 0 and flag and not (possible_goal in self.grid.restrictions[self.current]):
                    # Has down neigbour
                    self.goal = possible_goal
                    flag = False
            if flag:
                self.goal = self.grid.pred[self.current]
                
            self.last_goal_update = time.time()
            self.path_was_updated = False
            print('Next goal: ', self.goal)
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