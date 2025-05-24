import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseArray, PoseStamped
import os
from math import atan2, sqrt, sin, cos


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        


        self.declare_parameter('robot_name', 'X1_1')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        self.map_received = False
        #self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 2)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map'+self.robot_name[-1], self.map_callback, 2)

        if self.robot_name == 'X1_1':
            waypoints_file = 'waypoints1.csv'
        elif self.robot_name == 'X1_2':
            waypoints_file = 'waypoints2.csv'

        if os.path.exists(waypoints_file):
            with open(waypoints_file, 'r') as f:
                lines = f.readlines()
                self.waypoints = []
                for line in lines[1:]:
                    x, y = map(float, line.strip().split(','))
                    self.waypoints.append((x, y))
            self.get_logger().info(f"Loaded waypoints from {waypoints_file}")
        else:
            self.get_logger().error(f"Waypoints file {waypoints_file} not found. Please record waypoints first.")
            self.waypoints = []
        
        self.current_waypoint_index = 0
        self.position = None
        self.yaw = 0.0

        self.publisher = self.create_publisher(Twist, f'/model/{self.robot_name}/cmd_vel', 50)
        self.subscriber = self.create_subscription(PoseStamped, f'/model/{self.robot_name}/pose', self.pose_callback, 50)

        self.get_logger().info(f"Robot name: {self.robot_name}")
        self.get_logger().info(f"Publishing to /model/{self.robot_name}/cmd_vel")
        self.get_logger().info(f"Subscribing to /model/{self.robot_name}/pose")

        self.timer = self.create_timer(0.1, self.control_loop)

        # if os.path.exists('waypoints2.csv'):
        #     with open('waypoints2.csv', 'r') as f:
        #         lines = f.readlines()
        #         self.waypoints = []
        #         for line in lines[1:]:
        #             x, y = map(float, line.strip().split(','))
        #             self.waypoints.append((x, y))
        #     self.get_logger().info(f"Loaded waypoints: {self.waypoints}")
        # else:
        #     self.get_logger().error("Waypoints file not found. Please record waypoints first.")
        #     self.waypoints = []
        
        # self.current_waypoint_index = 0

        # self.publisher = self.create_publisher(Twist, '/model/X1/cmd_vel', 50)        
        # self.subscriber = self.create_subscription(PoseStamped, '/model/X1_2/pose', self.pose_callback, 50)

        # self.position = None

        # self.yaw = 0.0

        # self.timer = self.create_timer(0.1, self.control_loop)
    def map_callback(self, msg):
        if not self.map_received:
            self.get_logger().info("Map received for the first time!")
            self.map_received = True

            # Destroy subscription to stop listening further
            #self.map_sub.destroy()
            #self.get_logger().info("Unsubscribed from /map topic.")

    def pose_callback(self, msg):
        if not self.map_received:
            self.get_logger().warn(f"Waiting for /map{self.robot_name[-1]} topic before processing pose.")
            return    

        if msg.header.frame_id == 'harmonic': 
            self.position = msg.pose.position
            self.yaw = self.get_yaw_from_quaternion(msg.pose.orientation)

    def get_yaw_from_quaternion(self, orientation):
        from transforms3d.euler import quat2euler
        quat = (
            orientation.w,
            orientation.x,
            orientation.y,
            orientation.z
        )
        # Returns angles in radians: (roll, pitch, yaw)
        _, _, yaw = quat2euler(quat)
        return yaw


    def control_loop(self):
        # these 2 ifs are split so logging isnt spammed
        if self.position is None:
            return
        if self.current_waypoint_index >= len(self.waypoints):
            stop = Twist()
            stop.linear.x = 0.0
            stop.angular.z = 0.0
            self.publisher.publish(stop)
            self.get_logger().info("Stopping robot")
            return

        goal_x, goal_y = self.waypoints[self.current_waypoint_index]
        dx = goal_x - self.position.x
        dy = goal_y - self.position.y
        distance = sqrt(dx**2 + dy**2)

        if distance < 0.25:
            # self.get_logger().info(f"Reached waypoint {self.current_waypoint_index}, with distance {distance}")
            # self.get_logger().info(f"Current position: {self.position.x}, {self.position.y}")
            # self.get_logger().info(f"Waypoint: {goal_x}, {goal_y}")
            self.get_logger().info(f"Robot: {self.robot_name}, Reached waypoint {self.current_waypoint_index}, with distance {distance}")
            self.current_waypoint_index += 1
            return

        angle_to_goal = atan2(dy, dx)
        angle_diff = atan2(sin(angle_to_goal - self.yaw), cos(angle_to_goal - self.yaw))

        cmd = Twist()
        K_linear = .8 #1.3
        K_angular = 1.5

        cmd.linear.x = K_linear * distance
        cmd.linear.x = min(cmd.linear.x, 2.0)

        cmd.angular.z = K_angular * angle_diff
        cmd.angular.z = min(cmd.angular.z, 1.0)

        self.publisher.publish(cmd)
        self.get_logger().info(f"Publishing cmd: {cmd.linear.x}, {cmd.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
