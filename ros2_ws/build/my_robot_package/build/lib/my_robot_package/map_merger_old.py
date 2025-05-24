import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import numpy as np
import cv2
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
from transforms3d.euler import euler2quat

from copy import deepcopy

class ManualMapMerger(Node):
    def __init__(self):
        super().__init__('manual_map_merger')

        self.robot_pos1 = (5.45, -2.12, 7.01, 0, 0, -1.60)
        self.robot_pos2 = (.15, 3.52, 7.01, 0, 0, -1.59)

        self.dx = self.robot_pos2[0] - self.robot_pos1[0]
        self.dy = self.robot_pos2[1] - self.robot_pos1[1]
        self.dtheta = self.robot_pos2[5] - self.robot_pos1[5]
        self.get_logger().info(f"dx: {self.dx}, dy: {self.dy}, dtheta: {math.degrees(self.dtheta)}")


        self.map1 = None
        self.map2 = None
        self.map1_sub = self.create_subscription(OccupancyGrid, '/map1', self.map1_callback, 10)
        self.map2_sub = self.create_subscription(OccupancyGrid, '/map2', self.map2_callback, 10)

        self.merged_pub = self.create_publisher(OccupancyGrid, '/merge_map', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        #self.timer = self.create_timer(0.1, self.publish_merged_map_transform)

    def publish_merged_map_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'merge_map'
        t.child_frame_id = 'map2'

        t.transform.translation.x = self.dx
        t.transform.translation.y = self.dy
        t.transform.translation.z = 0.0

        q = euler2quat(0, 0, self.dtheta)
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]

        self.tf_broadcaster.sendTransform(t)

        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'merge_map'
        t2.child_frame_id = 'map1'

        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t2)

    def map1_callback(self, msg):
        self.map1 = msg
        self.try_merge()

    def map2_callback(self, msg):
        self.map2 = msg
        self.try_merge()

    def try_merge(self):
        if self.map1 is None or self.map2 is None:
            return

        res = min(self.map1.info.resolution, self.map2.info.resolution)

        # Origins
        map1_origin_x = self.map1.info.origin.position.x
        map1_origin_y = self.map1.info.origin.position.y
        map2_origin_x = self.map2.info.origin.position.x
        map2_origin_y = self.map2.info.origin.position.y

        # Map dimensions in world coordinates
        map1_max_x = map1_origin_x + self.map1.info.width * self.map1.info.resolution
        map1_max_y = map1_origin_y + self.map1.info.height * self.map1.info.resolution
        map2_max_x = map2_origin_x + self.map2.info.width * self.map2.info.resolution
        map2_max_y = map2_origin_y + self.map2.info.height * self.map2.info.resolution

        # Adjust map2 origin by known transform (dx, dy)
        map2_origin_x += self.dx
        map2_origin_y += self.dy
        map2_max_x += self.dx
        map2_max_y += self.dy

        # Merged map bounds
        min_x = min(map1_origin_x, map2_origin_x+ self.dx)
        min_y = min(map1_origin_y, map2_origin_y + self.dy)
        max_x = max(map1_max_x, map2_max_x)
        max_y = max(map1_max_y, map2_max_y)

        merged_width = int(np.ceil((max_x - min_x) / res))
        merged_height = int(np.ceil((max_y - min_y) / res))
        merged_data = [-1] * (merged_width * merged_height)

        def copy_map_to_merged(map_msg, origin_x, origin_y, dx=0.0, dy=0.0):
            for y in range(map_msg.info.height):
                for x in range(map_msg.info.width):
                    i = x + y * map_msg.info.width
                    val = map_msg.data[i]
                    if val == -1:
                        continue
                    wx = origin_x + x * map_msg.info.resolution + dx
                    wy = origin_y + y * map_msg.info.resolution + dy
                    mx = int((wx - min_x) / res)
                    my = int((wy - min_y) / res)
                    if 0 <= mx < merged_width and 0 <= my < merged_height:
                        mi = mx + my * merged_width
                        if merged_data[mi] == -1:
                            merged_data[mi] = val

        # Copy both maps to merged space
        copy_map_to_merged(self.map1, map1_origin_x, map1_origin_y)
        copy_map_to_merged(self.map2, self.map2.info.origin.position.x, self.map2.info.origin.position.y, self.dx, self.dy)

        # Build OccupancyGrid message
        merged_msg = deepcopy(self.map1)
        merged_msg.header.stamp = self.get_clock().now().to_msg()
        merged_msg.info.width = merged_width
        merged_msg.info.height = merged_height
        merged_msg.info.resolution = res
        merged_msg.info.origin.position.x = min_x
        merged_msg.info.origin.position.y = min_y
        merged_msg.data = merged_data

        self.merged_pub.publish(merged_msg)
        self.get_logger().info("Published merged map.")



def main(args=None):
    rclpy.init(args=args)
    node = ManualMapMerger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
