import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np



class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')
        self.publisher = self.create_publisher(OccupancyGrid, '/merge_map', 10)
        self.subscription = self.create_subscription(OccupancyGrid, '/map1', self.map1_callback, 10)
        self.subscription = self.create_subscription(OccupancyGrid, '/map2', self.map2_callback, 10)
        self.map1 = None
        self.map2 = None

    def merge_maps(self, map1, map2):

        if self.map1 is None or self.map2 is None:
            self.get_logger().info('Waiting for both maps to be available')
            return None
        

        merged_map = OccupancyGrid()
        merged_map.header = map1.header
        merged_map.header.frame_id = 'merge_map'
        

        robot_pos1 = (5.45, -2.12, 7.01, 0, 0, -1.60)
        robot_pos2 = (.15, 3.52, 7.01, 0, 0, -1.60)

        dx = robot_pos2[0] - robot_pos1[0]
        dy = robot_pos2[1] - robot_pos1[1]

        map1_origin_x = map1.info.origin.position.x
        map1_origin_y = map1.info.origin.position.y
        

        # im not sure why, but y needs to be '+' and x '-'
        # would expect them to be the same sign
        map2_origin_x = map2.info.origin.position.x - dx
        map2_origin_y = map2.info.origin.position.y + dy


        min_x = min(map1_origin_x, map2_origin_x)
        min_y = min(map1_origin_y, map2_origin_y)
        max_x = max(map1_origin_x + (map1.info.width * map1.info.resolution), map2_origin_x + (map2.info.width * map2.info.resolution))
        max_y = max(map1_origin_y + (map1.info.height * map1.info.resolution),map2_origin_y + (map2.info.height * map2.info.resolution))
        

        merged_map.info.origin.position.x = min_x
        merged_map.info.origin.position.y = min_y
        merged_map.info.resolution = min(map1.info.resolution, map2.info.resolution)
        merged_map.info.width = int(np.ceil((max_x - min_x) / merged_map.info.resolution))
        merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution))
        merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)


        for y in range(map1.info.height):
            for x in range(map1.info.width):
                i = x + y * map1.info.width
                wx = map1_origin_x + x * map1.info.resolution
                wy = map1_origin_y + y * map1.info.resolution
                mx = int((wx - min_x) / merged_map.info.resolution)
                my = int((wy - min_y) / merged_map.info.resolution)
                mi = mx + my * merged_map.info.width
                merged_map.data[mi] = map1.data[i]

        for y in range(map2.info.height):
            for x in range(map2.info.width):
                i = x + y * map2.info.width
                wx = map2_origin_x + x * map2.info.resolution
                wy = map2_origin_y + y * map2.info.resolution
                mx = int((wx - min_x) / merged_map.info.resolution)
                my = int((wy - min_y) / merged_map.info.resolution)
                mi = mx + my * merged_map.info.width
                if merged_map.data[mi] == -1:
                    merged_map.data[mi] = map2.data[i]

            self.publisher.publish(merged_map)
            self.get_logger().info('Merged map published')

    def map1_callback(self, msg):
        self.map1 = msg
        msg = self.merge_maps(self.map1, self.map2)

    
    def map2_callback(self, msg):
        self.map2 = msg
        msg = self.merge_maps(self.map1, self.map2)

def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()
    rclpy.spin(merge_map_node)
    merge_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()