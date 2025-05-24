import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import os

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        self.subscriber = self.create_subscription(
            PoseStamped,
            '/model/X1_1/pose',
            self.pose_callback,
            10
        )
        
        self.record_interval = .75  # seconds
        self.last_record_time = self.get_clock().now().nanoseconds
        self.waypoints = []

    def pose_callback(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        elapsed = (now_ns - self.last_record_time) / 1e9
        #self.get_logger().info(f"Elapsed time: {elapsed:.2f} seconds")

        if elapsed >= self.record_interval and msg.header.frame_id == 'harmonic':
            pos = msg.pose.position
            self.waypoints.append((pos.x, pos.y))
            self.last_record_time = now_ns
            self.get_logger().info(f"Recorded waypoint: ({pos.x:.2f}, {pos.y:.2f})")

    def save_waypoints_to_file(self):
        output_path = os.path.expanduser('waypoints1.csv')
        with open(output_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for wp in self.waypoints:
                writer.writerow(wp)
        self.get_logger().info(f"Waypoints saved to: {output_path}")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down and saving waypoints...")
        node.save_waypoints_to_file()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
