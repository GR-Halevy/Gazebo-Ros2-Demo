from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import rclpy

class GazeboTFBridge(Node):
    def __init__(self):
        super().__init__('gazebo_tf_bridge')
        self.br = TransformBroadcaster(self)

        # Subscriptions for both robots
        self.sub1 = self.create_subscription(
            Odometry,
            '/model/X1_1/odometry',
            self.odom_callback_X1_1,
            10
        )
        self.sub2 = self.create_subscription(
            Odometry,
            '/model/X1_2/odometry',
            self.odom_callback_X1_2,
            10
        )

    def publish_tf(self, msg, robot_name):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = f"{robot_name}_odom"
        t.child_frame_id = f"{robot_name}_base_link"


        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)

    def odom_callback_X1_1(self, msg):
        self.publish_tf(msg, "X1_1")

    def odom_callback_X1_2(self, msg):
        self.publish_tf(msg, "X1_2")


def main():
    rclpy.init()
    node = GazeboTFBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
