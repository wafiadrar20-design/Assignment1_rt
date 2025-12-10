import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
import math

class DistanceNode(Node):
    def __init__(self):
        super().__init__('distance_node')

        self.pose1 = None
        self.pose2 = None
        self.moving_turtle = ""

        self.threshold = 0.8
        self.min_b = 1.0
        self.max_b = 10.0

        self.distance_pub = self.create_publisher(Float32, "/turtles_distance", 10)

        self.create_subscription(Pose, "/turtle1/pose", self.cb1, 10)
        self.create_subscription(Pose, "/turtle2/pose", self.cb2, 10)
        self.create_subscription(String, "/moving_turtle", self.cb_moving, 10)

        self.get_logger().info("Distance node started.")

    def cb_moving(self, msg):
        self.moving_turtle = msg.data

    def cb1(self, msg):
        self.pose1 = msg
        self.evaluate()

    def cb2(self, msg):
        self.pose2 = msg
        self.evaluate()

    def stop_robot(self, name):
        pub = self.create_publisher(Twist, f"/{name}/cmd_vel", 10)
        twist = Twist()
        pub.publish(twist)

    def evaluate(self):
        if self.pose1 is None or self.pose2 is None:
            return

        dx = self.pose1.x - self.pose2.x
        dy = self.pose1.y - self.pose2.y
        dist = math.sqrt(dx*dx + dy*dy)

        self.distance_pub.publish(Float32(data=dist))

        if dist < self.threshold and self.moving_turtle in ["turtle1", "turtle2"]:
            self.get_logger().info(f"Too close! Stopping {self.moving_turtle}")
            self.stop_robot(self.moving_turtle)

        for name, pose in [("turtle1", self.pose1), ("turtle2", self.pose2)]:
            if pose.x < self.min_b or pose.x > self.max_b or pose.y < self.min_b or pose.y > self.max_b:
                self.get_logger().info(f"{name} out of bounds. Stopping.")
                self.stop_robot(name)

def main(args=None):
    rclpy.init(args=args)
    node = DistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
