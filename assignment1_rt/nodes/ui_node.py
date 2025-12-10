import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')
        self.moving_pub = self.create_publisher(String, '/moving_turtle', 10)
        self.get_logger().info("UI node started")

    def send_velocity(self, turtle, linear, angular):
        topic = f"/{turtle}/cmd_vel"
        pub = self.create_publisher(Twist, topic, 10)

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        self.get_logger().info(f"Sending vel to {turtle} ...")

        t0 = time.time()
        while time.time() - t0 < 1.0:   # لمدة ثانية واحدة
            pub.publish(twist)
            time.sleep(0.1)

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

    def run_console(self):
        while rclpy.ok():
            turtle = input("Choose turtle (turtle1 or turtle2): ").strip()
            if turtle not in ["turtle1", "turtle2"]:
                print("Invalid. Try again.")
                continue

            lin = float(input("Linear speed: "))
            ang = float(input("Angular speed: "))

            self.moving_pub.publish(String(data=turtle))
            self.send_velocity(turtle, lin, ang)
            self.moving_pub.publish(String(data=""))

def main(args=None):
    rclpy.init(args=args)
    node = UINode()
    try:
        node.run_console()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
