import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math

class WaypointNavigation(Node):
    def __init__(self):
        super().__init__('waypoint_navigation')
        self.declare_parameter('waypoint_1_x', 2.0)
        self.declare_parameter('waypoint_1_y', 1.0)
        self.declare_parameter('waypoint_2_x', 4.0)
        self.declare_parameter('waypoint_2_y', 3.0)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)

        self.waypoint_1 = (self.get_parameter('waypoint_1_x').get_parameter_value().double_value,
                            self.get_parameter('waypoint_1_y').get_parameter_value().double_value)
        self.waypoint_2 = (self.get_parameter('waypoint_2_x').get_parameter_value().double_value,
                            self.get_parameter('waypoint_2_y').get_parameter_value().double_value)
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value

        self.current_position = None
        self.previous_error = 0.0
        self.integral = 0.0

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.target_waypoint = self.waypoint_1  # Start with waypoint 1
        self.waypoint_reached = False

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        self.navigate()

    def navigate(self):
        if self.current_position is None:
            return

        # Calculate distance to the current waypoint
        target_x, target_y = self.target_waypoint
        dx = target_x - self.current_position.x
        dy = target_y - self.current_position.y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        # If within 0.1 meters of the waypoint, switch to the next one
        if distance < 0.1:
            if self.target_waypoint == self.waypoint_1:
                self.target_waypoint = self.waypoint_2
            else:
                self.get_logger().info("Navigation complete.")
                return

        # PID Control
        error = math.atan2(dy, dx)
        self.integral += error
        derivative = error - self.previous_error

        # Control for angular velocity
        angular_velocity = self.kp * error + self.ki * self.integral + self.kd * derivative
        linear_velocity = 0.5  # Constant speed

        # Publish the velocity command
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        cmd_vel_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(cmd_vel_msg)

        self.previous_error = error

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigation()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
