#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class AngleFilterNode(Node):
    def __init__(self):
        super().__init__('angle_filter_node')

        self.angle_min_deg = -90
        self.angle_max_deg = 90
        self.angle_min_rad = math.radians(self.angle_min_deg)
        self.angle_max_rad = math.radians(self.angle_max_deg)

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(
            LaserScan, '/filtered_scan', 10)

    def scan_callback(self, msg: LaserScan):
        new_msg = LaserScan()
        new_msg.header = msg.header
        new_msg.angle_increment = msg.angle_increment
        new_msg.time_increment = msg.time_increment
        new_msg.scan_time = msg.scan_time
        new_msg.range_min = msg.range_min
        new_msg.range_max = msg.range_max

        # Calculate start and end indices
        angle = msg.angle_min
        start_index = 0
        while angle < self.angle_min_rad:
            start_index += 1
            angle += msg.angle_increment

        angle = msg.angle_min
        end_index = 0
        while angle <= self.angle_max_rad:
            end_index += 1
            angle += msg.angle_increment

        new_msg.angle_min = msg.angle_min + start_index * msg.angle_increment
        new_msg.angle_max = msg.angle_min + end_index * msg.angle_increment
        new_msg.ranges = msg.ranges[start_index:end_index]
        if msg.intensities:
            new_msg.intensities = msg.intensities[start_index:end_index]

        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AngleFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
