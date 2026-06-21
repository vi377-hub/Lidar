#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math

class LidarMavrosFilter(Node):
    def __init__(self):
        super().__init__('lidar_mavros_filter')

        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, lidar_qos)

        self.pub = self.create_publisher(
            LaserScan, '/mavros/obstacle/send', mavros_qos)

        # Chỉnh MIN_VALID_DIST = khoảng cách càng đáp + buffer 5cm
        self.MIN_VALID_DIST = 0.5    # mét — đo thực tế rồi chỉnh
        self.MAX_VALID_DIST = 8.0    # mét
        self.NOISE_TOLERANCE = 0.15  # mét

        self.get_logger().info("Lidar Filter ONLINE → /mavros/obstacle/send")

    def is_valid_point(self, r):
        if math.isnan(r) or math.isinf(r) or r <= 0.0:
            return False
        return self.MIN_VALID_DIST <= r <= self.MAX_VALID_DIST

    def scan_cb(self, msg):
        raw_ranges = list(msg.ranges)
        num_points = len(raw_ranges)

        clean_ranges = [float(msg.range_max)] * num_points

        for i in range(num_points):
            r = raw_ranges[i]

            if not self.is_valid_point(r):
                continue

            prev_i = (i - 1) % num_points
            next_i = (i + 1) % num_points

            has_neighbor = (
                (self.is_valid_point(raw_ranges[prev_i]) and
                 abs(r - raw_ranges[prev_i]) < self.NOISE_TOLERANCE)
                or
                (self.is_valid_point(raw_ranges[next_i]) and
                 abs(r - raw_ranges[next_i]) < self.NOISE_TOLERANCE)
            )

            if has_neighbor:
                clean_ranges[i] = r

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.ranges = clean_ranges
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = LidarMavrosFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
