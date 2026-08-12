import math
import statistics
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan


# PX4 Collision Prevention dung ban do 72 sector, moi sector 5 do.
SECTOR_COUNT = 72
SECTOR_WIDTH_DEG = 360.0 / SECTOR_COUNT
ANGLE_OFFSET_DEG = 0.0
MIN_VALID_DIST = 0.35
MAX_OUTPUT_DIST = 6.0
TEMPORAL_WINDOW = 3


class LidarMavrosFilter(Node):
    def __init__(self):
        super().__init__('lidar_mavros_filter')

        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, lidar_qos)
        self.pub = self.create_publisher(
            LaserScan, '/mavros/obstacle/send', mavros_qos)

        self.sector_history = [
            deque(maxlen=TEMPORAL_WINDOW) for _ in range(SECTOR_COUNT)
        ]
        self.last_output_max = None
        self.last_log_ns = 0

        self.get_logger().info(
            'Lidar Filter ONLINE: 72 sectors, BODY_FRD, median 3 scans'
        )
        self.get_logger().warn(
            'MAVROS obstacle mav_frame phai duoc dat thanh BODY_FRD'
        )

    @staticmethod
    def _copy_header(source, target):
        target.header.stamp = source.header.stamp
        target.header.frame_id = 'base_link'

    def _reset_history_if_range_changed(self, output_max):
        if (
            self.last_output_max is not None
            and not math.isclose(output_max, self.last_output_max, abs_tol=0.01)
        ):
            for history in self.sector_history:
                history.clear()
        self.last_output_max = output_max

    def _temporal_filter(self, sector_index, raw_distance):
        history = self.sector_history[sector_index]
        history.append(raw_distance)

        if len(history) < TEMPORAL_WINDOW:
            return raw_distance

        return float(statistics.median(history))

    def scan_cb(self, msg):
        if not msg.ranges or msg.angle_increment == 0.0:
            self.get_logger().warn('Bo qua /scan rong hoac angle_increment = 0')
            return

        sensor_max = msg.range_max
        if not math.isfinite(sensor_max) or sensor_max <= 0.0:
            sensor_max = MAX_OUTPUT_DIST
        output_max = min(float(sensor_max), MAX_OUTPUT_DIST)
        clear_distance = output_max + 0.01
        self._reset_history_if_range_changed(output_max)

        # Moi phan tu chua cac khoang cach hop le trong mot sector BODY_FRD.
        sector_samples = [[] for _ in range(SECTOR_COUNT)]

        for index, distance in enumerate(msg.ranges):
            ros_angle = msg.angle_min + index * msg.angle_increment

            # MAVLink BODY_FRD: goc duong theo chieu kim dong ho = PHAI.
            body_angle_deg = (
                -math.degrees(ros_angle) + ANGLE_OFFSET_DEG
            ) % 360.0
            sector_index = int(body_angle_deg // SECTOR_WIDTH_DEG) % SECTOR_COUNT

            if not math.isfinite(distance):
                continue

            distance = float(distance)

            # Giu vung mu 0.35 m theo yeu cau. Diem xa hon tam xu ly cung
            # khong phai vat can doi voi ban do 6 m.
            if distance <= MIN_VALID_DIST or distance > output_max:
                continue

            sector_samples[sector_index].append(distance)

        filtered_ranges = []
        for sector_index, samples in enumerate(sector_samples):
            # Sau khi da chia dung 5 do, lay diem gan nhat de khong bo sot
            # vat can nho. Median theo thoi gian se loai diem nhieu mot-frame.
            raw_distance = min(samples) if samples else clear_distance
            filtered_ranges.append(
                self._temporal_filter(sector_index, raw_distance)
            )

        output = LaserScan()
        self._copy_header(msg, output)
        output.angle_min = 0.0
        output.angle_increment = math.radians(SECTOR_WIDTH_DEG)
        output.angle_max = output.angle_min + (
            SECTOR_COUNT - 1
        ) * output.angle_increment
        output.scan_time = msg.scan_time
        output.time_increment = (
            msg.scan_time / SECTOR_COUNT if msg.scan_time > 0.0 else 0.0
        )
        output.range_min = max(float(msg.range_min), 0.0)
        output.range_max = output_max
        output.ranges = filtered_ranges
        output.intensities = []

        self.pub.publish(output)

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns >= 1_000_000_000:
            obstacle_sectors = sum(
                distance <= output_max for distance in filtered_ranges
            )
            self.get_logger().info(
                f'Published 72 sectors: obstacle={obstacle_sectors}, '
                f'clear={SECTOR_COUNT - obstacle_sectors}, max={output_max:.1f}m'
            )
            self.last_log_ns = now_ns


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
