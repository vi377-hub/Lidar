#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math

class LidarMavrosFilter(Node):
    def __init__(self):
        super().__init__('lidar_mavros_filter')
        
        # 1. CẤU HÌNH QoS
        # QoS cho Lidar: Nhận dữ liệu BEST_EFFORT
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS cho MAVROS: Gửi dữ liệu RELIABLE
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 2. GÁN ĐÚNG CHUẨN QoS CHO TỪNG ĐẦU
        # Subscriber nhận dữ liệu từ RPLidar (Dùng lidar_qos)
        self.sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_cb, 
            lidar_qos)
        
        # Publisher gửi dữ liệu sạch sang MAVROS (Dùng mavros_qos)
        self.pub = self.create_publisher(
            LaserScan, 
            '/mavros/obstacle/send', 
            mavros_qos)

        # 3. CẤU HÌNH GIỚI HẠN VẬT LÝ
        self.MIN_VALID_DIST = 0.30  # mét 
        self.MAX_VALID_DIST = 8.0   # mét 
        
        # 4. CẤU HÌNH GÓC MÙ (Blind Spots)
        self.BLIND_SPOTS = [
            (165, -165), 
        ]

        # 5. CẤU HÌNH LỌC NHIỄU (Isolated Point Filter)
        self.NOISE_TOLERANCE = 0.15  # mét 

        self.get_logger().info("Lidar Mavros Filter Node is ONLINE (QoS Split OK)")

    def is_in_blind_spot(self, angle_deg):
        """Kiểm tra góc quét có bị vướng khung máy bay không"""
        for start, end in self.BLIND_SPOTS:
            if start <= end:
                if start <= angle_deg <= end: return True
            else: # Xử lý vắt ngang điểm 180/-180 độ
                if angle_deg >= start or angle_deg <= end: return True
        return False

    def scan_cb(self, msg):
        # Cập nhật timestamp thời gian thực để MAVROS/PX4 không từ chối dữ liệu cũ
        msg.header.stamp = self.get_clock().now().to_msg()
        
        raw_ranges = list(msg.ranges)
        num_points = len(raw_ranges)
        
        # Tạo giá trị an toàn (ngoài tầm bắn của Lidar) để báo cho PX4 vùng đó trống
        safe_value = msg.range_max + 1.0
        clean_ranges = [safe_value] * num_points

        for i in range(num_points):
            r = raw_ranges[i]
            
            # Bước 1: Loại bỏ dữ liệu lỗi (NaN, Inf, <= 0)
            if math.isnan(r) or math.isinf(r) or r <= 0.0:
                continue
                
            # Bước 2: Lọc theo khoảng cách vật lý thực tế
            if r < self.MIN_VALID_DIST or r > self.MAX_VALID_DIST:
                continue

            # Bước 3: Lọc theo góc mù cơ khí
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)
            # Chuẩn hóa về dải [-180, 180]
            angle_deg = (angle_deg + 180) % 360 - 180
            
            if self.is_in_blind_spot(angle_deg):
                continue

            # Bước 4: Lọc điểm nhiễu đơn lẻ (Kiểm tra lân cận trái/phải)
            prev_i = (i - 1) % num_points
            next_i = (i + 1) % num_points
            
            r_prev = raw_ranges[prev_i]
            r_next = raw_ranges[next_i]
            
            has_neighbor = False
            # Kiểm tra điểm bên trái
            if not math.isnan(r_prev) and not math.isinf(r_prev):
                if abs(r - r_prev) < self.NOISE_TOLERANCE:
                    has_neighbor = True
            
            # Kiểm tra điểm bên phải (nếu bên trái chưa khớp)
            if not has_neighbor and not math.isnan(r_next) and not math.isinf(r_next):
                if abs(r - r_next) < self.NOISE_TOLERANCE:
                    has_neighbor = True
            
            # Nếu điểm có sự kết nối với các điểm xung quanh -> Vật thể thật
            if has_neighbor:
                clean_ranges[i] = r 

        # Gán lại mảng dữ liệu đã lọc sạch
        msg.ranges = clean_ranges
        
        # Phát dữ liệu sang MAVROS
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = LidarMavrosFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
