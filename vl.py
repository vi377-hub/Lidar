#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
import pygame
import math
import threading
import os
import struct
import wave


WINDOW_SIZE = (800, 800)

MIN_VALID_DIST = 1.0    
MAX_VALID_DIST = 8.0    
WARNING_DIST   = 2.0    

SCALE = WINDOW_SIZE[0] / (2 * MAX_VALID_DIST)  # pixel/mét

CLUSTER_THRESHOLD = 0.25  # mét — 2 điểm cách nhau dưới 25cm → cùng cụm
NOISE_TOLERANCE   = 0.15  
BEEP_FILENAME = 'warning_beep.wav'
BEEP_INTERVAL = 0.4  # giây

def create_beep_sound(filename):
    if not os.path.exists(filename):
        with wave.open(filename, 'w') as f:
            f.setnchannels(1); f.setsampwidth(2); f.setframerate(44100)
            duration = 0.15; volume = 32767.0 * 0.5
            data = b""
            for i in range(int(44100 * duration)):
                value = int(volume * math.sin(2 * math.pi * 1000 * i / 44100))
                data += struct.pack('<h', value)
            f.writeframes(data)

class LidarVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_visualizer_node')
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_policy)
        self.latest_scan = []

    def scan_callback(self, msg):
        temp_data = []
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r) or r <= 0.0:
                continue
            if r < MIN_VALID_DIST or r > MAX_VALID_DIST:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            temp_data.append((angle, r))  
        self.latest_scan = temp_data

def get_euclidean_dist(p1, p2):
    """Tính khoảng cách Euclidean giữa 2 điểm polar (angle, dist_m)"""
    x1 = p1[1] * math.cos(p1[0]); y1 = p1[1] * math.sin(p1[0])
    x2 = p2[1] * math.cos(p2[0]); y2 = p2[1] * math.sin(p2[0])
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

def main():
    rclpy.init()
    lidar_node = LidarVisualizer()
    threading.Thread(target=rclpy.spin, args=(lidar_node,), daemon=True).start()

    pygame.init()
    lcd = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption('Lidar Monitor')
    font = pygame.font.SysFont('Arial', 18, bold=True)

    create_beep_sound(BEEP_FILENAME)
    try: warning_sound = pygame.mixer.Sound(BEEP_FILENAME)
    except: warning_sound = None

    clock = pygame.time.Clock()
    last_beep_time = 0
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False

        lcd.fill((15, 15, 15))
        center = (WINDOW_SIZE[0]//2, WINDOW_SIZE[1]//2)

        # Vẽ radar grid (đơn vị mét, mỗi vòng 1m)
        for r_m in range(1, int(MAX_VALID_DIST) + 1):
            pygame.draw.circle(lcd, (40, 40, 40), center, int(r_m * SCALE), 1)
            label = font.render(f"{r_m}m", True, (60, 60, 60))
            lcd.blit(label, (center[0] + int(r_m * SCALE) + 2, center[1]))

        # Vòng cảnh báo màu đỏ
        pygame.draw.circle(lcd, (120, 0, 0), center, int(WARNING_DIST * SCALE), 2)

        # Vòng mù (MIN_VALID_DIST)
        pygame.draw.circle(lcd, (60, 60, 60), center, int(MIN_VALID_DIST * SCALE))

        scan_points = list(lidar_node.latest_scan)

        # Phân loại điểm theo ngưỡng cảnh báo
        warning_points = [p for p in scan_points if p[1] < WARNING_DIST]
        safe_points    = [p for p in scan_points if p[1] >= WARNING_DIST]

        # Vẽ điểm an toàn (xanh lá)
        for theta, dist in safe_points:
            px = int(center[0] - dist * math.sin(theta) * SCALE)
            py = int(center[1] - dist * math.cos(theta) * SCALE)
            lcd.set_at((px, py), (0, 100, 0))

        # Phân cụm vật thể nguy hiểm
        clusters = []
        if warning_points:
            warning_points.sort(key=lambda x: x[0])
            current_cluster = [warning_points[0]]
            for i in range(1, len(warning_points)):
                if get_euclidean_dist(warning_points[i], warning_points[i-1]) < CLUSTER_THRESHOLD:
                    current_cluster.append(warning_points[i])
                else:
                    clusters.append(current_cluster)
                    current_cluster = [warning_points[i]]
            clusters.append(current_cluster)

        # Vẽ cụm và nhãn
        has_danger = len(clusters) > 0
        for cluster in clusters:
            mid_p = cluster[len(cluster)//2]
            theta_mid, dist_mid = mid_p

            target_x = int(center[0] - dist_mid * math.sin(theta_mid) * SCALE)
            target_y = int(center[1] - dist_mid * math.cos(theta_mid) * SCALE)

            # Xác định hướng
            angle_deg = math.degrees(theta_mid)
            angle_deg = (angle_deg + 180) % 360 - 180  # chuẩn hóa [-180, 180]
            if   -45  <= angle_deg <  45:  direction = "TRUOC"
            elif  45  <= angle_deg <  135: direction = "TRAI"
            elif -135 <= angle_deg < -45:  direction = "PHAI"
            else:                          direction = "SAU"

            # Vẽ điểm cụm (đỏ)
            for theta, dist in cluster:
                px = int(center[0] - dist * math.sin(theta) * SCALE)
                py = int(center[1] - dist * math.cos(theta) * SCALE)
                pygame.draw.circle(lcd, (255, 0, 0), (px, py), 3)

            # Đường chỉ hướng và nhãn (đơn vị mét)
            pygame.draw.line(lcd, (255, 255, 0), center, (target_x, target_y), 1)
            label_text = f"{direction} | {dist_mid:.2f}m"
            label_surf = font.render(label_text, True, (255, 255, 0))
            lcd.blit(label_surf, (target_x + 12, target_y - 12))

        if has_danger:
            pygame.draw.rect(lcd, (255, 0, 0), (0, 0, WINDOW_SIZE[0], WINDOW_SIZE[1]), 10)
            current_time = pygame.time.get_ticks()
            if (current_time - last_beep_time) > (BEEP_INTERVAL * 1000):
                if warning_sound: warning_sound.play()
                last_beep_time = current_time

        pygame.display.flip()
        clock.tick(30)

    lidar_node.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == '__main__':
    main()
