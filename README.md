# LiDAR Obstacle Detection for PX4

Dự án ROS 2 dùng RPLIDAR A1 để quan sát môi trường xung quanh drone, hiển thị vật cản và cung cấp dữ liệu cho PX4 Collision Prevention thông qua MAVROS.

## Chức năng

- Nhận dữ liệu `sensor_msgs/LaserScan` từ topic `/scan`.
- Hiển thị radar 360° bằng Pygame, phạm vi xử lý tối đa 6 m.
- Phân cụm các điểm gần nhau và cảnh báo hướng `TRUOC`, `SAU`, `TRAI`, `PHAI`.
- Phát âm thanh cảnh báo khi vật cản nằm trong phạm vi 2 m.
- Chuyển dữ liệu LiDAR thành 72 sector, mỗi sector 5° theo hệ trục `BODY_FRD`.
- Lấy khoảng cách gần nhất trong từng sector và lọc median qua ba vòng quét để giảm nhiễu tức thời.
- Publish dữ liệu đã lọc tới `/mavros/obstacle/send` cho PX4.

## File chính

- `Warning.py`: giao diện radar, phân cụm và cảnh báo vật cản.
- `lidar_filter.py`: lọc dữ liệu `/scan` và chuyển sang định dạng sector cho MAVROS/PX4.


