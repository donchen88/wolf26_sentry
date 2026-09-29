# 导航数据消息
# 由下位机（电控）上报，供导航使用
# 包含轮速和IMU数据
std_msgs/Header header

float32[4] wheel_speeds    # 轮速 [1號, 2號, 3號, 4號] 单位：m/s
float32[2] imu_angles      # IMU角度 [pitch, roll] 单位：度
