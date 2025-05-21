#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time


class AebController:
    def __init__(self):
        # 初始化ROS节点和订阅者/发布者
        self.scan_subscriber = rospy.Subscriber('/scan_raw', LaserScan, self.register_scan)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # 初始化速度消息
        self.velocity = Twist()
        self.velocity.linear.y = 0
        self.velocity.linear.z = 0
        self.velocity.angular.x = 0
        self.velocity.angular.y = 0
        self.velocity.angular.z = 0

        # 前进速度和安全距离配置
        self.forward_speed = 1.0  # 前进速度
        self.stop_distance = 0.3  # 安全距离

        # 初始化卡尔曼滤波器数组
        self.kalman_filters = [KalmanFilter() for _ in range(56)]  # 有56个扇区

    def register_scan(self, scan_data):
        # 获取激光雷达扫描数据
        ranges = np.array(scan_data.ranges)
        angle_min = scan_data.angle_min
        angle_increment = scan_data.angle_increment

        # 定义前方的角度范围
        front_angle_min = 0.0 * (np.pi / 180.0)
        front_angle_max = 10.0 * (np.pi / 180.0)

        start_index = int((front_angle_min - angle_min) / angle_increment)
        end_index = int((front_angle_max - angle_min) / angle_increment)

        # 确保索引在合法范围内
        start_index = max(0, start_index)
        end_index = min(len(ranges) - 1, end_index)

        # 提取前方的距离数据
        filtered_ranges = ranges[start_index:end_index + 1]
        #nan_mask = np.isnan(filtered_ranges)
        #filtered_ranges_without_nan = filtered_ranges[~nan_mask]

        # 对每个扇区的距离进行卡尔曼滤波
        smoothed_distances = []
        for i, distance in enumerate(filtered_ranges):
            if np.isfinite(distance):
                smoothed_distance = self.kalman_filters[i % 56].update(distance)
                smoothed_distances.append(smoothed_distance)

        # 找到前方的最近距离
        if smoothed_distances:
            min_distance = np.min(smoothed_distances)
        # 如果smoothed_distance数组为空，则将最短距离设置为无穷大
        else:
            min_distance = np.inf
        
        # 根据最近距离发布速度指令
        if np.isfinite(min_distance):
            rospy.logwarn('Closest obstacle distance: %f', min_distance)
            if min_distance < self.stop_distance:  # 如果障碍物距离小于安全距离，停止机器人
                self.velocity.linear.x = 0
            else:
                self.velocity.linear.x = self.forward_speed  # 否则继续前进
        else:
            rospy.logwarn('No valid obstacle found in the specified range.')
            self.velocity.linear.x = self.forward_speed

        self.cmd_vel_publisher.publish(self.velocity)

# 卡尔曼滤波器
class KalmanFilter:
    def __init__(self):
        # 初始化卡尔曼滤波器的参数
        self.x = None  # 初始状态
        self.P = 1.0  # 初始估计误差协方差
        self.Q = 0.0001  # 过程噪声协方差
        self.R = 0.0009  # 测量噪声协方差
        self.K = 0.0  # 卡尔曼增益
        self.last_time = 0.0  # 上一次执行的时间
        self.current_time = 0.0  # 这一次执行的时间
        self.interval_time = 0.0  # 两次执行的时间间隔

    def update(self, measurement):
        # 初始数据直接返回
        if self.x is None:
            self.x = measurement
            self.last_time = time.perf_counter()
            return self.x

        # 预测此时小车到最近障碍物的距离
        self.P = self.P + self.Q
        self.x = self.x - 1.0 * self.interval_time

        # 更新
        self.K = self.P / (self.P + self.R)
        self.x = self.x + self.K * (measurement - self.x)
        self.P = (1 - self.K) * self.P

        # 更新时间
        self.current_time = time.perf_counter()
        self.interval_time = self.current_time - self.last_time
        self.last_time = self.current_time

        return self.x


if __name__ == '__main__':
    rospy.init_node('aeb_controller')
    controller = AebController()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


