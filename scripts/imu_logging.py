#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import csv
from datetime import datetime

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data', 
            self.imu_callback,
            rclpy.qos.qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        today = datetime.now().strftime('%y%m%d_%H_%M')
        self.csv_file = open(f'imu_data_{today}.csv', mode='w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'roll', 'pitch', 'yaw', 'a_x', 'a_y', 'a_z', 'w_x', 'w_y', 'w_z'])

    def imu_callback(self, msg):
        timestamp = datetime.fromtimestamp(msg.header.stamp.sec).strftime('%X')
        roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z  = self.extract_rpy(msg)
        roll_deg = round(roll*180/np.pi, 2)
        pitch_deg = round(pitch*180/np.pi, 2)
        yaw_deg = round(yaw*180/np.pi, 2)
        self.csv_writer.writerow([timestamp, roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z])
        self.get_logger().info(f"{timestamp} roll: {roll_deg:.2f}, pitch: {pitch_deg:.2f}, yaw: {yaw_deg:.2f}")

    def extract_rpy(self, imu_msg):
        orientation = imu_msg.orientation
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        w_x = imu_msg.angular_velocity.x
        w_y = imu_msg.angular_velocity.y
        w_z = imu_msg.angular_velocity.z
        a_x = imu_msg.linear_acceleration.x
        a_y = imu_msg.linear_acceleration.y
        a_z = imu_msg.linear_acceleration.z

        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
        pitch = np.arcsin(2*(w*y - z*x))
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))

        return roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = ImuSubscriber()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
