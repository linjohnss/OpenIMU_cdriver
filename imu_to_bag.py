import pandas as pd
df = pd.read_csv('output.csv')

import rospy
import rosbag
from sensor_msgs.msg import Imu, NavSatFix

with rosbag.Bag('output.bag', 'w') as bag:
    for row in range(df.shape[0]):
        t = df['gps_timeofweek'][row] + df['gps_week'][row]*604800
        timestamp = rospy.Time.from_sec(t)
        imu_msg = Imu()
        imu_msg.header.stamp = timestamp

        # Populate the data elements for IMU
        # e.g. imu_msg.angular_velocity.x = df['a_v_x'][row]
        imu_msg.linear_acceleration.x = df['x_acc'][row]
        imu_msg.linear_acceleration.y = df['y_acc'][row]
        imu_msg.linear_acceleration.z = df['z_acc'][row]
        imu_msg.angular_velocity.x = df['x_gyro'][row]
        imu_msg.angular_velocity.y = df['y_gyro'][row]
        imu_msg.angular_velocity.z = df['z_gyro'][row]
        bag.write("/imu", imu_msg, timestamp)

        # gps_msg = NavSatFix()
        # gps_msg.header.stamp = timestamp

        # # Populate the data elements for GPS

        # bag.write("/gps", gpu_msg, timestamp)