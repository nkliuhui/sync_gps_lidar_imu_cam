import rosbag
import math
from sensor_msgs.msg import Imu, PointCloud2, NavSatFix
from gps_driver.msg import GPCHC, IMURaw

bag_open = rosbag.Bag('/home/lab118/lidar_data/rtk_lidar/2020-09-21-19-13-06.bag', 'r')
bag_write = rosbag.Bag('/home/lab118/lidar_data/rtk_lidar/2020-09-21-19-13-06_m2.bag', 'w')

for topic, msg, t in bag_open.read_messages():
    if topic == "/GPS/can_data":
        bag_write.write("/GPS/can_data", msg, t)
    elif topic == "/GPS/gps_chc":
        yaw = msg.position_covariance[2]
        roll = msg.position_covariance[1]
        pitch = msg.position_covariance[0]
        yaw = yaw - math.pi/2.0
        while(yaw > math.pi):
            yaw = yaw - math.pi * 2
        while(yaw < -math.pi):
            yaw = yaw + math.pi * 2
        # msg.position_covariance[2] = yaw
        # msg.position_covariance[1] = pitch
        msg.position_covariance = tuple( (roll, pitch, yaw, 0,0,0, 0,0,0) )
        bag_write.write("/GPS/gps_chc", msg, t)
    elif topic == "/huace/gpchc":
        yaw = msg.heading
        pitch = msg.roll
        roll = msg.pitch
        yaw = yaw - math.pi/2.0
        while(yaw > math.pi):
            yaw = yaw - math.pi * 2
        while(yaw < -math.pi):
            yaw = yaw + math.pi * 2
        msg.heading = yaw
        msg.pitch = pitch
        msg.roll = roll
        bag_write.write("/huace/gpchc", msg, t)
    elif topic == "/velodyne_points":
        bag_write.write("/velodyne_points", msg, t)
    elif topic == "/GPS/gps_can":
        bag_write.write("/GPS/gps_can", msg, t)
    elif topic == "/GPS/imu_data_raw":
        bag_write.write("/GPS/imu_data_raw", msg, t)
    elif topic == "/GPS/imu_data":
        bag_write.write("/GPS/imu_data", msg, t)

bag_open.close()
bag_write.close()