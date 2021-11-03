#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, String
from sensor_msgs.msgs import PointCloud

class Lidar:

    """
    Lidar data
    """

    def __init__(self):
        self.lidar_pub = rospy.Publisher('lidar',Float64 ,queue_size=10)

    def send_lidar_data(self):
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.lidar_pub(PointCloud())

if __name__ == '__main__':
    lidar = Lidar()
    lidar.send_lidar_data()