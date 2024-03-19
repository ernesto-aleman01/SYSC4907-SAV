#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from common.bridge import get_bridge
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


FLOATS_PER_POINT = 3


class Lidar:
    """
    Lidar data
    """

    def __init__(self):
        self.lidar_pub = rospy.Publisher('lidar', PointCloud, queue_size=10)



    def send_lidar_data(self):
        rospy.init_node('talker', anonymous=True)
        sim = get_bridge()

        rate = rospy.Rate(30)  # 30hz
        while not rospy.is_shutdown():

            point_cloud = PointCloud()
            point_cloud.points = []

            lidar_data = sim.get_lidar_point_cloud()
            lidarParam = sim.get_lidar_data()
            num_points = len(lidar_data) // FLOATS_PER_POINT

            # Convert format of points used by lidar to format of point used by PointCloud message
            for x in range(0, num_points):
                offset = x * FLOATS_PER_POINT
                point = Point32(lidar_data[offset], lidar_data[offset + 1], lidar_data[offset + 2])
                point_cloud.points.append(point)

            lidar_position = [lidarParam.pose.position.x_val, lidarParam.pose.position.y_val, lidarParam.pose.position.z_val]
            lidar_orientation = [lidarParam.pose.orientation.x_val, lidarParam.pose.orientation.y_val,
                                 lidarParam.pose.orientation.z_val, lidarParam.pose.orientation.w_val]
            tf_broadcaster = tf2_ros.TransformBroadcaster()

            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "base_link"  # Parent frame ID
            transform.child_frame_id = "lidar_link"  # Child frame ID
            transform.transform.translation.x = lidar_position[0]  # Translation x
            transform.transform.translation.y = lidar_position[1] # Translation y
            transform.transform.translation.z = lidar_position[2] # Translation z
            transform.transform.rotation.x = lidar_orientation[0]  # Quaternion x
            transform.transform.rotation.y = lidar_orientation[1]  # Quaternion y
            transform.transform.rotation.z = lidar_orientation[2]  # Quaternion z
            transform.transform.rotation.w = lidar_orientation[3]  # Quaternion w

            tf_broadcaster.sendTransform(transform)


            self.lidar_pub.publish(point_cloud)
            rate.sleep()


if __name__ == '__main__':
    lidar = Lidar()
    lidar.send_lidar_data()
