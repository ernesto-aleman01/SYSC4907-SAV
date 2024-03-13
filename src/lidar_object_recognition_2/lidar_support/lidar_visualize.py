import rospy
import cv2 as cv

import math

from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float64MultiArray
from common.bridge import get_bridge
from pathlib import Path
from aabb import AABB
from point import Point

from Models import point_to_gui_coords

q_size = 20
RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)

FONT = 0
FONT_SCALE = 0.8

NH = 0
CITY = 1
class LidarVisualizer:

    def __init__(self):

        self.bridge = get_bridge()
        self.map_path =  Path(__file__).parents[1] / 'NH_Top.png'
        self.image = cv.imread(str(self.map_path))
    def listener(self):
        rospy.init_node("new_lidar_visualizer", anonymous=True)
        rospy.Subscriber("new_lidar_data", Float64MultiArray, self.lidar_handler)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    def lidar_handler(self, lidar_data: PointCloud):
        image = cv.imread(str(self.map_path))
        # These are indexes into a bounding box for each of the points that it is composed of
        min_x = 0
        min_y = 1
        min_z = 2
        max_x = 3
        max_y = 4
        max_z = 5
        point_per_aabb = 6
        length = len(lidar_data.data)
        num_aabb = int(length/ point_per_aabb)

        # Find the closest point to the car
        all_aabbs = []

        for x in range(0, num_aabb):
            i = x * point_per_aabb
            min_point = Point(lidar_data.data[i + min_x], lidar_data.data[i + min_y], lidar_data.data[i + min_z])
            max_point = Point(lidar_data.data[i + max_x], lidar_data.data[i + max_y], lidar_data.data[i + max_z])
            aabb = AABB(min_point, max_point)
            all_aabbs.append(aabb)

        # These values can be changed to avoid obstacles that are farther from the car
        # Point(x,y,z)
        car_aabb = AABB(
            min_point=Point(0.0, -1.0, 0.0),
            max_point=Point(10.0, 1.0, 2.0)
        )
        # car_pos = Point(lidar_data.data[length-3], lidar_data.data[length-2], lidar_data.data[length-1])
        # car_x_float, car_y_float = point_to_gui_coords((car_pos.x, car_pos.y), 0)
        car_pos = self.bridge.get_position()
        car_or = self.bridge.get_orientation()
        car_x_float, car_y_float = point_to_gui_coords((car_pos.x_val.real, car_pos.y_val.real), 0)
        car_x, car_y = math.floor(car_x_float), math.floor(car_y_float)
        # Find the point of the closest AABB for the car to avoid, if any. Only if an AABB intersects
        # with the car's AABB is an obstacle considering for avoidance
        lidar_dist, lidar_pos = all_aabbs[0].vector_to_closest_point()
        lidar_y, lidar_x = math.floor(lidar_pos[0]) , math.floor(lidar_pos[1])
        lidar_y1, lidar_x1 = math.floor(all_aabbs[0].x_range.min), math.floor(all_aabbs[0].y_range.min)
        lidar_y2, lidar_x2 = math.floor(all_aabbs[0].x_range.max), math.floor(all_aabbs[0].y_range.max)

        cv.rectangle(image, (car_x + 5, car_y + 10), (car_x - 5, car_y - 10), RED, 2)
        cv.rectangle(image, (car_x - lidar_x1, car_y - lidar_y1), (car_x - lidar_x2, car_y - lidar_y2), GREEN, 2)
        cv.arrowedLine(image, (car_x, car_y), ((car_x - lidar_x), (car_y - lidar_y)), BLUE, 2)
        cv.putText(image, f'{car_or}', (100, 100), FONT, FONT_SCALE, GREEN)
        cv.imshow("TESTING", image)
        cv.waitKey(1)



if __name__ == "__main__":
    lidar_vis = LidarVisualizer()
    lidar_vis.listener()
