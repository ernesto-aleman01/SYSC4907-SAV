#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

# ROS Image message
from sensor_msgs.msg import Image, CompressedImage
from sensors.msg import SceneDepth

from common.bridge import get_bridge, SCENE_IMAGE, DEPTH_IMAGE


class FastCamera:
    """
    Lidar data
    """
    def __init__(self):
        self.bridge = get_bridge()
        self.publisher = rospy.Publisher('fast_cam', Image, queue_size=1)
    def get_scene_image(self):
        resp = self.bridge.get_image(0, SCENE_IMAGE)
        img1d = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
        # reshape array to 3 channel image array
        return img1d.reshape(resp.height, resp.width, 3)

    def listen(self):
        rospy.init_node("fast_camera", anonymous=True)
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            depth_img = self.bridge.get_image(0, 1, pixels_as_float=True)
            scene_img = self.bridge.get_image(0, SCENE_IMAGE)
            scene_img_rgb_string = scene_img.image_data_uint8
            msg = Image()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "frameId"
            msg.encoding = "rgb8"
            msg.height = scene_img.height  # 360  # resolution should match values in settings.json
            msg.width = scene_img.width  # 640
            msg.data = scene_img_rgb_string
            msg.is_bigendian = 0
            msg.step = msg.width * 3

            self.publisher.publish(msg)

            img1d = np.frombuffer(scene_img.image_data_uint8, dtype=np.uint8)
            img3d = img1d.reshape(scene_img.height, scene_img.width, 3)
            cv2.imshow("FAST_CAM", img3d)
            cv2.waitKey(1)
            rate.sleep()


if __name__ == '__main__':
    fast_cam = FastCamera()
    fast_cam.listen()
