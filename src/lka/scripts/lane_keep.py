#!/usr/bin/env python
import cv2 as cv
import numpy as np

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from lka.msg import Lane
from lka.msg import Lanes
from lka.msg import Margins
from cv_bridge import CvBridge, CvBridgeError

import lane_detect

class LaneKeepAssist:

    def __init__(self, metrics):
        self.steeringPub = rospy.Publisher('lka/steering', Float64, queue_size=1)
        self.lanePub = rospy.Publisher('lka/lanes', Lanes, queue_size=1)
        self.marginsPub = rospy.Publisher('lka/margins', Margins, queue_size=1)

        self.steering = 0.0
        self.metrics = metrics

    def margin_correction(self, left_margin, right_margin, height, width):
        # If metrics are enabled, then send margins to metrics node for analysis
        if self.metrics:
            margins = Margins()
            margins.margin_diff = right_margin - left_margin
            self.marginsPub.publish(margins)

        return (right_margin - left_margin)/(width/2)

    def construct_lane_msg(self, lane, values):
        lane.exists = True
        rospy.loginfo("lane values:{}".format(values))
        lane.slope = values[0]
        lane.y_cept = values[1]

    def processImage(self, img):
        bridge  = CvBridge()
        height = img.height
        width  = img.width
        margin_steer = 0.0
        x_intersection = None
        # Weights
        x_to_margin = 0.6
        current_to_prev = 0.9
        

        if img != None:
            try:
                cv_img = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")

                result = lane_detect.lane_detect(cv_img, (height, width))
                if result != None:
                    left_line, right_line = result
                    rospy.loginfo(result)

                    lanes_msg = Lanes() # Left lane is first element and right lane is second.

                    if len(left_line) != 0 and len(right_line) != 0: # There are both left and right lane lines
                        left_margin  = (width/2) - ((height - left_line[1])/left_line[0])
                        right_margin = ((height - right_line[1])/right_line[0]) - (width/2)
                        
                        x_intersection = (left_line[1] - right_line[1])/(right_line[0] - left_line[0])
                        margin_steer = self.margin_correction(left_margin, right_margin, height, width)

                        self.construct_lane_msg(lanes_msg.lane_lines[0], left_line)
                        self.construct_lane_msg(lanes_msg.lane_lines[1], right_line)

                    elif len(left_line) == 0 and len(right_line) != 0: # Probably too close to the right side of the lane
                        right_margin = ((height - right_line[1])/right_line[0]) - (width/2)
                        left_margin  = (width) - right_margin

                        x_intersection = ((height/2) - right_line[1])/(right_line[0] - 0)
                        margin_steer = self.margin_correction(left_margin, right_margin, height, width)

                        self.construct_lane_msg(lanes_msg.lane_lines[1], right_line)

                    elif len(right_line) == 0 and len(left_line) != 0: # Probably too close to the left side of the lane
                        left_margin  = (width/2) - ((height - left_line[1])/left_line[0])
                        right_margin = (width) - left_margin

                        x_intersection = (left_line[1] - (height/2))/(0 - left_line[0])
                        margin_steer = self.margin_correction(left_margin, right_margin, height, width)

                        self.construct_lane_msg(lanes_msg.lane_lines[0], left_line)

                    # Calculate x-intersection and steering correction to center x-intersection steering correction
                    if x_intersection != None:
                        x_steering = (x_intersection / (width / 2)) - 1
                    else:
                        x_steering = self.steering

                    # Use margin steering correction or just use x-intersection or previous steering value
                    if (margin_steer != 0):
                        x_steering = (x_to_margin * x_steering) + ((1-x_to_margin) * margin_steer)
                    
                    self.steering = (current_to_prev * x_steering) + ((1-current_to_prev) * self.steering)
                    rospy.loginfo('Steering Value')
                    rospy.loginfo(self.steering)

                    self.steeringPub.publish(self.steering)
                    self.lanePub.publish(lanes_msg)
                else: # An error
                    rospy.loginfo('Unable to detect lines...')
                    

            except CvBridgeError as e:
                rospy.loginfo(e)

def listener():
    rospy.init_node('lka', anonymous=True)
    lka = LaneKeepAssist(True)
    rospy.Subscriber('fast_cam', Image, lka.processImage)
    rospy.spin()

if __name__ == "__main__":
    listener()