#!/usr/bin/env python3

import rospy
import numpy as np
import cv2 as cv
import time

from sign_car_recognition.msg import DetectionResult, DetectionResults
from std_msgs.msg import Float64, Float64MultiArray
from lane_keep_assist.msg import LaneStatus, LaneLine
from lka.msg import Lanes, Lane
from common.bridge import CarControls, get_bridge, SCENE_IMAGE
from lane_bound_status import LaneBoundStatus
from mapping_navigation.msg import PathData
from road_segment_type import RoadSegmentType
from road_warning import RoadWarning
from aabb import AABB
from point import Point

from enum import Enum
import math
from typing import List, Set, Tuple, Any, Dict


# What state is the car in?
class CCState(Enum):
    NORMAL = 1
    OBJECT_AVOID = 2
    STREET_RULE = 3


class StreetRuleAction(Enum):
    STOP_SIGN = 1
    SPEED_LIMIT = 2  # Unused


class AvoidanceState(Enum):
    """Unused"""
    DANGER = 1
    WARNING = 2
    NORMAL = 3


class AvoidDirection(Enum):
    """
    If in object avoidance mode, approx. where is the object?
    TODO: Super dumb/basic implementation that is meant to be replaced later
    Must handle multiple objects
    """
    LEFT = 1
    FRONT = 2
    RIGHT = 3


class DetectionRegion(Enum):
    """The values that map to these will likely be heuristic"""
    FAR_LEFT = 1
    LEFT = 2
    CENTER = 3
    RIGHT = 4
    FAR_RIGHT = 5


class CarAction(Enum):
    STOP = 1


class SensorSystem(Enum):
    LIDAR = 1
    CAMERA_DETECTION = 2
    LANE_DETECTION = 3
    NAVIGATION = 4


class StopState(Enum):
    NONE = 0
    DETECTED = 1
    STOPPING = 2
    RESUMING = 3


ROADWARNINGSPEEDS = {RoadWarning.TURN_AHEAD: 2.5, RoadWarning.INTERSECTION_AHEAD: 2.5,
                     RoadWarning.STRAIGHT_ROAD_AHEAD: 5.0, RoadWarning.END_OF_PATH: 0}
ROADSEGMENTSPEEDS = {RoadSegmentType.STRAIGHT: 5.0, RoadSegmentType.INTERSECTION: 2.5, RoadSegmentType.TURN: 2.5}

INITIAL_COOLDOWN = 10
INITIAL_SPEED = 0

IMAGE_WIDTH = 640

FONT = 0
FONT_SCALE = 0.3
REC_THICKNESS = 2
LINE_THICKNESS = 3

LOOKAHEAD_DEPTH = 15
REACT_DEPTH = 10

RED = (0, 0, 255)
GREEN = (0, 255, 0)
YELLOW = (0, 255, 255)

LEFT_TURN_ANGLE = -0.6
RIGHT_TURN_ANGLE = 0.6
MAX_TURN_ANGLE = 1.0

STOPPED_SPEED = 0.001
HOLD_BRAKE = 1
RELEASE_BRAKE = 0
STOP_THROTTLE = 0

THRESHOLD = 0.4


class CentralControl:
    """Central Controller containing logic that processes all the sensor data"""

    def __init__(self):
        self.bridge = get_bridge()
        self.car_controls = CarControls()

        self.ready = False
        self.speed: float = INITIAL_SPEED
        self.nav_steering = 0.0
        self.lane_steering = 0.0
        self.yolo_steering = 0.0
        self.lidar_steering = 0.0
        self.reverse_steering = 0.0
        self.lidar_left = False
        self.lidar_right = False
        self.avoiding = 0
        self.cc_state: Set[CCState] = {CCState.NORMAL}

        # Store detected sign and object data. Should refresh every object-detect cycle
        self.sign_data: List[DetectionResult] = []
        self.stop_state: StopState = StopState.NONE
        self.stop_cooldown: int = INITIAL_COOLDOWN

        self.object_data: List[DetectionResult] = []

        self.avoid: List[DetectionResult] = []
        # Action, Data
        self.persistent_actions: List[Tuple[CarAction, Any]] = []
        # Store Lane Keep Assist lane detections
        self.lanes: List[LaneLine] = []
        self.lane_status: LaneBoundStatus = LaneBoundStatus.NO_BOUNDS
        self.lane_debug: LaneStatus = None
        self.l_int = 364 / 1.5
        self.l_slope = -0.6
        self.r_int = 137.28205 / 1.5
        self.r_slope = 0.27991

        self.target_speed_pub = rospy.Publisher('target_speed', Float64, queue_size=10)
        self.current_road_segment = RoadSegmentType.STRAIGHT
        self.next_road_segment = RoadWarning.SAME_AHEAD

        # TODO: implement system
        # Each subsystem will have their own formatted recommendation object?
        self.recommendations: Dict[SensorSystem: List] = {
            SensorSystem.LIDAR: [],
            SensorSystem.LANE_DETECTION: [],
            SensorSystem.CAMERA_DETECTION: [],
            SensorSystem.NAVIGATION: []
        }

        self.tick = 0
        self.approachingIntersection = False

    # Get scene image using Airsim API. For debugging purposes
    def get_scene_image(self):
        resp = self.bridge.get_image(0, SCENE_IMAGE)
        img1d = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
        # reshape array to 3 channel image array
        return img1d.reshape(resp.height, resp.width, 3)

    def listen(self):
        rospy.init_node("central_control", anonymous=True)
        #rospy.Subscriber("lane_info", LaneStatus, self.handle_lane_data)
        rospy.Subscriber("lka/steering", Float64, self.handle_lane_steering)
        rospy.Subscriber("lka/lanes", Lanes, self.handle_lka_lines)
        rospy.Subscriber("navigation", PathData, self.handle_navigation_data)
        rospy.Subscriber("throttling", Float64, self.handle_throttling_data)
        rospy.Subscriber("object_detection", DetectionResults, self.handle_object_recognition)
        rospy.Subscriber("sensor/speed", Float64, self.handle_speed)
        rospy.Subscriber("new_lidar_data", Float64MultiArray, self.handle_lidar_detection)
        rospy.Subscriber("stop_sign_detection", DetectionResults, self.handle_stop_sign)

        # Midpoint of the image width
        MID_X = IMAGE_WIDTH / 2

        rate = rospy.Rate(10)  # 10hz
        while self.next_road_segment != RoadWarning.END_OF_PATH:
            # TODO: some intelligent decision making process here?
            self.avoid.clear()

            # Get image to draw on
            scene = self.get_scene_image()

            # Check the current road segment status and define a max speed for them
            if not self.next_road_segment == RoadWarning.SAME_AHEAD:
                self.target_speed_pub.publish(ROADWARNINGSPEEDS[self.next_road_segment])
            else:
                self.target_speed_pub.publish(ROADSEGMENTSPEEDS[self.current_road_segment])

            # Iterate through the stored objects and signs (only stop signs for now)
            # to see if there are actions worth taking
            if self.stop_state != StopState.RESUMING:
                for sign in self.sign_data:
                    # Heuristic values
                    if sign.depth < REACT_DEPTH:
                        self.stop_state = StopState.STOPPING
                        self.car_controls.throttle = STOP_THROTTLE
                        self.car_controls.brake = HOLD_BRAKE
                        self.cc_state.add(CCState.STREET_RULE)
                    elif sign.depth <= LOOKAHEAD_DEPTH:
                        self.cc_state.add(CCState.STREET_RULE)
                        self.stop_state = StopState.DETECTED
                        break

            # Lane boundary calculations (left, right)
            # NOTE: The lane line detections from LKA are done on a 640 x 360 image
            # slope = (y1-y2)/(x1-x2)
            # intercept = y - ax
            # if self.lane_status == LaneBoundStatus.ONE_BOUND_LEFT or self.lane_status == LaneBoundStatus.TWO_BOUNDS:
            #     lane = self.lanes[0]
            #     l_slope = (lane.y1 - lane.y2) / (lane.x1 - lane.x2)
            #     l_int = (lane.y2 - l_slope * lane.x2)
            # else:
            #     # No bounds, use hardcoded default
            #     l_slope, l_int = -0.19868, 364 / 1.5

            # if self.lane_status == LaneBoundStatus.ONE_BOUND_RIGHT or self.lane_status == LaneBoundStatus.TWO_BOUNDS:
            #     lane = self.lanes[1] if len(self.lanes) == 2 else self.lanes[0]
            #     r_slope = (lane.y1 - lane.y2) / (lane.x1 - lane.x2)
            #     r_int = (lane.y2 - r_slope * lane.x2)
            # else:
                # No bounds, use hardcoded default
            #     r_slope, r_int = 0.27991, 137.28205 / 1.5
            if not self.object_data:
                self.cc_state.add(CCState.NORMAL)
            else:
                for obj in self.object_data:
                    is_danger = False
                    # Get x-center, y near the bottom of bounding box
                    # cx, cy = (obj.xmax + obj.xmin) / 2, 0.5 * (obj.ymax - obj.ymin) + obj.ymin
                    cx, cy = obj.xmax + (obj.xmin / 2), obj.ymax
                    # Check if in danger zone
                    if ((cy > (self.l_slope * obj.xmax + self.l_int) or cy > (self.l_slope * obj.xmin + self.l_int)) and
                            (cy > (self.r_slope * obj.xmin + self.r_int) or cy > (self.r_slope * obj.xmax + self.r_int))
                            and obj.depth < LOOKAHEAD_DEPTH):
                        is_danger = True
                        self.cc_state.add(CCState.OBJECT_AVOID)
                        self.avoid.append(obj)
                        rospy.loginfo(f'Mark as avoid: {obj}')

                    # Add bounding boxes for debug image
                    # Colours are (B, G, R)pu
                    color = RED if is_danger else GREEN
                    x1, x2, y1, y2 = math.floor(obj.xmin), math.floor(obj.xmax), math.floor(obj.ymin), math.floor(
                        obj.ymax)
                    cv.rectangle(scene, (x1, y1), (x2, y2), color, REC_THICKNESS)
                    cv.putText(scene, f'{obj.name}: {obj.depth}', (x2 + 10, y2), FONT, FONT_SCALE, color)

            # Change state if there's nothing worth avoiding
            if not self.avoid:
                self.cc_state.discard(CCState.OBJECT_AVOID)

            if CCState.OBJECT_AVOID in self.cc_state:
                # Actions taken when in Object Avoidance mode
                # Sort for closest object
                self.avoid.sort(key=lambda x: x.depth)
                # Get closest
                cur = self.avoid[0]
                # cx, cy = (math.floor((cur.xmax + cur.xmin) / 2), math.floor((cur.ymax + cur.ymin) / 2))
                cx, cy = math.floor(cur.xmax + (cur.xmin / 2)), math.floor(cur.ymax)
                # Take action based on where the object approximately is
                if cx < MID_X:
                    # Right side
                    self.yolo_steering = RIGHT_TURN_ANGLE
                    cv.putText(scene, 'Go Right', (cx, cy), FONT, FONT_SCALE, YELLOW)
                else:
                    # Left side
                    self.yolo_steering = LEFT_TURN_ANGLE
                    cv.putText(scene, 'Go Left', (cx, cy), FONT, FONT_SCALE, YELLOW)

            elif CCState.STREET_RULE in self.cc_state:
                # Actions taken when in Street Rule mode
                if self.stop_state == StopState.STOPPING:
                    self.car_controls.brake = HOLD_BRAKE
                    self.car_controls.throttle = STOP_THROTTLE
                    if self.speed <= STOPPED_SPEED:
                        # When you are stopped, you can start the resuming process
                        self.stop_state = StopState.RESUMING
                elif self.stop_state == StopState.RESUMING:
                    # Stop breaking
                    time.sleep(1)
                    self.car_controls.brake = RELEASE_BRAKE

                    # Count down cooldown period
                    if not self.sign_data:
                        self.stop_cooldown -= 1
                    else:
                        # Reset count
                        self.stop_cooldown = INITIAL_COOLDOWN

                    if self.stop_cooldown <= 0:
                        # Done the stop sign process
                        self.stop_state = StopState.NONE
                        self.cc_state.discard(CCState.STREET_RULE)

            if self.bridge.has_collided():
                self.car_controls.is_manual_gear = True
                self.car_controls.manual_gear = -1
                time.sleep(1.4)
                self.car_controls.brake = HOLD_BRAKE
                self.car_controls.is_manual_gear = False
                self.car_controls.manual_gear = 1
            else:
                self.car_controls.is_manual_gear = False
                self.car_controls.manual_gear = 1

            # Steering priority descision
            if self.car_controls.manual_gear == -1:
                self.car_controls.steering = self.reverse_steering
            elif CCState.OBJECT_AVOID in self.cc_state:
                if self.avoiding < 5:
                    self.car_controls.steering = self.yolo_steering
                    self.avoiding += 1
                else:
                    self.car_controls.steering = self.nav_steering
            elif self.approachingIntersection:
                self.car_controls.steering = self.nav_steering
            else:
                # if self.lane_status == LaneBoundStatus.NO_BOUNDS:
                    # self.car_controls.steering = 0.1
                # else:
                    # self.car_controls.steering = self.lane_steering
                self.car_controls.steering = self.nav_steering
                self.avoiding = 0
            self.car_controls.steering = self.nav_steering
            # Set Controls in simulator
            if self.ready:
                self.bridge.set_controls(self.car_controls)

                # Mark danger zones
                cv.line(scene, (0, round(self.l_int)), (round(-self.l_int / self.l_slope), 0), RED, LINE_THICKNESS)  # Left
                cv.line(scene, (639, round(639 * self.r_slope + self.r_int)), (round(-self.r_int / self.r_slope), 0), RED, LINE_THICKNESS)  # Right
                # cv.putText(scene, f'Steering:{self.lane_steering}', (100, 100), FONT, FONT_SCALE, RED)
                # Write debug image every two images
                if self.tick % 2 == 0:
                    """
                    DON'T LEAVE THIS RUNNING FOR LONG PERIODS OF TIME OR YOU WILL FILL YOUR HARD DRIVE
                    """
                    # cv.imwrite(f'/home/mango/test_imgs/n_{rospy.Time.now()}_s.png', scene)
                cv.imshow("object_avoid_scene", scene)
                cv.waitKey(1)

            rate.sleep()
            self.tick += 1

        # End of path reached, wait until stopped before shutting down
        self.car_controls.brake = HOLD_BRAKE
        self.car_controls.throttle = STOP_THROTTLE
        self.bridge.set_controls(self.car_controls)
        while self.speed != 0:
            rate.sleep()
        rospy.signal_shutdown("Path complete")

    def handle_lane_data(self, lane_data: LaneStatus):
        """
        Results from lane detection
        If the detection methods agree on the number of lane bounds a percent difference per line is returned
        If the detection methods don't agree, a single error of 100 is returned, 100 is the value itself
        There can be a max number of two lane bounds returned
        """
        rospy.loginfo(lane_data)
        self.lane_debug = lane_data
        grad_av_diff = sum(lane_data.gradient_diff) / len(lane_data.gradient_diff)
        hls_av_diff = sum(lane_data.hls_diff) / len(lane_data.hls_diff)

        if grad_av_diff < hls_av_diff < THRESHOLD:
            # Gradient is more reliable
            self.lanes = lane_data.gradient_lane_bounds
            self.lane_status = LaneBoundStatus(lane_data.lane_gradient_status)
        elif grad_av_diff < THRESHOLD:
            # HLS color thresholding is more reliable
            self.lanes = lane_data.hls_lane_bounds
            self.lane_status = LaneBoundStatus(lane_data.lane_hls_status)
        else:
            # Other two methods are deemed unreliable, use segmentation results
            self.lanes = lane_data.segmentation_lane_bounds
            self.lane_status = LaneBoundStatus(lane_data.lane_segmentation_status)

    def handle_lane_steering(self, lane_steering: Float64):
        steering = lane_steering.data
        if steering < LEFT_TURN_ANGLE:
            self.lane_steering = LEFT_TURN_ANGLE
        elif steering > RIGHT_TURN_ANGLE:
            self.lane_steering = RIGHT_TURN_ANGLE
        else:
            self.lane_steering = steering
        if self.approachingIntersection or CCState.OBJECT_AVOID in self.cc_state:
            return
        # self.car_controls.steering = self.lane_steering

    def handle_lka_lines(self, lanes: Lanes):
        if lanes.lane_lines[0].exists:
            self.l_int = lanes.lane_lines[0].y_cept
            self.l_slope = lanes.lane_lines[0].slope
            self.lane_status = LaneBoundStatus.ONE_BOUND_LEFT
        else:
            self.l_slope, self.l_int = -0.6, 364 / 1.5

        if lanes.lane_lines[1].exists:
            self.r_int = lanes.lane_lines[1].y_cept
            self.r_slope = lanes.lane_lines[1].slope
            self.lane_status = LaneBoundStatus.ONE_BOUND_RIGHT
        else:
            self.r_slope, self.r_int = 0.27991, 137.28205 / 1.5

        if lanes.lane_lines[0].exists and lanes.lane_lines[1].exists:
            self.lane_status = LaneBoundStatus.TWO_BOUNDS
        elif not lanes.lane_lines[0].exists and not lanes.lane_lines[1].exists:
            self.lane_status = LaneBoundStatus.NO_BOUNDS




    def handle_speed(self, speed: Float64):
        self.speed = speed.data

    # Results from navigation
    # Returns the steering angle from pure pursuit
    # Returns the current road segment type
    # Returns a warning of a new road segment if one is within 5 meter of the car
    def handle_navigation_data(self, navigation_data: PathData):
        self.nav_steering = navigation_data.steering_angle
        self.current_road_segment = RoadSegmentType(navigation_data.current_segment)
        self.next_road_segment = RoadWarning(navigation_data.next_segment)

        self.approachingIntersection = RoadWarning(navigation_data.next_segment) == RoadWarning.INTERSECTION_AHEAD or \
                                       RoadSegmentType(navigation_data.current_segment) == RoadSegmentType.INTERSECTION
        self.car_controls.steering = navigation_data.steering_angle

    def handle_throttling_data(self, throttling_data: Float64):
        self.car_controls.throttle = throttling_data.data

    def handle_lidar_detection(self, lidar_data: Float64MultiArray):
        self.lidar_left = False
        self.lidar_right = False
        # Current implementation of lidar obstacle avoidance overrides navigation steering suggestion.
        # This can cause a crash if the car is at an intersection, where it can prevent turning or cause an opposite
        # turn than what is required depending on nearby obstacles
        if self.approachingIntersection:
            return

        # These are indexes into a bounding box for each of the points that it is composed of
        min_x = 0
        min_y = 1
        min_z = 2
        max_x = 3
        max_y = 4
        max_z = 5
        point_per_aabb = 6

        num_aabb = int(len(lidar_data.data) / point_per_aabb)

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

        # Find the point of the closest AABB for the car to avoid, if any. Only if an AABB intersects
        # with the car's AABB is an obstacle considering for avoidance

        closest_aabb_vector = None
        distance_to_aabb = None

        for aabb in all_aabbs:
            if car_aabb.intersection(aabb):
                (distance, vector_to_closest_point) = aabb.vector_to_closest_point()

                if closest_aabb_vector is None:
                    closest_aabb_vector = vector_to_closest_point
                    distance_to_aabb = distance
                elif distance < distance_to_aabb:
                    closest_aabb_vector = vector_to_closest_point
                    distance_to_aabb = distance

        # Avoid the nearest obstacle if any
        if closest_aabb_vector is not None:
            # Lidar points are relative to the car. The "y" dimension, index 1, refers to left or right.
            # The "x" dimension, index 0, refers to the horizontal distance from the car.
            if closest_aabb_vector[1] > 0:
                #self.car_controls.steering = -(min(MAX_TURN_ANGLE, 0.125 / closest_aabb_vector[0]))
                self.lidar_left = True
            elif closest_aabb_vector[1] < 0:
                #self.car_controls.steering = min(MAX_TURN_ANGLE, 0.125 / closest_aabb_vector[0])
                self.lidar_right = True


    def handle_object_recognition(self, res: DetectionResults):
        # Determine the "region" that the object falls into (all in front)
        # far_left/left/center/right/far_right
        # TODO: use angle in future?
        if not self.ready:
            self.ready = True

        # Sort through and deal with detections
        self.object_data.clear()

        detection_list: List[DetectionResult] = res.detection_results
        for detection in detection_list:
            if (detection.class_num == 0 or detection.class_num == 2 or detection.class_num == 7) and detection.confidence > 0.4:
                # Only deal with cars for now
                self.object_data.append(detection)

    def handle_stop_sign(self, res: DetectionResults):
        if not self.ready:
            self.ready = True
        # Sort through and deal with sign detection
        self.sign_data.clear()
        detection_list: List[DetectionResult] = res.detection_results
        for detection in detection_list:
            if detection.class_num == 11 and detection.confidence > 0.5:
                # Stop sign. Depth is likely going to be smaller than 25 because of the image resolution
                self.sign_data.append(detection)


if __name__ == "__main__":
    # Do something
    centralControl = CentralControl()
    centralControl.listen()
