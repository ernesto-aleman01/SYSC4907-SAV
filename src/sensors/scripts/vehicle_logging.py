import json

import rospy
from pathlib import Path
from datetime import datetime
from typing import List
from common.bridge import get_bridge
from common.models import LogEntry
from std_msgs.msg import Float64MultiArray
from sign_car_recognition.msg import DetectionResult, DetectionResults

class Logging:
    def __init__(self):
        self.client = get_bridge()
        rospy.init_node('logging', anonymous=True)
        rospy.Subscriber('new_lidar_data', Float64MultiArray, self.update_detections)
        rospy.Subscriber("stop_sign_detection", DetectionResults, self.update_sign_value)
        rospy.on_shutdown(self.write_log)
        self.rate = rospy.Rate(1)  # 1 Hz
        self.log: List[LogEntry] = []
        self.current_detections = 0
        self.stop_sign = False

    def update_detections(self, lidar_data: Float64MultiArray):
        self.current_detections = len(lidar_data.data)

    def update_sign_value(self,res: DetectionResults):
        detection_list: List[DetectionResult] = res.detection_results
        for detection in detection_list:
            if detection.confidence > 0.6:
                self.stop_sign = True

    def create_log(self):
        while not rospy.is_shutdown():
            time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            pos = self.client.get_position().two_tuple()
            steering = self.client.get_steering()
            throttle = self.client.get_throttle()
            has_collided = self.client.has_collided()
            speed = self.client.get_speed()
            brake = self.client.get_brake()
            self.log.append(LogEntry(time, pos, steering, throttle, brake, speed, has_collided, self.stop_sign, self.current_detections))
            self.stop_sign = False
            self.rate.sleep()

    def write_log(self):
        path_file: str = rospy.get_param('/path_file')
        path_name = path_file.split('.')[0]  # Get file name
        log_path = Path(__file__).parents[3] / 'testing' / 'AirSimTests' / 'log' / f'{path_name}.log'
        with open(log_path, 'w') as f:
            f.write(json.dumps([vars(entry) for entry in self.log]))


if __name__ == '__main__':
    logger = Logging()
    logger.create_log()
