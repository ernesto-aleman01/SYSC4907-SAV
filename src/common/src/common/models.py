import json
from typing import Tuple, Any


class LogEntry:
    def __init__(self,
                 time: str,
                 pos: Tuple[float, float],
                 steering: float,
                 throttle: float,
                 brake: float,
                 speed: float,
                 has_collided: bool,
                 stop_sign: bool,
                 lidar_detections: int):
        self.time = time
        self.pos = pos
        self.steering = steering
        self.throttle = throttle
        self.brake = brake
        self.speed = speed
        self.has_collided = has_collided
        self.stop_sign = stop_sign
        self.lidar_detections = lidar_detections

    def default(self, o: Any) -> Any:
        return json.dumps(dict(o))


class BrakePoint:

    def __init__(self, time: str, pos:Tuple[float,float]):

        self.time
        self.pos = pos
