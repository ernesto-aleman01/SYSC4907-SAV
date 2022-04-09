import rospy
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float64MultiArray

from cluster_detection import ClusterDetection

class LidarObjectDetect:

    def __init__(self):
        self.lidarPub = rospy.Publisher("lidar_data", Float64MultiArray, queue_size=10)
        self.cluster_detection = ClusterDetection()

    def listener(self):
        rospy.Subscriber("lidar", PointCloud, self.handle_lidar_data)

    def handle_lidar_data(self, data: PointCloud):
        # Bounding boxes are sent as a flat array
        float_array = Float64MultiArray()
        float_array.data = self.cluster_detection.find_bounding_boxes(data.points)
        self.lidarPub.publish(float_array)

if __name__ == "__main__":
    # Do something
    cc = LidarObjectDetect()
    cc.listener()