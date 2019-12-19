import rospy
from std_msgs.msg import Int32MultiArray

class ObstacleDetector:

    def __init__(self, topic):
        self.left = -1
        self.mid = -1
        self.right = -1
        self.side_left = -1
        self.side_right = -1
	self.back = -1
        rospy.Subscriber(topic, Int32MultiArray, self.read_distance)
        
    def read_distance(self, data):
        self.left = data.data[0]
        self.mid = data.data[1]
        self.right = data.data[2]
        self.side_left = data.data[6]
        self.side_right = data.data[7]
	self.back = data.data[4]
    def get_distance(self):
#	print("mid", self.mid, "side_right", self.right, "side_left", self.left)
        return self.left, self.mid, self.right, self.side_left, self.side_right, self.back
