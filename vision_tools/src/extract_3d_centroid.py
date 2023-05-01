import rospy
from sensor_msgs.msg import Image, RegionOfInterest
from vision_msgs.msg import Object, ObjectArray
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge  
# Math 
import numpy as np
from math import pow, sqrt, tan, radians

class Extract3DCentroid():
    def __init__(self, topicDepthImg, topicObject, camFov_vertical, camFov_horizontal):
        # Image FOV for trig calculations
        self.camFov_vertical = camFov_vertical
        self.camFov_horizontal = camFov_horizontal

        # Messages
        self.msg_cvDepthImg           = None
        self.msg_roi                = RegionOfInterest()
        self.msg_obj                = Object()
        self.msg_centroidPoint      = PointStamped()

        # Subscribers
        self.sub_depthImg = rospy.Subscriber(topicDepthImg, Image, self.callback_depthImg)
        self.sub_object = rospy.Subscriber(topicObject, Object, self.callback_object)
        
        self.pub_centroidPoint = rospy.Publisher(
            "/utbots/vision/selected/objectPoint", PointStamped, queue_size=10)
        
        rospy.init_node("extract_3d_centroid")

        # Time
        self.loopRate = rospy.Rate(30)
        # Cv
        self.cvBridge = CvBridge()
        # self.mainLoop()

    def callback_depthImg(self, msg):
        self.msg_cvDepthImg = self.cvBridge.imgmsg_to_cv2(msg, "32FC1")

    def callback_object(self, msg):
        self.msg_obj = msg

    def calculate_2d_centroid(self, roi):
        mean_y = roi.y_offset + roi.height//2
        mean_x = roi.x_offset + roi.width//2
        return Point()
    
    