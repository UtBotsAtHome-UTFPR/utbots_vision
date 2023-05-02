#!/usr/bin/env python3

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
        self.msg_cvDepthImg         = None
        self.msg_roi                = RegionOfInterest()
        self.msg_obj                = Object()
        self.msg_centroidPoint      = PointStamped()

        # Subscribers
        self.sub_depthImg = rospy.Subscriber(topicDepthImg, Image, self.callback_depthImg)
        self.sub_object = rospy.Subscriber(topicObject, Object, self.callback_object)
        
        self.pub_centroidPoint = rospy.Publisher(
            "/utbots/vision/selected/objectPoint", PointStamped, queue_size=10)
        
        rospy.init_node("extract_3d_centroid", anonymous=True)

        # Time
        self.loopRate = rospy.Rate(30)
        # Cv
        self.cvBridge = CvBridge()
        self.mainLoop()

    def callback_depthImg(self, msg):
        rospy.loginfo("Depth")
        self.msg_cvDepthImg = self.cvBridge.imgmsg_to_cv2(msg, "32FC1")

    def callback_object(self, msg):
        rospy.loginfo("Object")
        self.msg_obj = msg
        self.msg_centroidPoint = self.calculate_3d_centroid(msg.roi)

    def calculate_3d_centroid(self, roi):
        mean_y = roi.y_offset + roi.height//2
        mean_x = roi.x_offset + roi.width//2
        rospy.loginfo(self.msg_cvDepthImg[mean_x, mean_y])
        return self.Get3dPointFromDepthPixel(Point(mean_x, mean_y, 0), self.msg_cvDepthImg[mean_x, mean_y])
    
    ''' By using rule of three and considering the FOV of the camera:
            - Calculates the 3D point of a depth pixel '''
    def Get3dPointFromDepthPixel(self, pixelPoint, depth):

        # Constants
        maxAngle_x = self.camFov_horizontal/2
        maxAngle_y = self.camFov_vertical/2
        screenMax_x = 1.0
        screenMax_y = 1.0
        screenCenter_x = screenMax_x / 2.0
        screenCenter_y = screenMax_y / 2.0

        # Distances to screen center
        distanceToCenter_x = pixelPoint.x - screenCenter_x
        distanceToCenter_y = pixelPoint.y - screenCenter_y

        # Horizontal angle (xz plane)
        xz_angle_deg = maxAngle_x * distanceToCenter_x / screenCenter_x
        xz_angle_rad = radians(xz_angle_deg)
        
        # Vertical angle (yz plane)
        yz_angle_deg = maxAngle_y * distanceToCenter_y / screenCenter_y
        yz_angle_rad = radians(yz_angle_deg)

        # Coordinates
        num = depth
        denom = sqrt(1 + pow(tan(xz_angle_rad), 2) + pow(tan(yz_angle_rad), 2))
        z = num / denom
        x = z * tan(xz_angle_rad)
        y = z * tan(yz_angle_rad)

        # Corrections
        x = -x
        y = -y

        return Point(x, y, z)
    
    def mainLoop(self):
        while rospy.is_shutdown() == False:
            rospy.loginfo("LOop")
            self.loopRate.sleep()
            self.pub_centroidPoint.publish(self.msg_centroidPoint)
    

if __name__ == '__main__':
    Extract3DCentroid(
    "/camera/depth/image_raw",
    "/utbots/vision/selected/object",
    43,
    57)
