#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, RegionOfInterest
from vision_msgs.msg import Object, ObjectArray
from geometry_msgs.msg import PointStamped, Point, TransformStamped
from tf.msg import tfMessage
from cv_bridge import CvBridge  
import cv2
# Math 
import numpy as np
from math import pow, sqrt, tan, radians

class Extract3DCentroid():
    def __init__(self, topicDepthImg, topicObject, camFov_vertical, camFov_horizontal):
        # Image FOV for trig calculations
        self.camFov_vertical = camFov_vertical
        self.camFov_horizontal = camFov_horizontal

        # Messages
        self.msg_tfStamped          = TransformStamped()
        self.msg_cvDepthImg         = None
        self.msg_roi                = RegionOfInterest()
        self.msg_obj                = Object()
        self.msg_centroidPoint      = PointStamped()
        self.msg_centroidPoint.header.frame_id = "object_center"

        # Flags
        self.new_depthMsg = False
        self.new_objMsg = False

        # Subscribers
        self.sub_depthImg = rospy.Subscriber(topicDepthImg, Image, self.callback_depthImg)
        self.sub_object = rospy.Subscriber(topicObject, Object, self.callback_object)
        
        self.pub_centroidPoint = rospy.Publisher(
            "/utbots/vision/selected/objectPoint", PointStamped, queue_size=1)
        self.pub_tf = rospy.Publisher(
            "/tf", tfMessage, queue_size=10)
        
        # Cv
        self.cvBridge = CvBridge()
        
        rospy.init_node("extract_3d_centroid", anonymous=True)

        # Time
        self.loopRate = rospy.Rate(50)
        self.mainLoop()

    def callback_depthImg(self, msg):
        self.msg_cvDepthImg = self.cvBridge.imgmsg_to_cv2(msg, "32FC1")
        self.new_depthMsg = True

    def callback_object(self, msg):
        self.msg_obj = msg
        self.new_objMsg = True

    def getMeanDistance(self):
        # Stores the point distances of every point inside the object's region of interest
        npArray = self.msg_cvDepthImg[0:self.msg_obj.roi.height, 0:self.msg_obj.roi.width]

        # Calculates the mean value
        rowMeans = np.array([])
        for row in npArray:
            rowMeans = np.append(rowMeans, np.mean(row))
        depthMean = np.mean(rowMeans)

        return depthMean

    # Redefines the xy point 
    def redefineScale(self, point):
        x = point.x/self.msg_obj.parent_img.width
        y = point.y/self.msg_obj.parent_img.height
        return Point(x, y, 0)

    def calculate_3d_centroid(self, roi):
        mean_y = roi.y_offset + roi.height//2
        mean_x = roi.x_offset + roi.width//2
        return self.get3dPointFromDepthPixel(self.redefineScale(Point(mean_x, mean_y, 0)), self.getMeanDistance())
    
    # By using rule of three and considering the FOV of the camera: Calculates the 3D point of a depth pixel '''
    def get3dPointFromDepthPixel(self, pixelPoint, pointRadius):
        # Constants
        # -- Half angle for each quadrant
        maxAngle_x = self.camFov_horizontal/2
        maxAngle_y = self.camFov_vertical/2
        # -- Screen dimensions are adjusted to 0-1 in X and Y
        CENTER_X = 1/2.0
        CENTER_Y = 1/2.0

        # Distances to screen center
        distanceToCenter_x = pixelPoint.x - CENTER_X
        distanceToCenter_y = pixelPoint.y - CENTER_Y

        # Horizontal angle (xz plane)
        theta = radians(maxAngle_x * distanceToCenter_x / CENTER_X)
        
        # Vertical angle (yz plane)
        phi = radians(maxAngle_y * distanceToCenter_y / CENTER_Y)

        # Coordinates
        # -- mm to m  conversion 
        pointRadius = pointRadius / 1000 

        z = pointRadius/ sqrt(1 + pow(tan(theta), 2) + pow(tan(phi), 2))
        x = z * tan(theta)
        y = z * tan(phi)

        # Corrections
        x = -x
        y = -y

        return Point(x, y, z)
    
    # Transformation tree methods
    def SetupTfMsg(self):
        self.msg_tfStamped.header.frame_id = "camera_link"
        self.msg_tfStamped.header.stamp = rospy.Time.now()
        self.msg_tfStamped.child_frame_id = "object_center"
        self.msg_tfStamped.transform.translation.x = 0
        self.msg_tfStamped.transform.translation.y = 0
        self.msg_tfStamped.transform.translation.z = 0
        self.msg_tfStamped.transform.rotation.x = 0.0
        self.msg_tfStamped.transform.rotation.y = 0.0
        self.msg_tfStamped.transform.rotation.z = 0.0
        self.msg_tfStamped.transform.rotation.w = 1.0

        msg_tf = tfMessage([self.msg_tfStamped])
        self.pub_tf.publish(msg_tf)
    
    def mainLoop(self):
        while rospy.is_shutdown() == False:
            self.loopRate.sleep()
            if(self.new_objMsg == True and self.new_depthMsg == True):
                self.msg_centroidPoint.point = self.calculate_3d_centroid(self.msg_obj.roi)
            self.pub_centroidPoint.publish(self.msg_centroidPoint)
            self.SetupTfMsg()
    

if __name__ == '__main__':
    Extract3DCentroid(
    "/camera/depth/image_raw",
    "/utbots/vision/selected/object",
    43,
    57)
