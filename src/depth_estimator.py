#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from math import radians, sqrt, tan
import numpy as np

import cv2

class DepthEstimatorNode():
    def __init__(self, topic_point, topic_depthImg, topic_boundingBox, camFov_vertical, camFov_horizontal):
        # Image FOV for trig calculations
        self.camFov_vertical = camFov_vertical
        self.camFov_horizontal = camFov_horizontal
        
        # Published 3D point
        self.msg_point = None
        self.pub_point = rospy.Publisher(
            topic_point, 
            Point, 
            queue_size=0)

        # Subscribed depth img
        self.msg_depthImg = None
        self.newDepthImg = False
        self.sub_depthImg = rospy.Subscriber(
            topic_depthImg,
            Image,
            self.callback_depthImg,
            queue_size=1)

        # Subscribed bounding box
        self.msg_boundingBox = None
        self.newBoundingBox = False
        self.sub_boundingBox = rospy.Subscriber(
            topic_boundingBox,
            BoundingBox2D,
            self.callback_boundingBox,
            queue_size=1)

        # ROS node
        rospy.init_node('depth_estimator', anonymous=True)

        # Time
        self.loopRate = rospy.Rate(30)

        # Loop
        rospy.loginfo("Starting loop...")
        self.mainLoop()

    def callback_depthImg(self, msg):
        self.msg_depthImg = msg
        self.newDepthImg = True

    def callback_boundingBox(self, msg):
        self.msg_boundingBox = msg
        self.newBoundingBox = True

    def CropDepthImg(self, img, imgEncoding, boxCenter, boxWidth, boxHeight):
        if imgEncoding == "32FC1":
            imageHeight, imageWidth = img.shape
        else:
            imageHeight, imageWidth, a = img.shape

        boxWidth = max(abs(boxWidth), 2)
        boxHeight = max(abs(boxHeight), 2)

        x0 = max(int(boxCenter.x * imageWidth - boxWidth/2), 0)
        y0 = max(int(boxCenter.y * imageHeight - boxHeight/2), 0)

        xf = max(int(boxCenter.x * imageWidth + boxWidth/2), imageWidth)
        yf = max(int(boxCenter.y * imageHeight + boxHeight/2), imageHeight)

        cropped_img = img[y0:yf, x0:xf]
        return cropped_img

    def GetAverageDepth(self, depthImg):
        height, width = depthImg.shape
        npArray = depthImg[0:height, 0:width]

        rowMeans = np.array([])
        for row in npArray:
            rowMeans = np.append(rowMeans, np.mean(row))

        depthMean = np.mean(rowMeans)
        return depthMean

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

    def XyzToZxy(self, point):
        return Point(point.z, point.x, point.y)

    def mainLoop(self):
        while rospy.is_shutdown() == False:
            self.loopRate.sleep()

            if self.newDepthImg == True and self.newBoundingBox == True:
                self.newDepthImg = False
                self.newBoundingBox = False

                try:
                    croppedDepthImg = self.CropDepthImg(
                        self.msg_depthImg,
                        "32FC1",
                        self.msg_boundingBox.center,
                        self.msg_boundingBox.size_x,
                        self.msg_boundingBox.size_y)

                    cv2.imshow("Cropped Depth", croppedDepthImg)
                    if cv2.waitKey(5) & 0xFF == 27:
                        break

                    averageDepth = self.GetAverageDepth(croppedDepthImg)

                    self.msg_point = self.Get3dPointFromDepthPixel(
                        self.msg_boundingBox.center, 
                        averageDepth)
                    self.msg_point = self.XyzToZxy(self.msg_point)
                    self.pub_point.publish(self.msg_point)

                except:
                    print("------------- Error in depth crop -------------")
                    continue

if __name__ == '__main__':
    DepthEstimatorNode(
        "/apollo/vision/depth_estimator/point", 
        "/apollo/vision/depth_estimator/depth",
        "/apollo/vision/depth_estimator/box",
        43,
        57)