#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox
from geometry_msgs.msg import PointStamped, Point, TransformStamped
from tf.msg import tfMessage
from cv_bridge import CvBridge  
import actionlib
from utbots_actions.msg import Extract3DPointAction, Extract3DPointResult
# Math 
import numpy as np
from math import pow, sqrt, sin, tan, radians

class Extract3DCentroid():
    def __init__(self, topicDepthImg, camFov_vertical, camFov_horizontal):
        # Image FOV for trig calculations
        self.camFov_vertical = camFov_vertical
        self.camFov_horizontal = camFov_horizontal

        # Messages
        self.msg_depthImg           = Image()
        self.msg_tfStamped          = TransformStamped()
        self.msg_bbox               = BoundingBox()
        self.msg_centroidPoint      = PointStamped()
        self.msg_cropped            = Image()
        self.msg_centroidPoint.header.frame_id = "object_center"

        # Subscribers
        self.sub_depthImg = rospy.Subscriber(topicDepthImg, Image, self.callback_depthImg)
        
        # Publishers
        self.pub_cropped = rospy.Publisher(
            "/utbots/vision/selected/croppedImg", Image, queue_size=1)
        self.pub_tf = rospy.Publisher(
            "/tf", tfMessage, queue_size=10)
        
        # Cv
        self.cvBridge = CvBridge()

        # Distance value (from the camera to the object)
        self.distance = 0
        
        rospy.init_node("extract_3d_centroid", anonymous=True)

        # Action server initialization
        self._as = actionlib.SimpleActionServer('extract_3d_centroid', Extract3DPointAction, self.extract3Dpoint_action, False)
        self._as.start()

        # Time
        self.loopRate = rospy.Rate(1)
        self.mainLoop()

    def callback_depthImg(self, msg):
        self.msg_depthImg = msg

# Distance of the object methods of calculation

    #-# Ordinary mean of all values (NOT VERY GOODs)
    def getMeanDistance(self):
        height, width = self.cv_depthFrame.shape
        npArray = self.cv_depthFrame[0:height, 0:width]

        rowMeans = np.array([])
        for row in npArray:
            rowMeans = np.append(rowMeans, np.mean(row))

        depthMean = np.mean(rowMeans)
        return depthMean
    
    #-# Mean from the center considering the values only less than 100mm away from the mean distance
    ## (PROMISING, BUT PERFORMS BADLY WITH TRANSPARENT OBJECTS)
    def getFilteredDistance(self):
        height, width = self.cv_depthFrame.shape
        filtPixels, distanceMean = self.SpiralFiltering(self.cv_depthFrame, width, height, 100)
        return distanceMean

    def movingAverage(self, new_value, current_mean, n_values):
        return current_mean+(new_value-current_mean)/n_values
    
    # Iterates through the image in a counterclockwise manner starting from the center
    # Returns the filtered pixels list and the list mean
    # Filters points if the point is further from the mean distance by threshold
    def SpiralFiltering(self, img, w, h, threshold): 
        hw = w//2
        hh = h//2
        x = hw
        y = hh
        dx = 1
        dy = 0
        filtPixels = []
        filtMean = 0
        nFilt = 0
        for i in range(0, int(pow(max(w,h), 2))):
            if x >= 0 and x < w and y >= 0 and y < h:
                if x == hw and y == hh:
                    filtPixels.append(img[y][x])
                    filtMean = img[y][x]
                    nFilt = 1
                elif abs(img[y][x] - filtMean) <= threshold:
                    filtPixels.append(img[y][x])
                    nFilt += 1
                    filtMean = self.movingAverage(img[y,x], filtMean, nFilt)
            cx = x - hw
            cy = y - hh
            if (cx == cy) or (cx == -cy and cx < 0) or (cx == -(1 + cy) and cx >= 0):
                store = dy
                dy = -dx
                dx = store
            x += dx
            y += dy 
        return filtPixels, filtMean
    #-#


    #-# Removes the outlier values (PROMISING, BUT CRASHES WITH BIG FRAMES)
    def getMeanDistanceWoutOutliers(self):
        allpixels = np.array([])
        height, width = self.cv_depthFrame.shape
        image = self.cv_depthFrame[0:height, 0:width]

        if image.size > 0:
            # Add every pixel to a list
            for i in range(0, height):
                for j in range(0, width):
                    if(image[i][j] != 0):
                        allpixels = np.append(allpixels, image[i][j])

            # Calculates the first quartile and third quartile (equivalent to percentile 25 and 75, respectively)
            q3 = np.percentile(allpixels, 75)
            q1 = np.percentile(allpixels, 25)
            interquartile = q3 - q1
            max = q3 + (1.5*interquartile)
            min = q1 - (1.5*interquartile)

            filteredpixels = np.array([])

            # Add only pixels within the min and max boundary to a np.array (filtered outliers)
            for pixel in allpixels:
                if(pixel < max and pixel > min):
                    filteredpixels = np.append(filteredpixels, pixel)

            # Returns the mean
            return np.mean(filteredpixels)
        else:
            return 0
    #-#

    #-# Ordinary Median of the Values (VERY PROMISING AND EASY)
    def getMedianDistance(self):
        allpixels = np.array([])
        height, width = self.cv_depthFrame.shape
        image = self.cv_depthFrame[0:height, 0:width]

        for i in range(0, height):
            for j in range(0, width):
                if(image[i][j] > 0):
                    allpixels = np.append(allpixels, image[i][j])

        return np.median(allpixels)
    #-#
#
    def calculate_3d_centroid(self, msg_bbox):
        mean_y = msg_bbox.ymin + (msg_bbox.ymax - msg_bbox.ymin)//2
        mean_x = msg_bbox.xmin + (msg_bbox.xmax - msg_bbox.xmin)//2
        calculatedDistance = self.getMeanDistanceWoutOutliers()
        # calculatedDistance = self.getFilteredDistance()
        # calculatedDistance = self.getMedianDistance()
        if calculatedDistance > 0:
            self.distance = calculatedDistance
            return self.get3dPointFromDepthPixel(Point(mean_x, mean_y, 0), self.distance)
        else:
            return Point(0,0,0)
    
    # By using rule of three and considering the FOV of the camera: Calculates the 3D point of a depth pixel '''
    def get3dPointFromDepthPixel(self, pixel, distance):
        # Set the height and width of the parent image (camera)
        width  = self.msg_bbox.xmax - self.msg_bbox.xmin 
        height = self.msg_bbox.ymax - self.msg_bbox.ymin 

        # Centralize the camera reference at (0,0,0)
        ## (x,y,z) are respectively horizontal, vertical and depth
        ## Theta is the angle of the point with z axis in the zx plane
        ## Phi is the angle of the point with z axis in the zy plane
        ## x_max is the distance of the side border from the camera
        ## y_max is the distance of the upper border from the camera
        theta_max = self.camFov_horizontal/2 
        phi_max = self.camFov_vertical/2
        x_max = width/2.0
        y_max = height/2.0
        x = pixel.x - x_max
        y = pixel.y - y_max

        # Caculate angle theta and phi
        theta = radians(theta_max * x / x_max)
        phi = radians(phi_max * y / y_max)

        # Convert the spherical radius rho from Kinect's mm to meter
        rho = distance/1000

        # Calculate x, y and z
        y = rho * sin(phi)
        x = sqrt(pow(rho, 2) - pow(y, 2)) * sin(theta)
        z = x / tan(theta)

        # Change coordinate scheme
        ## We calculate with (x,y,z) respectively horizontal, vertical and depth
        ## For the plot in 3d space, we need to remap the coordinates to (z, -x, -y)
        point_zxy = Point(z, -x, -y)

        rospy.loginfo("Euclidian distance from camera: " + str(sqrt(pow(z, 2) + pow(y, 2) + pow(x, 2))) + "m")

        return point_zxy
 
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

    def extract3Dpoint_action(self, goal):
        self.msg_bbox = goal.Object_bbox
        action_res = Extract3DPointResult()
        print(self.msg_bbox)

        if self.msg_bbox.xmax > self.msg_bbox.xmin and self.msg_bbox.ymax > self.msg_bbox.ymin:
            try:
            # Crops the full depth image
                self.cv_depthFrame = self.cvBridge.imgmsg_to_cv2(self.msg_depthImg, "32FC1")
                self.cv_depthFrame = self.cv_depthFrame[self.msg_bbox.ymin:self.msg_bbox.ymax, self.msg_bbox.xmin:self.msg_bbox.xmax]
                self.msg_cropped = self.cvBridge.cv2_to_imgmsg(self.cv_depthFrame, "passthrough")

                print(self.cv_depthFrame)

                self.msg_centroidPoint.point = self.calculate_3d_centroid(self.msg_bbox)
                self.SetupTfMsg()
                self.pub_cropped.publish(self.msg_cropped)
                
                action_res.Point = self.msg_centroidPoint
                action_res.Success = True
            except:
                action_res.Success = False
                self._as.set_aborted()
        else:
            action_res.Success = False
        self._as.set_succeeded(action_res)
    
    def mainLoop(self):
        while rospy.is_shutdown() == False:
            self.loopRate.sleep()

if __name__ == '__main__':
    Extract3DCentroid(
    "/camera/depth_registered/image_raw",
    43,
    57)
