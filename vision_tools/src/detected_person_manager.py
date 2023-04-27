#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, RegionOfInterest
from vision_msgs.msg import ImageArray
import random
import cv2
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox

enable_TRACKING = False

class Person():
    def __init__(self, coordinates, rgb_img):
        self.image = rgb_img
        self.bridge = CvBridge()

        # Messages to publish
        self.msg_roi = self.formatROI(coordinates)
        self.msg_croppedImg = self.crop_img_msg()

    def formatROI(self, coordinates):
        roi = RegionOfInterest()
        roi.x_offset = coordinates[0]
        roi.y_offset = coordinates[1]
        roi.width = coordinates[2] - roi.x_offset
        roi.height = coordinates[3] - roi.y_offset
        return roi

    def crop_img_msg(self):
        cropped_cv_img = self.image[self.msg_roi.y_offset:self.msg_roi.height, self.msg_roi.x_offset:self.msg_roi.width]
        return self.bridge.cv2_to_imgmsg(cropped_cv_img, encoding="passthrough")

    def get_img_msg(self):
        return self.msg_croppedImg
    
    def get_ROI(self):
        return self.msg_roi

class DetectedPersonManager():
    def __init__(self, new_topic_rgbImg, new_topic_boundingBoxes, topic_imageArray):

        # Control flag for new YOLO detection
        self.new_detection = False

        # Messages
        self.msg_rgbImg = None   # Image
        self.msg_bBoxes = None   # BoundingBox List
        self.msg_imgArray = ImageArray() # ImageArray

        self.cv_img = None       # CvImage
        self.bridge = CvBridge()

        # Subscribers
        self.sub_rgbImg = rospy.Subscriber(new_topic_rgbImg, Image, self.callback_rgbImg)
        self.sub_bBoxes = rospy.Subscriber(new_topic_boundingBoxes, BoundingBoxes, self.callback_bBoxes)

        # Publishers
        self.pub_imageArray = rospy.Publisher(topic_imageArray, ImageArray, queue_size=1)

        # Every Person instance of a detection
        self.person_list = [] # Person

        # ROS node
        rospy.init_node('detected_person_manager', anonymous=True)
        
        # Time
        self.loopRate = rospy.Rate(30)
        
        # Main
        self.mainLoop()

    def callback_rgbImg(self, msg):
        self.msg_rgbImg = msg
        self.cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
    def callback_bBoxes(self, msg):
        self.msg_bBoxes = msg

        for person in self.person_list:
            del person

        for bbox in self.msg_bBoxes.bounding_boxes:
            if(bbox.Class == "person"):
                coordinates = self.extract_coordinates(bbox)
                person = Person(coordinates, self.cv_img)
                self.person_list.append(person.get_img_msg())      

        for person in self.person_list:
            self.msg_imgArray.data = self.person_list
                        
        self.new_detection = True

    def extract_coordinates(self, msg):
        return [msg.xmin, msg.ymin, msg.xmax, msg.ymax]

    def mainLoop(self):
        while rospy.is_shutdown() == False:
            self.loopRate.sleep()
            self.pub_imageArray.publish(self.msg_imgArray)
            
if __name__ == "__main__":
    DetectedPersonManager(
        "/camera/rgb/image_raw",
        "/darknet_ros/bounding_boxes",
        "/utbots/vision/people_imgs")


