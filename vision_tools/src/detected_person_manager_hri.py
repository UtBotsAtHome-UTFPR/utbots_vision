#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, RegionOfInterest
import random
import cv2
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox

enable_TRACKING = False

class Person():
    def __init__(self, coordinates, rgb_img):
        self.label = self.generate_label()
        rospy.loginfo("Person "+self.label+" instance created")
        self.image = rgb_img
        self.matched = False
        self.bridge = CvBridge()

        # Messages to publish
        self.msg_roi = self.formatROI(coordinates)
        self.msg_croppedImg = self.crop_img_msg()

        # new_topics to publish
        self.pub_croppedImg = rospy.Publisher("humans/bodies/"+self.label+"/cropped", Image, queue_size=1)
        self.pub_roi = rospy.Publisher("humans/bodies/"+self.label+"/roi", RegionOfInterest, queue_size=1)

        self.publish_new_topics()

    def __del__(self):
        rospy.loginfo("Person "+self.label+" instance destroyed")
        self.pub_croppedImg.unregister()
        self.pub_roi.unregister()

    def generate_label(self):
        return "".join(random.sample("abcdefghijklmnopqrstuvwxyz", 5))

    def set_match(self, value):
        self.matched = value

    def get_match(self):
        return self.matched

    def update_coordinates(self, coordinates):
        self.msg_croppedImg = self.crop_img_msg()
        self.msg_roi = self.formatROI(coordinates)
        self.publish_new_topics()

    def crop_img_msg(self):
        cropped_cv_img = self.image[self.msg_roi.y_offset:self.msg_roi.height, self.msg_roi.x_offset:self.msg_roi.width]
        return self.bridge.cv2_to_imgmsg(cropped_cv_img, encoding="passthrough")

    def formatROI(self, coordinates):
        roi = RegionOfInterest()
        roi.x_offset = coordinates[0]
        roi.y_offset = coordinates[1]
        roi.width = coordinates[2] - roi.x_offset
        roi.height = coordinates[3] - roi.y_offset
        return roi

    def getROI(self):
        return self.msg_roi
    
    def publish_new_topics(self):
        self.pub_croppedImg.publish(self.msg_croppedImg)
        self.pub_roi.publish(self.msg_roi)

class DetectedPersonManager():
    def __init__(self, new_topic_rgbImg, new_topic_boundingBoxes):

        # Control flag for new YOLO detection
        self.new_detection = False

        # Messages
        self.msg_rgbImg = None  # Image
        self.msg_bBoxes = None  # BoundingBox List

        self.cv_img = None      # CvImage
        self.bridge = CvBridge()

        # Subscribers
        self.sub_rgbImg = rospy.Subscriber(new_topic_rgbImg, Image, self.callback_rgbImg)
        self.sub_bBoxes = rospy.Subscriber(new_topic_boundingBoxes, BoundingBoxes, self.callback_bBoxes)

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
        person_list_tmp = []

        for bbox in self.msg_bBoxes.bounding_boxes:
            coordinates = self.extract_coordinates(bbox)
            if(enable_TRACKING == True):
                repeated_person = False
                if(len(self.person_list) > 0):
                    for person in self.person_list:
                        if(self.compareROI(coordinates, person.getROI()) == True):
                            repeated_person = True
                            person.set_match(True)
                            person.update_coordinates(coordinates)
                            person_list_tmp.append(person)
                            # rospy.loginfo("Updated and added")
                        if(repeated_person == True):
                            break
                        else:
                            person_list_tmp.append(Person(coordinates, self.cv_img))
                            # rospy.loginfo("Created and added")           
                else:
                    person_list_tmp.append(Person(coordinates, self.cv_img))
                    # rospy.loginfo("Created and added")  
            else:
                person_list_tmp.append(Person(coordinates, self.cv_img))

        for person in self.person_list:
            if(person.get_match == False):
                del person
            else:
                person.set_match(False)

        self.person_list = person_list_tmp
        self.new_detection = True

    def extract_coordinates(self, msg):
        return [msg.xmin, msg.ymin, msg.xmax, msg.ymax]

    def compareROI(self, coordinates, msg):
        new_left    = coordinates[0]
        new_btm     = coordinates[1]
        new_right   = coordinates[2]
        new_top     = coordinates[3]
        new_area    = (new_top-new_btm)*(new_right-new_left)

        old_left    = msg.x_offset
        old_btm     = msg.y_offset
        old_right   = old_left+msg.width
        old_top     = old_btm+msg.height
        old_area    = msg.height*msg.width

        overlap_perc = ((max(new_left, old_left)-min(new_right, old_right))*(max(new_btm, old_btm)-min(new_top, old_top)))/old_area

        # rospy.loginfo("REGISTERED l:{}, b:{}, r:{}, t:{}".format(old_left, old_btm, old_right, old_top))
        # rospy.loginfo("NEW        l:{}, b:{}, r:{}, t:{}".format(new_left, new_btm, new_right, new_top))
        # rospy.loginfo("Heights: n - {}  o - {}".format(new_top-new_btm, msg.height))
        # rospy.loginfo(str(overlap_perc) + "\n")
        
        return (overlap_perc > 0.5) and (new_area > 0.5*old_area)

    def mainLoop(self):
        while rospy.is_shutdown() == False:
            self.loopRate.sleep()
            
if __name__ == "__main__":
    DetectedPersonManager(
        "/camera/rgb/image_raw",
        "/darknet_ros/bounding_boxes")


