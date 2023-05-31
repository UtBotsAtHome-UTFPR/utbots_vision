#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, RegionOfInterest
from vision_msgs.msg import Object, ObjectArray
from cv_bridge import CvBridge  
from darknet_ros_msgs.msg import BoundingBoxes

class DetectedPersonManager():
    def __init__(self, new_topic_rgbImg, new_topic_boundingBoxes, topic_personArray):

        # Messages
        self.msg_rgbImg = Image()   # Image
        self.msg_personArray = ObjectArray() # PersonArray
        self.msg_person = Object()

        self.msg_img = Image()

        # OpenCV
        self.cv_img = None       # CvImage
        self.bridge = CvBridge()

        # Flags
        self.new_rgbImg = False

        # Subscribers
        self.sub_rgbImg = rospy.Subscriber(new_topic_rgbImg, Image, self.callback_rgbImg)
        self.sub_bBoxes = rospy.Subscriber(new_topic_boundingBoxes, BoundingBoxes, self.callback_bBoxes)

        # Publishers
        self.pub_personArray = rospy.Publisher(topic_personArray, ObjectArray, queue_size=1)
        ## Temporary test
        self.pub_image = rospy.Publisher("/utbots/vision/person/selected/image", Image, queue_size=1)
        self.pub_person = rospy.Publisher("/utbots/vision/person/selected/object", Object, queue_size=1)
        ##

        # ROS node
        rospy.init_node('detected_person_manager', anonymous=True)
        
        # Time
        self.loopRate = rospy.Rate(30)
        
        # Main
        self.mainLoop()

    def callback_rgbImg(self, msg):
        self.msg_rgbImg = msg
        self.cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        self.new_rgbImg = True
            
    def callback_bBoxes(self, msg):
        bbox_list = msg

        person_list = []

        if self.new_rgbImg == True:
            self.new_rgbImg = False
            for bbox in bbox_list.bounding_boxes:
                if(bbox.Class == "person"):
                    person = Object()
                    person.class_.data = "person"
                    person.parent_img = self.msg_rgbImg
                    person.roi = self.formatROI(bbox)
                    person.cropped = self.crop_img_msg(self.cv_img, bbox)
                    person_list.append(person)        
        # Temporary test
        if len(person_list) > 0: 
            self.msg_img = person_list[0].cropped
            self.msg_person = person_list[0]
        #
        self.msg_personArray.array = person_list
                    
    def crop_img_msg(self, image, bbox):
        cropped_cv_img = image[bbox.ymin:bbox.ymax, bbox.xmin:bbox.xmax]
        # try:
        img_msg = self.bridge.cv2_to_imgmsg(cropped_cv_img, encoding="rgb8")
        # except:
            # img_msg = Image()
        return img_msg
    
    def formatROI(self, bbox):
        roi = RegionOfInterest()
        roi.x_offset = bbox.xmin
        roi.y_offset = bbox.ymin
        roi.width = bbox.xmax - roi.x_offset
        roi.height = bbox.ymax - roi.y_offset
        return roi

    def mainLoop(self):
        while rospy.is_shutdown() == False:
            self.loopRate.sleep()
            self.pub_personArray.publish(self.msg_personArray)
            # Temporary test
            self.pub_image.publish(self.msg_img)
            self.pub_person.publish(self.msg_person)
            #
            
if __name__ == "__main__":
    DetectedPersonManager(
        "/camera/rgb/image_raw",
        "/darknet_ros/bounding_boxes",
        "/utbots/vision/person/detection/personImgArray")


