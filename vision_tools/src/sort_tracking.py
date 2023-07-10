#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, RegionOfInterest
from vision_msgs.msg import Object, ObjectArray
from cv_bridge import CvBridge 
import cv2
import numpy as np
import time
from darknet_ros_msgs.msg import BoundingBoxes
from sort.tracker import SortTracker

class DeepSortTracking():
    def __init__(self, new_topic_rgbImg, new_topic_boundingBoxes, topic_trackImg):

        # Messages
        self.msg_rgbImg = Image()   # Image

        # OpenCV
        self.cv_img = None       # CvImage
        self.bridge = CvBridge()

        # Flags
        self.new_rgbImg = False
        self.new_detection = False

        # Subscribers
        self.sub_rgbImg = rospy.Subscriber(new_topic_rgbImg, Image, self.callback_rgbImg)
        self.sub_bBoxes = rospy.Subscriber(new_topic_boundingBoxes, BoundingBoxes, self.callback_bBoxes)

        # Publishers
        self.pub_trackImg = rospy.Publisher(topic_trackImg, Image, queue_size=1)

        # Detections list of tuples ([left,top,w,h], confidence, detection_class)
        self.detections = []

        self.tracker = SortTracker(max_age=30, 
                                   min_hits=2)

        # ROS node
        rospy.init_node('deep_sort_tracking', anonymous=True)
        
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
        if self.new_rgbImg == True:
            self.detections = np.empty((0, 5))
            for bbox in bbox_list.bounding_boxes:
                # if(bbox.Class == "person"):
                # Formats the bbox informations to the tracking method input format
                detection = np.array([[bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax, bbox.probability]], dtype="object")
                detection = detection.reshape(1, 5)
                self.detections = np.append(self.detections, detection, axis=0)
            
            remaining = 5 - np.size(self.detections)
            if remaining > 0:
                for i in range(5 - remaining, 5): 
                    self.detections = np.append(self.detections, np.array[0,0,0,0,0.0], axis=i)
                    
        self.new_detection = True

    def mainLoop(self):
        while rospy.is_shutdown() == False:
            self.loopRate.sleep()
            cv_img = self.cv_img
            if self.new_rgbImg == True and self.new_detection == True:
                self.new_detection, self.new_rgbImg = False, False

                start = time.perf_counter()

                # Performs tracking in the detected objetcs
                tracks = self.tracker.update(self.detections, self.cv_img)
                for track in tracks:
                    if not track.is_confirmed():
                        continue
                    track_id = track.track_id
                    ltrb = track.to_ltrb()

                    # Draw the bboxes with the ids
                    cv2.rectangle(cv_img, (int(ltrb[0]), int(ltrb[1])), (int(ltrb[2]), int(ltrb[3])), (0,0,255), 2)
                    cv2.putText(cv_img, "ID" + str(track_id), (int(ltrb[0]), int(ltrb[1] - 10)), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 2)

                # Calculates the FPS and draws in the image
                end = time.perf_counter()
                period = end - start
                fps = 1/period
                cv2.putText(cv_img, f'FPS: {int(fps)}', (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 2)

                self.pub_trackImg.publish(self.bridge.cv2_to_imgmsg(cv_img, encoding="rgb8"))          
                # cv2.imshow('tracked_img', cv_img)
                # cv2.destroyAllWindows()


if __name__ == "__main__":
    DeepSortTracking(
        "/camera/rgb/image_raw",
        "/darknet_ros/bounding_boxes",
        "/utbots/vision/track_image")


 