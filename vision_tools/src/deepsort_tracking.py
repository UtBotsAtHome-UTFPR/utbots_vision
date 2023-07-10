#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, RegionOfInterest
from vision_msgs.msg import Object, ObjectArray
from cv_bridge import CvBridge 
import cv2
import time
from darknet_ros_msgs.msg import BoundingBoxes
from deep_sort_realtime.deepsort_tracker import DeepSort

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
            for bbox in bbox_list.bounding_boxes:
                if(bbox.Class == "person"):
                    # Formats the bbox informations to the tracking method input format
                    self.detections.append(([bbox.xmin, bbox.ymin, int(bbox.xmax - bbox.xmin), int(bbox.ymax - bbox.ymin)], bbox.probabilty, bbox.Class))
        self.new_detection = True

    def mainLoop(self):
        while rospy.is_shutdown() == False:
            self.loopRate.sleep()
            if self.new_rgbImg == True and self.detections == True:
                self.new_detection, self.new_rgbImg = False

                start = time.perf_counter()

                # Performs tracking in the detected objetcs
                tracks = tracker.update_tracks(self.detections, frame=self.cv_img)
                for track in tracks:
                    if not track.is_confirmed():
                        continue
                    track_id = track.track_id
                    ltrb = track.to_ltrb()

                    # Draw the bboxes with the ids
                    cv2.rectangle(self.cv_img, (int(ltrb[0]), int(ltrb[1])), (int(ltrb[2]), int(ltrb[3])), (0,0,255), 2)
                    cv2.putText(self.cv_img, "ID" + str(track_id), (int(ltrb[0]), int(ltrb[1] - 10)), cv2.FONT_HERSHEY_SIMPLEX)

                # Calculates the FPS and draws in the image
                end = time.perf_counter()
                period = end - start
                fps = 1/period
                cv2.putText(self.cv_img, f'FPS: {int(fps)}', (20, 70), cv2.FONT_HERSHEY_SIMPLEX)

                self.pub_trackImg.publish(self.bridge.cv2_to_imgmsg(self.cv_img, encoding="rgb8"))               
                cv2.imshow('tracked_img', self.cv_img)

if __name__ == "__main__":
    DeepSortTracking(
        "/camera/rgb/image_raw",
        "/darknet_ros/bounding_boxes",
        "/utbots/vision/track_image")


 