#!/usr/bin/env python
import cv2
from cv_bridge import CvBridge, CvBridgeError
import mediapipe as mp
import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image


class LockPose():
    def __init__(self, topic_rgbImg, topic_depthImg):

        # Messages
        self.msg_targetStatus   = "?" # String
        self.msg_targetPoint    = Point()   # Point
        self.msg_rgbImg         = None      # Image
        self.msg_depthImg       = None      # Image

        # To tell if there's a new msg
        self.newRgbImg = False
        self.newDepthImg = False

        # Publishers and Subscribers
        self.pub_targetStatus = rospy.Publisher(
            "/apollo/vision/lock/target/status", String, queue_size=10)
        self.pub_targetPoint = rospy.Publisher(
            "/apollo/vision/lock/target/point", Point, queue_size=10)
        self.sub_rgbImg = rospy.Subscriber(
            topic_rgbImg, Image, self.callback_rgbImg)
        self.sub_depthImg = rospy.Subscriber(
            topic_depthImg, Image, self.callback_depthImg)

        # ROS node
        rospy.init_node('locker_human', anonymous=True)

        # Time
        self.loopRate = rospy.Rate(30)
        self.t_last = 0.0  # sec
        self.t_timeout = 0.250  # sec

        # Cv
        self.cvBridge = CvBridge()

        # Mediapipe
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_pose = mp.solutions.pose

        # Calls main loop
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.75,
            min_tracking_confidence=0.9)
        self.mainLoop()

    def callback_rgbImg(self, msg):
        self.msg_rgbImg = msg
        self.newRgbImg = True
        # print("- RGB: new msg")

    def callback_depthImg(self, msg):
        self.msg_depthImg = msg
        self.newDepthImg = True
        # print("- Depth: new msg")

    def ProcessImg(self, msg_img):
        # Conversion to cv image
        cvImg = self.cvBridge.imgmsg_to_cv2(self.msg_rgbImg, "bgr8")

        # Not writeable passes by reference (more performance)
        cvImg.flags.writeable = False

        # Converts BGR to RGB
        cvImg = cv2.cvtColor(cvImg, cv2.COLOR_BGR2RGB)

        # Image processing
        poseResults = self.pose.process(cv2.cvtColor(cvImg, cv2.COLOR_BGR2RGB))

        # To draw the hand annotations on the image
        cvImg.flags.writeable = True

        # Back to BGR
        cvImg = cv2.cvtColor(cvImg, cv2.COLOR_RGB2BGR)

        # Returns
        return cvImg, poseResults

    def DrawLandmarks(self, cv_rgbImg, poseResults):
        self.mp_drawing.draw_landmarks(
            cv_rgbImg,
            poseResults.pose_landmarks,
            self.mp_pose.POSE_CONNECTIONS,
            landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style())

    # Gets points for torso (shoulders and hips)
    def GetTorsoPoints(self, landmark):
        rightShoulder = Point(
            landmark[self.mp_pose.PoseLandmark.RIGHT_SHOULDER].x, 
            landmark[self.mp_pose.PoseLandmark.RIGHT_SHOULDER].y, 
            landmark[self.mp_pose.PoseLandmark.RIGHT_SHOULDER].z)
        leftShoulder = Point(
            landmark[self.mp_pose.PoseLandmark.LEFT_SHOULDER].x, 
            landmark[self.mp_pose.PoseLandmark.LEFT_SHOULDER].y, 
            landmark[self.mp_pose.PoseLandmark.LEFT_SHOULDER].z)
        rightHip = Point(
            landmark[self.mp_pose.PoseLandmark.RIGHT_HIP].x, 
            landmark[self.mp_pose.PoseLandmark.RIGHT_HIP].y, 
            landmark[self.mp_pose.PoseLandmark.RIGHT_HIP].z)
        leftHip = Point(
            landmark[self.mp_pose.PoseLandmark.LEFT_HIP].x, 
            landmark[self.mp_pose.PoseLandmark.LEFT_HIP].y, 
            landmark[self.mp_pose.PoseLandmark.LEFT_HIP].z)
        return [rightShoulder, leftShoulder, rightHip, leftHip]

    def GetPointsCenter(self, points):
        sum_x = 0
        sum_y = 0
        counter = 0
        for point in points:
            sum_x = sum_x + point.x
            sum_y = sum_y + point.y
            counter = counter + 1
        return Point(sum_x/counter, sum_y/counter, 0)

    def CropTorsoImg(self, img, imgEncoding, torsoPoints, torsoCenter):
        if imgEncoding == "32FC1":
            imageHeight, imageWidth = img.shape
        else:
            imageHeight, imageWidth, a = img.shape
        torsoWidth = max(abs(torsoPoints[0].x - torsoPoints[1].x) * imageWidth, 1)
        torsoHeight = max(abs(torsoPoints[0].y - torsoPoints[2].y) * imageHeight, 1)

        x0 = max(int(torsoCenter.x * imageWidth - torsoWidth/2), 0)
        y0 = max(int(torsoCenter.y * imageHeight - torsoHeight/2), 0)
        xf = min(int(torsoCenter.x * imageWidth + torsoWidth/2), imageWidth)
        yf = min(int(torsoCenter.y * imageHeight + torsoHeight/2), imageHeight)

        cropped_image = img[y0:yf, x0:xf]
        return cropped_image

    def GetTorsoDistance(self, croppedDepthImg):
        height, width = croppedDepthImg.shape
        npArray = croppedDepthImg[0:height, 0:width]

        rowMeans = np.array([])
        for row in npArray:
            rowMeans = np.append(rowMeans, np.mean(row))

        depthMean = np.mean(rowMeans)
        return depthMean

    def PublishEverything(self):
        self.pub_targetStatus.publish(self.msg_targetStatus)
        self.pub_targetPoint.publish(self.msg_targetPoint)

    def mainLoop(self):
        while rospy.is_shutdown() == False:
            self.loopRate.sleep()
            self.PublishEverything()
            print("\nTARGET")
            print(" - status: {}".format(self.msg_targetStatus))
            print(" - xyz: ({}, {}, {})".format(
                self.msg_targetPoint.x, 
                self.msg_targetPoint.y, 
                self.msg_targetPoint.z))
                
            # Else -> new RGB and new depth are true...
            if self.newRgbImg == True and self.newDepthImg == True:
                self.newRgbImg = False
                self.newDepthImg = False

                cv_rgbImg, poseResults = self.ProcessImg(self.msg_rgbImg)
                self.DrawLandmarks(cv_rgbImg, poseResults)
                cv2.imshow('MediaPipe Pose', cv_rgbImg)
                if cv2.waitKey(5) & 0xFF == 27:
                    break

                # If found landmarks...
                if poseResults.pose_landmarks:
                    torsoPoints = self.GetTorsoPoints(poseResults.pose_landmarks.landmark)
                    torsoCenter = self.GetPointsCenter(torsoPoints)
                    self.msg_point = torsoCenter
                    self.msg_targetStatus = "Located"

                    cv_depthImg = self.cvBridge.imgmsg_to_cv2(self.msg_depthImg, "32FC1")
                    cv2.imshow("depth Img", cv_depthImg)

                    try:
                        croppedRgbImg = self.CropTorsoImg(cv_rgbImg, "passthrough", torsoPoints, torsoCenter)
                        cv2.imshow("Cropped RGB", croppedRgbImg)
                    except:
                        continue
                    try:
                        croppedDepthImg = self.CropTorsoImg(cv_depthImg, "32FC1", torsoPoints, torsoCenter)
                        cv2.imshow("Cropped Depth", croppedDepthImg)
                        torsoCenter.z = self.GetTorsoDistance(croppedDepthImg)
                        self.msg_targetPoint = torsoCenter
                        self.msg_targetPoint = Point(torsoCenter.z, 0, 0)
                    except:
                        continue

                # Nothing detected...
                else:
                    t_now = rospy.get_time()
                    if (t_now - self.t_last > self.t_timeout and self.msg_targetStatus != "?"):
                        self.t_last = t_now
                        self.msg_targetPoint = Point(0, 0, 0)
                        self.msg_targetStatus = "?"

if __name__ == "__main__":
    lockHand = LockPose("/camera/rgb/image_raw", "/camera/depth_registered/image_raw")