#!/usr/bin/python3

# Node for training the network for a new person

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from reportlab.lib.pagesizes import letter
from reportlab.pdfgen import canvas
import cv2
import rospkg

class GetDetectionFrame():
    def __init__(self):
        self.bridge = CvBridge()
        self.detMsg = BoundingBoxes()
        self.detImg = Image()
        self.gotMsg, self.gotImg = False, False
        self.package_path = rospkg.RosPack().get_path('vision_tools')
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.callback_bBoxes)
        rospy.Subscriber("/darknet_ros/detection_image", Image, self.callback_image)
        self.pub_detectionImage = rospy.Publisher("/camera/image", Image, queue_size=10)  
        rospy.init_node('get_detection_frame', anonymous=True)
        self.loopRate = rospy.Rate(30)
        self.main()

    def callback_bBoxes(self, msg):    
        self.detMsg = msg
        self.gotMsg = True

    def callback_image(self, msg):
        self.detImg = msg
        if self.gotMsg == True:
            self.gotImg = True

    def main(self):
        while rospy.is_shutdown() == False:
            self.loopRate.sleep()
            if self.gotImg and self.gotMsg:
                # Create a PDF file
                c = canvas.Canvas(f"{self.package_path}/detection/output.pdf", pagesize=letter)

                self.pub_detectionImage.publish(self.detImg)
                cv_image = self.bridge.imgmsg_to_cv2(self.detImg, desired_encoding="bgr8")
                # Save the image as PNG
                cv2.imwrite(f"{self.package_path}/detection/output_image.png", cv_image)
                # Add the image to the PDF
                c.drawImage(f"{self.package_path}/detection/output_image.png", 100, 500, width=400, height=300)

                textY0 = 480
                rospy.loginfo("Class | Probability")
                person_num = 0 
                c.drawString(100, textY0, "Class | Probability")
                for bbox in self.detMsg.bounding_boxes:
                    if bbox.Class == "Person":
                        person_num += 1
                    
                rospy.loginfo(f"Number of people | {person_num}")
                c.drawString(100, textY0, f"Number of people | {person_num}")
                print(bbox.Class)

                # Save the PDF file
                c.save()

                # Shutdown the ROS node after saving the message
                rospy.signal_shutdown('Got detection frame...')
                
if __name__ == "__main__":
    GetDetectionFrame()