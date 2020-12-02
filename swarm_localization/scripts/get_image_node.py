#!/usr/bin/env python3

"""
This is a script that walks through some of the basics of working with
images with opencv in ROS. 

Author: Paul Ruvolo
Editors: Luke Nonas-Hunter & Sam Kaplan
"""

import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3

class GetImage(object):
    """ The BallTracker is a Python object that encompasses a ROS node 
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic, id):
        """ Initialize the ball tracker """
        rospy.init_node('ball_tracker')
        self.cv_image = None                        # the latest image from the camera
        self.binary_image = None                    # the latest image passed through a color filter
        self.cut_out_image = None                   # Result of cv_image .* binary_image
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        self.id = id
        
        self.binary_image_repeated = np.zeros((600,600,3))
        self.KERNEL = np.ones((5,5),np.uint8)

        rospy.Subscriber(image_topic, Image, self.process_image)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        cv2.namedWindow(f'{self.id}_video_window')
        cv2.namedWindow(f'{self.id}_threshold_image')
        cv2.namedWindow(f'{self.id}_contour_image')

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        
        # Convert ROS msg to OpenCV
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        # Convert color image to binary image and filter out backgroud
        # Background is assumed to be made up of grey objects
        cv_image_hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV )
        binary_image_inverted = cv2.inRange(cv_image_hsv, (0,0,0), (255,5,225))
        binary_image_unfiltered = cv2.bitwise_not(binary_image_inverted)

        # Filter Image using morphological transformations
        filter_1 = cv2.morphologyEx(binary_image_unfiltered, cv2.MORPH_OPEN, self.KERNEL)
        filter_2 = cv2.morphologyEx(filter_1, cv2.MORPH_CLOSE, self.KERNEL)
        self.binary_image = filter_2
        self.binary_image_repeated[:,:,0] = self.binary_image
        self.binary_image_repeated[:,:,1] = self.binary_image
        self.binary_image_repeated[:,:,2] = self.binary_image
        print(self.cv_image)
        self.cut_out_image = np.multiply(self.binary_image_repeated, self.cv_image)
        self.find_contours(self.cut_out_image)

    

    def find_contours(self, image):
        # Grayscale (even though its already binary)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
        
        # Find Canny edges 
        edged = cv2.Canny(gray, 30, 200) 
        
        # keep in mind this will edit the 'edged' image 
        contours, hierarchy = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
        
        
        # Draw all contours 
        # -1 signifies drawing all contours 
        cv2.drawContours(image, contours, -1, (0, 255, 0), 3) 
        
        cv2.imshow(f'{self.id}_contour_image', image) 



    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if not self.cv_image is None:
                #print(self.cv_image.shape)
                cv2.imshow(f'{self.id}_video_window', self.cv_image)
                cv2.waitKey(5)
            if not self.cut_out_image is None:
                cv2.imshow(f'{self.id}_threshold_image', self.cut_out_image)
            # start out not issuing any motor commands
            r.sleep()

if __name__ == '__main__':
    # redBOT = GetImage('/robot1/camera/image_raw','red')
    # redBOT.run()

    yellowBOT = GetImage('/robot2/camera/image_raw','yellow')
    yellowBOT.run()