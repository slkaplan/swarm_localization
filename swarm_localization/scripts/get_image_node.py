#!/usr/bin/env python3

"""
This is a script that walks through some of the basics of working with
images with opencv in ROS. 

Author: Paul Ruvolo
Editors: Luke Nonas-Hunter & Sam Kaplan
"""

import rospy
from sensor_msgs.msg import Image, CameraInfo
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

    def __init__(self, camera_topic, id):
        """ Initialize the ball tracker """
        rospy.init_node('ball_tracker')
        self.cv_image = None            # the latest image from the camera
        self.binary_image = None        # the latest image passed through a color filter
        self.bridge = CvBridge()        # used to convert ROS messages to OpenCV
        self.id = id

        self.binary_image_repeated = np.zeros((600,600,3))
        self.KERNEL = np.ones((3,3),np.uint8)

        rospy.Subscriber(f"{camera_topic}/image_raw",
                         Image,
                         self.process_image)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        cv2.namedWindow(f'{self.id}_video_window')
        cv2.namedWindow(f'{self.id}_threshold_image')
        # cv2.namedWindow(f'{self.id}_contour_image')

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
        binary_image_unfiltered[-35:, :] = 0

        # Filter Image using morphological transformations
        opening = cv2.morphologyEx(binary_image_unfiltered,
                                   cv2.MORPH_OPEN,
                                   self.KERNEL,
                                   iterations=2)
        # Finding sure foreground area
        dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
        ret, sure_fg = cv2.threshold(dist_transform,
                                     0.7 * dist_transform.max(),
                                     255,
                                     0)
        sure_fg = np.uint8(sure_fg)
        # sure background area
        sure_bg = cv2.dilate(opening,self.KERNEL,iterations=3)
        unknown = cv2.subtract(sure_bg, sure_fg)
        # Marker labelling
        ret, markers = cv2.connectedComponents(sure_fg)
        markers = markers + 1
        markers[unknown==255] = 0
        markers = cv2.watershed(self.cv_image, markers)
        self.cv_image[markers == -1] = [255,0,0]
        binary_image_unfiltered[markers == -1] = 0
        binary_image_filtered = cv2.erode(binary_image_unfiltered,
                                          self.KERNEL,
                                          iterations=2)
        self.binary_image = binary_image_filtered
        self.find_contours(binary_image_filtered)


    def find_contours(self, binary_image):
        # Find contours of binary image
        contours, hierarchy = cv2.findContours(binary_image,
                                               cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            # Calculate moments for each contour
            moment = cv2.moments(contour)

            contour_image = np.uint8(np.zeros(binary_image.shape))
            cv2.drawContours(contour_image, [contour],-1,1,thickness=cv2.FILLED)

            # Find the average color of the inside of the contour.
            average_color = cv2.mean(self.cv_image, contour_image)

            # calculate x, y coordinate of center
            contour_x = int(moment["m10"] / moment["m00"])
            contour_y = int(moment["m01"] / moment["m00"])
            cv2.circle(self.cv_image, (contour_x, contour_y),
                       5,
                       (0, 0, 0),
                       -1)
            cv2.putText(self.cv_image,
                        "centroid",
                        (contour_x - 5, contour_y - 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.75,
                        (0, 0, 0),
                        2)

    def get_camera_info(self, msg):
         

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if not self.cv_image is None:
                # print(self.cv_image.shape)
                cv2.imshow(f'{self.id}_video_window', self.cv_image)
                cv2.waitKey(5)
            if not self.binary_image is None:
                cv2.imshow(f'{self.id}_threshold_image', self.binary_image)
            # start out not issuing any motor commands
            r.sleep()

if __name__ == '__main__':
    # redBOT = GetImage('/robot1/camera/image_raw','red')
    # redBOT.run()

    yellowBOT = GetImage('/robot2/camera','yellow')
    yellowBOT.run()