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
import math
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from swarm_localization.msg import Vertex
import tf.transformations as t

class GetImage(object):
    """
    The BallTracker is a Python object that encompasses a ROS node 
    that can process images from the camera and search for a ball within.
    The node will issue motor commands to move forward while keeping
    the ball in the center of the camera's field of view.
    """

    def __init__(self, robot_name, robot_id):
        """
        Initializes ROS node, publishers and subscribers, and sets image processing parameters
        """
        rospy.init_node('get_data')
        self.cv_image = None            # the latest image from the camera
        self.binary_image = None        # the latest image passed through a color filter
        self.bridge = CvBridge()        # used to convert ROS messages to OpenCV
        self.id = robot_id
        self.inv_K = None
        


        self.current_odom_x = 0.0
        self.current_odom_y = 0.0
        self.current_odom_theta = 0.0

        self.other_robot = []
        self.other_robot_xy = []

        self.binary_image_repeated = np.zeros((600,600,3))
        self.KERNEL = np.ones((3,3),np.uint8)

        rospy.Subscriber(f"{robot_name}/camera/image_raw",
                         Image,
                         self.process_image)
        rospy.Subscriber(f"{robot_name}/camera/camera_info",
                         CameraInfo,
                         self.get_camera_info)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # subscribes to odometry
        self.odom_sub = rospy.Subscriber(f"{robot_name}/odom", Odometry, self.odom_process)
        self.vertex_pub = rospy.Publisher("/vertex", Vertex, queue_size=10)


        
        cv2.namedWindow(f'{self.id}_video_window')
        cv2.namedWindow(f'{self.id}_threshold_image')
        # cv2.namedWindow(f'{self.id}_contour_image')

    def process_image(self, msg):
        """
        Process image messages from ROS and stash them in an attribute
        called cv_image for subsequent processing.
        """

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
        other_robot = []
        other_robot_xy = []
        for contour in contours:
            # Calculate moments for each contour
            moment = cv2.moments(contour)

            contour_image = np.uint8(np.zeros(binary_image.shape))
            cv2.drawContours(contour_image, [contour],-1,1,thickness=cv2.FILLED)

            # Find the average color of the inside of the contour.
            average_color = cv2.mean(self.cv_image, contour_image)
            b, g, r, _ = average_color
            current_robot_id = (round(b/255) * 100) + (round(g/255) * 10) + round(r/255)
            other_robot.append(current_robot_id)

            # calculate x, y coordinate of center
            contour_x = int(moment["m10"] / moment["m00"])
            contour_y = int(moment["m01"] / moment["m00"])
            position = self.calculate_neato_position([contour_x, contour_y, 1])
            other_robot_xy.append(position[0])
            other_robot_xy.append(position[2])
            
            # contour_y = np.max(np.where(np.sum(contour_image > 0,axis=1) > 0))
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

            print(f"Output: {self.calculate_neato_position([contour_x, contour_y, 1])}")
            print()
        self.other_robot = other_robot
        self.other_robot_xy = other_robot_xy

    def calculate_neato_position(self, pixel_coordinates):
        """
        Convert camera pixel coordinates to a 3D vector.
        """
        if self.inv_K is None:
            return None
        
        vector = np.dot(self.inv_K, np.array(pixel_coordinates).T)
        print(f"Pixel Coordinates: {np.array(pixel_coordinates).T}")
        print(f"Vector: {vector}")
        # TODO: Rotate by negative of pitch
        print(f"K-Inv: {self.inv_K}")
        height_camera = 0.25
        height_robot = 0.09
        constant = height_camera / vector[1]
        output = vector * constant
        offset_factor = height_robot / (2 * output[1])
        offset = np.array([offset_factor * output[0],
                           0,
                           offset_factor * output[2]])
        return output - offset

    def get_camera_info(self, msg):
        """
        Get K matrix from a ros message and invert it.
        """
        if self.inv_K is None:
            K = np.array(msg.K)
            
            K = np.reshape(K, (3, 3))
            self.inv_K = np.linalg.inv(K)

    def odom_process(self, msg):
        self.current_odom_x = msg.pose.pose.position.x
        self.current_odom_y = msg.pose.pose.position.y
        orientation_tuple = (msg.pose.pose.orientation.x,
                             msg.pose.pose.orientation.y,
                             msg.pose.pose.orientation.z,
                             msg.pose.pose.orientation.w)
        angles = t.euler_from_quaternion(orientation_tuple)
        self.current_odom_theta = angles[2]
        print("X: " + str(self.current_odom_x))
        print("Y: " + str(self.current_odom_y))

    def run(self):
        """
        The main run loop, in this node it doesn't do anything
        """
        r = rospy.Rate(5)
        current_index = math.floor(rospy.get_time() / 5)
        while not rospy.is_shutdown():
            if not self.cv_image is None:
                # print(self.cv_image.shape)
                cv2.imshow(f'{self.id}_video_window', self.cv_image)
                cv2.waitKey(5)
            if not self.binary_image is None:
                cv2.imshow(f'{self.id}_threshold_image', self.binary_image)
            # start out not issuing any motor commands


            print(f"Time: {math.floor(rospy.get_time() / 5)}")
            new_index = math.floor(rospy.get_time() / 5)
            if current_index < new_index:
                current_index = new_index
                vertex = Vertex(vertex_id=self.id + (current_index * 1000),
                                x=self.current_odom_x,
                                y=self.current_odom_y,
                                theta=self.current_odom_theta,
                                view=[robot_id + (current_index * 1000)
                                      for robot_id in self.other_robot],
                                data=self.other_robot_xy)
                self.vertex_pub.publish(vertex)

            r.sleep()


if __name__ == '__main__':
    redBOT = GetImage('/robot1', 1)
    redBOT.run()

    # yellowBOT = GetImage('/robot2', 11)
    # yellowBOT.run()