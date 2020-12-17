#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import pickle
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from swarm_localization.msg import Vertex

class VertexGroup():
    """
    The VertexGroup is a Python object that encompasses a ROS node.
    It listens to a topic /vertex and collects them in IndividualVertex objects.
    Once the ROS node is killed through the terminal it will write all data to a pickle object.
    """
    def __init__(self):
        rospy.init_node('vertex_collect')
        rospy.Subscriber("/vertex",
                         Vertex,
                         self.add_vertex)

        self.wait_list = {}
        self.verticies = {}
        self.vertex_index = 0

    def add_vertex(self, msg):
        print(f"Recieve {self.vertex_index}")
        new_vertex = IndividualVertex(msg.vertex_id, self.vertex_index, msg.x, msg.y, msg.theta)
        msg.data += (1111, 1111)
        print(msg.data)
        verticies = self.wait_list.get(msg.vertex_id, None)
        if verticies is not None:
            for vertex_id in verticies:
                self.verticies[vertex_id].fufill(msg.vertex_id, self.vertex_index, msg.x, msg.y)
        
        for index, vertex_id in enumerate(msg.view + (msg.vertex_id + 1000,)):
            try:
                vertex = self.verticies[vertex_id]
            except KeyError:
                reliant_vertecies = self.wait_list.get(vertex_id, [])
                reliant_vertecies.append(msg.vertex_id)
                self.wait_list[vertex_id] = reliant_vertecies
                new_vertex.disapoint(vertex_id, (msg.data[index * 2], msg.data[(index * 2) + 1]))
                vertex = None
            if vertex is not None:
                new_vertex.add_edge(vertex.index, (msg.data[index], msg.data[index + 1]))

        self.verticies[msg.vertex_id] = new_vertex
        self.vertex_index += 1

class IndividualVertex():
    def __init__(self, vertex_id, vertex_index, x, y, theta):
        self.index = vertex_index
        self.id = vertex_id
        self.x = x
        self.y = y
        self.theta = theta
        self.edge = {}
        self.wait_list = {}

    def add_edge(self, other_index, relative_location):
        self.edge[other_index] = relative_location

    def fufill(self, other_id, other_index, other_x, other_y):
        relative_location = self.wait_list[other_id]
        print(f"Fufill: {other_index}")
        if relative_location == (1111, 1111):
            transformation_matrix = \
                np.linalg.inv(np.array([[math.cos(self.theta), -math.sin(self.theta), self.x],
                                        [math.sin(self.theta), math.cos(self.theta), self.y],
                                        [0, 0, 1]]))
            location_vector = np.array([other_x, other_y, 1]).T
            relative_location_vector = np.matmul(transformation_matrix, location_vector)
            relative_location = (relative_location_vector[0], relative_location_vector[1])
        self.add_edge(other_index, relative_location)

    def disapoint(self, other_id, relative_location):
        self.wait_list[other_id] = relative_location

if __name__ == "__main__":
    vertex_group = VertexGroup()
    # test_pub = rospy.Publisher("/vertex", Vertex, queue_size=10)
    # data_1 = Vertex(vertex_id=1111,
    #                 x=0,
    #                 y=0,
    #                 theta=0,
    #                 view=[1011],
    #                 data=[1, 1])
    # data_2 = Vertex(vertex_id=1011,
    #                 x=0,
    #                 y=0,
    #                 theta=0,
    #                 view=[],
    #                 data=[])
    # data_3 = Vertex(vertex_id=2111,
    #                 x=1,
    #                 y=0,
    #                 theta=0,
    #                 view=[],
    #                 data=[])
    # r = rospy.Rate(5)
    # r.sleep()
    # test_pub.publish(data_1)
    # r.sleep()
    # test_pub.publish(data_2)
    # r.sleep()
    # test_pub.publish(data_3)
    # r.sleep()
    # print(vertex_group.verticies[1111].edge)
    rospy.spin()
    with open("test.pickle", "wb") as f:
        pickle.dump(vertex_group.verticies,f )