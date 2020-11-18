#!/usr/bin/env python3

"""
Testing getting two robots in Gazebo
"""

import rospy


class SwarmBot:

    def __init__(self):


        # set up subscribers
        rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.update_initial_pose)
        rospy.Subscriber('/scan', LaserScan, self.scan_process)


        # set up publishers
