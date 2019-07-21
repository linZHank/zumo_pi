#!/usr/bin/env python

"""
Two DC motor differential drive robot class.
moving a robot forward, backward, and turning.
Thanks to Tony DiCola's work in Adafruit_MotorHAT
License: MIT License https://opensource.org/licenses/MIT
"""

from __future__ import absolute_import, print_function
import time
import atexit
import numpy as np
import rospy
from geometry_msgs.msg import Twist, TransformStamped

from Adafruit_MotorHAT import Adafruit_MotorHAT


class ViconBridge(object):
    def __init__(self, log_level=rospy.INFO):
        """
        A vicon_bridge class
        """
        rospy.init_node('vicon_bridge', anonymous=True, log_level=rospy.DEBUG)
        # Init tracked pose
        self.obj_transform = TransformStamped().transform
        self.zumo_transform = TransformStamped().transform
    	# Subscribers
    	rospy.Subscriber('vicon/KUKA_TOOL/KUKA_TOOL', TransformStamped, self._obj_trans_callback)
    	rospy.Subscriber('vicon/zumo/zumo', TransformStamped, self._zumo_trans_callback)

    def _obj_trans_callback(self, data):
        """
        Callback function for data from object transformation topic
        """
        self.obj_transform = data.transform
        rospy.logdebug("Object at: {}".format(self.obj_transform))

    def _zumo_trans_callback(self, data):
        """
        Callback function for data from zumo transformation topic
        """
        self.zumo_transform = data.transform
        rospy.logdebug("Zumo at: {}".format(self.zumo_transform))
