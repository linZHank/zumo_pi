#!/usr/bin/env python

"""
Zumo Robot working under the Vicon MoCap System
"""

from __future__ import absolute_import, print_function
import time
import atexit
import numpy as np
import rospy
from geometry_msgs.msg import Twist, TransformStamped

from .zumo_driver import ZumoDriver


class ViconZumo(ZumoDriver):
    def __init__(self):
        """
        A Vicon guided Zumo class
        """
        # Init tracked pose
        self.obj_transform = TransformStamped().transform
        self.zumo_transform = TransformStamped().transform
    	# Subscribers
    	rospy.Subscriber('vicon/KUKA_TOOL/KUKA_TOOL', TransformStamped, self._obj_trans_callback)
    	rospy.Subscriber('vicon/zumo/zumo', TransformStamped, self._zumo_trans_callback)
        # execute superclass.__init__
        super(ViconZumo, self).__init__(log_level=rospy.INFO)

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
