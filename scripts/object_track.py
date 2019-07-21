#!/usr/bin/env python
from __future__ import absolute_import, print_function

import rospy
from geometry_msgs.msg import TransformStamped
from zumo import zumo_driver

class ViconZumo(zumo_driver.ZumoDriver):
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

KP_LIN = 5
KD_LIN = 0.5
KP_ANG = 10
KP_ANG = 0.1

if __name__ == "__main__":
    # Instantiate zumo driver
    zumo = ViconZumo()
    zumo.run()
    #
#    while not rospy.is_shutdown():
#        # locate_obj()
        # locate_zumo()
        # compute_cmdv()
