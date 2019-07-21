#!/usr/bin/env python
from __future__ import absolute_import, print_function

import rospy
from geometry_msgs.msg import TransformStamped
from zumo import vicon_zumo

KP_LIN = 5
KD_LIN = 0.5
KP_ANG = 10
KP_ANG = 0.1

if __name__ == "__main__":
    # Instantiate zumo driver
    zumo = vicon_zumo.ViconZumo(log_level=rospy.DEBUG)
    zumo.run()
    #
#    while not rospy.is_shutdown():
#        # locate_obj()
        # locate_zumo()
        # compute_cmdv()
