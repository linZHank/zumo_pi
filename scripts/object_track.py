#!/usr/bin/env python
from __future__ import absolute_import, print_function

import rospy
from zumo import zumo_driver, vicon_bridge

KP_LIN = 5
KD_LIN = 0.5
KP_ANG = 10
KP_ANG = 0.1

if __name__ == "__main__":
    # Instantiate zumo driver
    zumo = zumo_driver.ZumoDriver()
    zumo.run()
    vicon = vicon_bridge.ViconBridge()
    #
    while not rospy.is_shutdown():
        rospy.Subscriber('Vicon')
        locate_obj()
        locate_zumo()
        compute_cmdv()
