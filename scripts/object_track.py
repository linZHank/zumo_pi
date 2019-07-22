#!/usr/bin/env python

"""
Pre-requisite:
    ```
    roslaunch vicon_bridge vicon.launch
    roslaunch zumo_pi zumo_driver.launch
    ```
"""

from __future__ import absolute_import, print_function

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist, TransformStamped
# from zumo import vicon_zumo

def transform_to_pose(transform):
    position = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
    orientation = np.array([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])

    return position, orientation

KP_LIN = 5
KD_LIN = 0.5
KP_ANG = 10
KD_ANG = 0.1

class ViconBridge(object):
    def __init__(self, log_level=rospy.DEBUG):
        """
        A vicon_bridge class
        """
        rospy.init_node("vicon_tracker", anonymous=True, log_level=log_level)
        self._rate = 10
        # Init tracked pose
        self.obj_transform = TransformStamped().transform
        self.zumo_transform = TransformStamped().transform
    	# Subscribers
    	rospy.Subscriber('vicon/KUKA_TOOL/KUKA_TOOL', TransformStamped, self._obj_trans_callback)
    	rospy.Subscriber('vicon/zumo/zumo', TransformStamped, self._zumo_trans_callback)
        rospy.logdebug("Vicon tracker initiated.")
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

if __name__ == "__main__":
    # Instantiate zumo driver
    tracker = ViconBridge()
    print(tracker.obj_transform)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    # unit vectors
    vec_x = np.array([1, 0])
    vec_y = np.array([0, 1])
    # Init errors
    err_lin = 0
    del_err_lin = 0
    err_ang = 0
    del_err_ang = 0
    err_lin_pre = 0
    err_ang_pre = 0
    # Specify publishing rate
    rate = rospy.Rate(tracker._rate)
    while not rospy.is_shutdown():
        # Compute errors
        obj_pos, _ = transform_to_pose(tracker.obj_transform)
        zumo_pos, zumo_orn = transform_to_pose(tracker.zumo_transform)
        err_lin = np.linalg.norm(obj_pos[:2] - zumo_pos[:2])
        ang_z2g = np.arctan2(obj_pos[1]-zumo_pos[1], obj_pos[0]-zumo_pos[0])
        err_ang = ang_z2g - tf.transformations.euler_from_quaternion(zumo_orn)[2]
        if err_ang > np.pi:
            err_ang -= np.pi*2
        elif err_ang < -np.pi:
            err_ang += np.pi*2
        del_err_lin = err_lin - err_lin_pre
        del_err_ang = err_ang - err_ang_pre
        cmd_vel = Twist()
        v_lin = KP_LIN*err_lin + KD_LIN*del_err_lin
        v_ang = KP_ANG*err_ang + KD_ANG*del_err_ang
        if err_lin < 0.4:
            v_lin = 0
            v_ang = 0
        cmd_vel.linear.x = v_lin
        cmd_vel.angular.z = v_ang
        cmd_vel_pub.publish(cmd_vel)
        rospy.loginfo("cmd_vel: {}".format(cmd_vel))
        err_lin_pre = err_lin
        err_ang_pre = err_ang

        rate.sleep()
