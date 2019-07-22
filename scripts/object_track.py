#!/usr/bin/env python
from __future__ import absolute_import, print_function

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist, TransformStamped
from zumo import vicon_zumo

def transform_to_pose(transform):
    position = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
    orientation = np.array([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])

    return position, orientation

KP_LIN = 5
KD_LIN = 0.5
KP_ANG = 10
KP_ANG = 0.1

if __name__ == "__main__":
    # Instantiate zumo driver
    zumo = vicon_zumo.ViconZumo()
    zumo.run()
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    # unit vectors
    vec_x = np.array([1, 0])
    vec_y = np.array([0, 1])
    # init errors
    obj_pos, _ = transform_to_pose(zumo.obj_transform)
    zumo_pos, zumo_orn = transform_to_pose(zumo.zumo_transform)
    print("obj_pos: {} \nzumo_pos: {}, zumo_orn: {}".format(obj_pos, zumo_pos, zumo_orn))
    err_lin = np.linalg.norm(obj_pos[:2] - zumo_pos[:2])
    del_err_lin = 0
    ang_z2g = np.arctan2(obj_pos[1]-zumo_pos[1], obj_pos[0]-zumo_pos[0])
    err_ang = ang_z2g - tf.transformations.euler_from_quaternion(zumo_orn)
    if err_ang > np.pi:
        err_ang -= np.pi*2
    elif err_ang < -np.pi:
        err_ang += np.pi*2
    del_err_ang = 0
    rate = rospy.Rate(zumo._rate)
    while not rospy.is_shutdown():
        cmd_vel = Twist()
        v_lin = KP_LIN*err_lin + KD_LIN*del_err_lin
        v_ang = KP_ANG*err_ang + KD_ANG*del_err_ang
        cmd_vel.linear.x = v_lin
        cmd_vel.angular.z = v_ang
        cmd_vel_pub.publish(cmd_vel)
        rospy.loginfo("cmd_vel: {}".format(cmd_vel))
        rate.sleep()
