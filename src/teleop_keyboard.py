#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import print_function

import getch
import rospy
from std_msgs.msg import String
import Robot


def callback(data, zumo):
  if data.data == 'a':
    rospy.loginfo("{}: Turning left <".format(data.data))
    zumo.left(128, 0.1)
  elif data.data == 's':
    rospy.loginfo("{}: Moving backward v".format(data.data))
    zumo.backward(64, 0.1)
  elif data.data == 'd':
    rospy.loginfo("{}: Turning right >".format(data.data))
    zumo.right(128, 0.1)
  elif data.data == 'w':
    rospy.loginfo("{}: Moving forward ^".format(data.data))
    zumo.forward(128, 0.1)
  else:
    print("waiting for command ...")
    pass


def executor():
  rospy.Subscriber("keyboard_command", String, callback(zumo))  
    
if __name__ == "__main__":
  rospy.init_node("key_command_execute", anonymous=True)
  zumo = Robot().Robot(left_trim=LEFT_TRIM, right_trim=RIGHT_TRIM)
  executor()
  rospy.spin()
  
