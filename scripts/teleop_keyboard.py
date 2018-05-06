#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import print_function

import getch
import rospy
from std_msgs.msg import String
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import Robot

zumo = Robot.Robot()

def callback(data):
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
    print("...")
    pass

def executor():
  rospy.Subscriber("keyboard_command", String, callback)
  
def main():
  rospy.init_node("zumo_teleop")
  executor()
  rospy.spin()
  
if __name__ == "__main__":
  main()
