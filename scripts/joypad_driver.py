#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import print_function

import getch
import rospy
from geometry_msgs.msg import Twist
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import Robot


zumo = Robot.Robot()

class Driver:
  def __init__(self):
    self._last_received = rospy.get_time()
    self._timeout = rospy.get_param('~timeout', 2)
    self._rate = rospy.get_param('~rate', 10)
    self._mode = "S" # stop
    self._max_speed = 255 # max zumo speed
    self._zumo_speed = 0
    # Setup subscriber for velocity twist message
    rospy.Subscriber("cmd_vel", Twist, self.velocity_received_callback)

  def velocity_received_callback(self, message):
    """Handle new velocity command message."""

    self._last_received = rospy.get_time()

    # Extract linear and angular velocities from the message
    linear = message.linear.x
    angular = message.angular.z
    # Determine moving mode by linear and angular velocity
    assert abs(linear<4e-3) or abs(angular<4e-3) # they cannot both be greater than 0
    if linear >= 4e-3:
      self._mode = "F" # forward
      self._zumo_speed = int(linear*self._max_speed)
      if self._zumo_speed > self._max_speed:
        self._zumo_speed = self._max_speed
    elif linear <= -4e-3:
      self._mode = "B" # backward
      self._zumo_speed = int(-linear*self._max_speed)
      if self._zumo_speed > self._max_speed:
        self._zumo_speed = self._max_speed
    elif angular >= 4e-3:
      self._mode = "L" # left turn
      self._zumo_speed = int(angular*self._max_speed)
      if self._zumo_speed > self._max_speed:
        self._zumo_speed = self._max_speed
    elif angular <= -4e-3:
      self._mode = "R" # right turn
      self._zumo_speed = int(-angular*self._max_speed)
      if self._zumo_speed > self._max_speed:
        self._zumo_speed = self._max_speed
    else:
      self._mode = "S"
        
  def run(self):
    """The control loop of the driver."""
    rate = rospy.Rate(self._rate)

    while not rospy.is_shutdown():
      # If we haven't received new commands for a while, we
      # may have lost contact with the commander-- stop
      # moving
      delay = rospy.get_time() - self._last_received
      if delay < self._timeout:
        if self._mode == "F":
          zumo.forward(self._zumo_speed, 1./self._rate)
        elif self._mode == "B":
          zumo.backward(self._zumo_speed, 1./self._rate)
        elif self._mode == "L":
          zumo.left(self._zumo_speed, 1./self._rate)
        elif self._mode == "R":
          zumo.right(self._zumo_speed, 1./self._rate)
        else:
          zumo.stop()
      else:
        zumo.stop()

      rate.sleep()
      
def main():
  rospy.init_node("joypad_driver")
  driver = Driver()
  driver.run()
  rospy.spin()
  
if __name__ == "__main__":
  main()
