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


class Driver:
  def __init__(self):
    rospy.init_node('driver')

    self._last_received = rospy.get_time()
    self._timeout = rospy.get_param('~timeout', 2)
    self._rate = rospy.get_param('~rate', 10)
    self._max_speed = rospy.get_param('~max_speed', 0.5)
    self._wheel_base = rospy.get_param('~wheel_base', 0.09)

    # Setup subscriber for velocity twist message
    rospy.Subscriber("cmd_vel", Twist, self.velocity_received_callback)

  def velocity_received_callback(self, message):
    """Handle new velocity command message."""

    self._last_received = rospy.get_time()

    # Extract linear and angular velocities from the message
    linear = message.linear.x
    angular = message.angular.z

    # Ideally we'd now use the desired wheel speeds along
    # with data from wheel speed sensors to come up with the
    # power we need to apply to the wheels, but we don't have
    # wheel speed sensors. Instead, we'll simply convert m/s
    # into percent of maximum wheel speed, which gives us a
    # duty cycle that we can apply to each motor.
    self._left_speed = left_speed/self._max_speed * max_robot_speed
    self._right_speed_percent = (100 * right_speed/self._max_speed)

  def run(self):
    """The control loop of the driver."""

    rate = rospy.Rate(self._rate)

    while not rospy.is_shutdown():
      # If we haven't received new commands for a while, we
      # may have lost contact with the commander-- stop
      # moving
      delay = rospy.get_time() - self._last_received
      if delay < self._timeout:
        self._left_motor.move(self._left_speed_percent)
        self._right_motor.move(self._right_speed_percent)
      else:
        self._left_motor.move(0)
        self._right_motor.move(0)

      rate.sleep()
      
def main():
  rospy.init_node("joypad_driver")
  driver = Driver()
  driver.run()
  rospy.spin()
  
if __name__ == "__main__":
  main()
