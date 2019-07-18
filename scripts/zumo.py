#!/usr/bin/env python

"""
Two DC motor differential drive robot class.
moving a robot forward, backward, and turning.
Thanks to Tony DiCola's work in Adafruit_MotorHAT
License: MIT License https://opensource.org/licenses/MIT
"""

from __future__ import absolute_import, print_function
import time
import atexit
import rospy
from geometry_msgs.msg import Twist

from Adafruit_MotorHAT import Adafruit_MotorHAT


class Zumo(object):
    def __init__(self, addr=0x60, left_id=4, right_id=1, left_trim=0, right_trim=0,
                 stop_at_exit=True):
        """Create an instance of the robot.  Can specify the following optional
        parameters:
         - addr: The I2C address of the motor HAT, default is 0x60.
         - left_id: The ID of the left motor, default is 1.
         - right_id: The ID of the right motor, default is 2.
         - left_trim: Amount to offset the speed of the left motor, can be positive
                      or negative and use useful for matching the speed of both
                      motors.  Default is 0.
         - right_trim: Amount to offset the speed of the right motor (see above).
         - stop_at_exit: Boolean to indicate if the motors should stop on program
                         exit.  Default is True (highly recommended to keep this
                         value to prevent damage to the bot on program crash!).
        """
        rospy.init_node('zumo', anonymous=True, log_level=rospy.DEBUG)
        # Initialize motor HAT and left, right motor.
        self._mh = Adafruit_MotorHAT(addr)
        self._left_motor = self._mh.getMotor(left_id)
        self._right_motor = self._mh.getMotor(right_id)
        self._left_trim = left_trim
        self._right_trim = right_trim
        self._left_motor.run(Adafruit_MotorHAT.RELEASE)
        self._right_motor.run(Adafruit_MotorHAT.RELEASE) # start with motors turned off
        # ROS params
        self._last_received = rospy.get_time()
        self._timeout = rospy.get_param('~timeout', 5)
        self._rate = rospy.get_param('~rate', 10)
        self._max_speed = rospy.get_param('~max_speed', 0.5)
        self._wheel_base = rospy.get_param('~wheel_base', 0.086)
    	# Subscribe to /cmd_vel
    	rospy.Subscriber("cmd_vel", Twist, self._cmd_vel_callback)
        # Configure all motors to stop at program exit if desired.
        if stop_at_exit:
            atexit.register(self.stop)

    def _cmd_vel_callback(self, data):
        """
        Callback function for Handling received data from '/cmd_vel' topic
        """
        self._last_received = rospy.get_time()
        # Extract linear and angular velocities from the message
        linear = data.linear.x
        angular = data.angular.z
        # Calculate wheel speeds in m/s
        left_speed = linear - angular*self._wheel_base/2
        right_speed = linear + angular*self._wheel_base/2
        # Compute motor speed in scale of [0,255]
        self._left_speed_scale = (255 * left_speed/self._max_speed)
        self._right_speed_scale = (255 * right_speed/self._max_speed)
        assert -255 <= self._left_speed_scale <= 255
        assert -255 <= self._right_speed_scale <= 255

    def _left_speed(self, speed):
        """Set the speed of the left motor, taking into account its trim offset.
        """
        assert 0 <= speed <= 255, 'Speed must be a value between 0 to 255 inclusive!'
        speed += self._left_trim
        speed = max(0, min(255, speed))  # Constrain speed to 0-255 after trimming.
        self._left_motor.setSpeed(speed)

    def _right_speed(self, speed):
        """Set the speed of the right motor, taking into account its trim offset.
        """
        assert 0 <= speed <= 255, 'Speed must be a value between 0 to 255 inclusive!'
        speed += self._right_trim
        speed = max(0, min(255, speed))  # Constrain speed to 0-255 after trimming.
        self._right_motor.setSpeed(speed)

    def run(self):
        """
        Control loop of driving Zumo
        """
        rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            # Stop moving if no received new commands for a while
            delay = rospy.get_time() - self._last_received
            if delay < self._timeout:
                if self._left_speed_scale >= 0: # set left motor speed and dir
                    self._left_motor.setSpeed(self._left_speed_scale)
                    self._left_motor.run(Adafruit_MotorHAT.FORWARD)
                else:
                    self._left_motor.setSpeed(math.abs(self._left_speed_scale))
                    self._left_motor.run(Adafruit_MotorHAT.BACKWARD)
                if self._right_speed_scale >= 0: # set right motor speed and dir
                    self._right_motor.setSpeed(self._right_speed_scale)
                    self._right_motor.run(Adafruit_MotorHAT.FORWARD)
                else:
                    self._right_motor.setSpeed(math.abs(self._right_speed_scale))
                    self._right_motor.run(Adafruit_MotorHAT.BACKWARD)
            else:
                self._left_motor.run(Adafruit_MotorHAT.RELEASE)
                self._right_motor.run(Adafruit_MotorHAT.RELEASE)
            rate.sleep()

    def stop(self):
        """Stop all movement."""
        self._left.run(Adafruit_MotorHAT.RELEASE)
        self._right.run(Adafruit_MotorHAT.RELEASE)
