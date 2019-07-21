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
import numpy as np
import rospy
from geometry_msgs.msg import Twist, TransformStamped

from Adafruit_MotorHAT import Adafruit_MotorHAT


class ZumoDriver(object):
    def __init__(self, addr=0x60, left_id=4, right_id=1, left_trim=0, right_trim=0,
                 stop_at_exit=True, log_level=rospy.INFO):
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
        rospy.init_node('zumo_driver', anonymous=True, log_level=log_level)
        rospy.loginfo("Initiate zumo_driver node.")
        # Initialize motor HAT and left, right motor.
        self.mh = Adafruit_MotorHAT(addr)
        self.left_motor = self.mh.getMotor(left_id)
        self.right_motor = self.mh.getMotor(right_id)
        self.left_trim = left_trim
        self.right_trim = right_trim
        self.left_speed_scale = 0
        self.right_speed_scale = 0
        self.left_motor.run(Adafruit_MotorHAT.RELEASE)
        self.right_motor.run(Adafruit_MotorHAT.RELEASE) # start with motors turned off
        # ROS params
        self.last_received = rospy.get_time()
        self.timeout = rospy.get_param('~timeout', 5)
        self.rate = rospy.get_param('~rate', 10)
        self.max_speed = rospy.get_param('~max_speed', 0.5)
        self.wheel_base = rospy.get_param('~wheel_base', 0.086)
    	# Subscribe to /cmd_vel
    	rospy.Subscriber('cmd_vel', Twist, self._cmd_vel_callback)
        # Configure all motors to stop at program exit if desired.
        if stop_at_exit:
            atexit.register(self.stop)

    def cmd_vel_callback(self, data):
        """
        Callback function for Handling received data from '/cmd_vel' topic
        """
        self.last_received = rospy.get_time()
        # Extract linear and angular velocities from the message
        linear = data.linear.x
        angular = data.angular.z
        rospy.logdebug("cmd_vel: {}".format(data))
        # Calculate wheel speeds in m/s
        left_speed = linear - angular*self.wheel_base/2
        right_speed = linear + angular*self.wheel_base/2
        # Compute motor speed in scale of [0,255]
        self.left_speed_scale = np.clip(int(255 * left_speed/self.max_speed), -255, 255)
        self.right_speed_scale = np.clip(int(255 * right_speed/self.max_speed), -255, 255)
        assert -255 <= self.left_speed_scale <= 255
        assert -255 <= self.right_speed_scale <= 255

    def run(self):
        """
        Control loop of driving Zumo
        """
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # Stop moving if no received new commands for a while
            delay = rospy.get_time() - self.last_received
            if delay < self.timeout:
                if self.left_speed_scale >= 0: # set left motor speed and dir
                    self.left_motor.setSpeed(self.left_speed_scale)
                    self.left_motor.run(Adafruit_MotorHAT.FORWARD)
                    rospy.logdebug("Left motor foward: {}".format(self.left_speed_scale))
                else:
                    self.left_motor.setSpeed(np.abs(self.left_speed_scale))
                    self.left_motor.run(Adafruit_MotorHAT.BACKWARD)
                    rospy.logdebug("Left motor backward: {}".format(self.left_speed_scale))
                if self.right_speed_scale >= 0: # set right motor speed and dir
                    self.right_motor.setSpeed(self.right_speed_scale)
                    self.right_motor.run(Adafruit_MotorHAT.FORWARD)
                    rospy.logdebug("Right motor foward: {}".format(self.right_speed_scale))
                else:
                    self.right_motor.setSpeed(np.abs(self.right_speed_scale))
                    self.right_motor.run(Adafruit_MotorHAT.BACKWARD)
                    rospy.logdebug("Right motor backward: {}".format(self.right_speed_scale))
            else:
                self.left_motor.run(Adafruit_MotorHAT.RELEASE)
                self.right_motor.run(Adafruit_MotorHAT.RELEASE)
            rate.sleep()

    def stop(self):
        """Stop all movement."""
        self.left_motor.run(Adafruit_MotorHAT.RELEASE)
        self.right_motor.run(Adafruit_MotorHAT.RELEASE)
