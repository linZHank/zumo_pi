#!/usr/bin/env python
from __future__ import absolute_import, print_function

import rospy
from zumo import zumo_driver

zumo = zumo_driver.ZumoDriver()

if __name__ == "__main__":
    zumo.run()
