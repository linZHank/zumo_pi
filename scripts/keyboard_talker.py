#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import print_function

import getch
import rospy
from std_msgs.msg import String

def kb_talker():
  """Keyboard talker publishes keyboard input
  """
  pub = rospy.Publisher("keyboard_command", String, queue_size=10)
  rospy.init_node("keypress", anonymous=True)
  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    cmd = getch.getch() # cmd will be a unicode

    # publish only 'a', 's', 'd', 'w'
    if cmd == 'a':
      rospy.loginfo("{}: Turning left <".format(cmd))
    elif cmd == 's':
      rospy.loginfo("{}: Moving backward v".format(cmd))
    elif cmd == 'd':
      rospy.loginfo("{}: Turning right >".format(cmd))
    elif cmd == 'w':
      rospy.loginfo("{}: Moving forward ^".format(cmd))
    else:
      pass
    pub.publish(cmd)
    rate.sleep()

if __name__=='__main__':
  try:
    kb_talker()
  except rospy.ROSInterruptException:
    pass
