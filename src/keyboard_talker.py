#!/usr/bin/env python



def kb_talker():
  """Keyboard talker publishes keyboard input
  """
  pub = rospy.Publisher("keyboard_command", String, queue_size=10)
  rospy.init_node("keypress", annoymous=True)
  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    key = ord(getch.getch()) # convert whatever key pressed (keyboard, joypad, joystick...) to an ord value
    

if __name__=="main":
  try:
    kb_talker()
  except rospy.ROSInterruptException:
    pass
