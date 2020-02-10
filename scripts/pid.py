#!/usr/bin/env python
import rospy

class TheNode(object):
  # This class holds the rospy logic for a generic PID controller

  def __init__(self):
    rospy.init_node('pid', anonymous=True) # intialize node

  def main_loop(self):
    rate = rospy.Rate(1) # 1 hz refresh rate

    P = 1.0
    I = 2.0
    D = 3.0
    rospy.set_param("rCtrl/P",P)
    rospy.set_param("rCtrl/I",I)
    rospy.set_param("rCtrl/D",D)

    while not rospy.is_shutdown():
      rospy.loginfo(rospy.get_param('rCtrl'))
      rate.sleep()

if __name__ == '__main__':
    try:
        a = TheNode()
        a.main_loop()
    except rospy.ROSInterruptException:
        pass
