#!/usr/bin/env python
import rospy
from lab1.msg import pid_input
from lab1.msg import pid_output

def controller(data, self):
  curr = data.current
  tar = data.target
  # Define the error turm e as (target - current)
  e = tar-curr

  # P term is P*(target-current)
  e_P = self.P*e

  # D term is D*((target-current)-(previous_target-previous_current))
  e_D = self.D*(e-self.e_1)

  # I term is I*(previous_sum + (target-current))
  e_I = self.I*(self.integral_1+e)

  self.out.source = 'distance'
  self.out.control_effort = e_P + e_I + e_D
  self.target.publish(self.out)
  rospy.loginfo(self.out)

  self.e_1 = e
  self.integral_1= self.integral_1+e

class TheNode(object):
  # This class holds the rospy logic for a generic PID controller

  def __init__(self):
    rospy.init_node('pid', anonymous=True) # intialize node

    self.target = rospy.Publisher('/control', pid_output, queue_size=10)
    self.out = pid_output()
    self.e_1 = 0
    self.integral_1 = 0

  def main_loop(self):
    rate = rospy.Rate(10) # 10 hz refresh rate
    rospy.Subscriber('/subscribe', pid_input, controller, self)

    #P = 1.0
    #I = 2.0
    #D = 3.0
    #rospy.set_param("rCtrl/P",P)
    #rospy.set_param("rCtrl/I",I)
    #rospy.set_param("rCtrl/D",D)

    self.P = rospy.get_param("rCtrl/P")
    self.I = rospy.get_param("rCtrl/I")
    self.D = rospy.get_param("rCtrl/D")
    rospy.spin()

    while not rospy.is_shutdown():
      #rospy.loginfo(rospy.get_param('rCtrl'))
      rospy.loginfo('P: {0} I: {1} D: {2}'.format(self.P, self.I, self.D))
      rate.sleep()

if __name__ == '__main__':
    try:
        a = TheNode()
        a.main_loop()
    except rospy.ROSInterruptException:
        pass
