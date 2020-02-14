#!/usr/bin/env python
import rospy
from lab1.msg import pid_input # import pid_input message
from lab1.msg import pid_output # import pid_output message

def controller(data, self):
  # get most recent rosparam values
  self.P = rospy.get_param("rCtrl/P")
  self.I = rospy.get_param("rCtrl/I")
  self.D = rospy.get_param("rCtrl/D")

  curr = data.current # unpack current
  tar = data.target # unpack target

  # err is (target - current)
  err = tar - curr

  # err_P term is (P*err)
  err_P = self.P*err

  # err_I term is (I*(err_sum + err)
  err_I = self.I*(self.err_sum + err)

  # err_D term is (D*(err - err_last))
  err_D = self.D*(err - self.err_last)

  self.output.source = 'distance' # set source TODO: make rosparam value
  self.output.control_effort = err_P + err_I + err_D # set control effort
  self.target.publish(self.output) # publish output msg

  rospy.loginfo(self.output) # debug

  # err_last is (previous_target - previous_current)
  self.e_last = err
  self.err_sum = self.err_sum + err

class TheNode(object):
  # This class holds the rospy logic for a generic PID controller

  def __init__(self):

    rospy.init_node('pid', anonymous=True) # intialize node

    # initialize publisher node for generic PID controller
    self.target = rospy.Publisher('/control', pid_output, queue_size=10)

    self.output = pid_output()  # default pid_output type
    self.err_last = 0 # init derivative term
    self.err_sum = 0 # init integral term

  def main_loop(self):
    # initialize subscriber node for messages from a generic source
    rospy.Subscriber('/subscribe', pid_input, controller, self)

    # get initial rosparam values
    self.P = rospy.get_param("rCtrl/P")
    self.I = rospy.get_param("rCtrl/I")
    self.D = rospy.get_param("rCtrl/D")

    rospy.spin() # wait for messages

if __name__ == '__main__':
    try:
        a = TheNode()
        a.main_loop()
    except rospy.ROSInterruptException:
        pass
