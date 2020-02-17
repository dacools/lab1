#!/usr/bin/env python
import rospy
from lab1.msg import pid_input # import pid_input message
from lab1.msg import pid_output # import pid_output message

def controller(data, self):
  # get most recent rosparam values
  self.P = rospy.get_param("rCtrl/P")
  self.I = rospy.get_param("rCtrl/I")
  self.D = rospy.get_param("rCtrl/D")

  curr_l = data.current_left # unpack current left
  curr_r = data.current_right # unpack current right
  tar_l = data.target_left # unpack target left
  tar_r = data.target_right # unpack target right

  # err is (target - current)
  err_l = tar_l - curr_l
  err_r = tar_r - curr_r

  # err_P term is (P*err)
  err_P_l = self.P*err_l
  err_P_r = self.P*err_r

  # err_I term is (I*(err_sum + err)
  err_I_l = self.I*(self.err_sum_l + err_l)
  err_I_r = self.I*(self.err_sum_r + err_r)

  # err_D term is (D*(err - err_last))
  err_D_l = self.D*(err_l - self.err_last_l)
  err_D_r = self.D*(err_r - self.err_last_r)

<<<<<<< HEAD
  self.output.source = 'distance' # set source TODO: make rosparam value
  self.output.control_left = err_P_l + err_I_l + err_D_l # set left control effort
  self.output.control_right = err_P_r + err_I_r + err_D_r # set right control effort
=======
  self.output.source = self.src # set source TODO: make rosparam value
  self.output.control_effort = err_P + err_I + err_D # set control effort
>>>>>>> riley_branch
  self.target.publish(self.output) # publish output msg

  rospy.loginfo(self.output) # debug

  # err_last is (previous_target - previous_current)
  self.err_last_l = err_l
  self.err_last_r = err_r

  # err_sum is accumulated err plus current err
  self.err_sum_l = self.err_sum_l + err_l
  self.err_sum_r = self.err_sum_r + err_r

class TheNode(object):
  # This class holds the rospy logic for a generic PID controller

  def __init__(self):

    rospy.init_node('pid', anonymous=True) # intialize node

    # initialize publisher node for generic PID controller
    self.target = rospy.Publisher('/control', pid_output, queue_size=10)

    self.output = pid_output()  # default pid_output type
    self.err_last_l = 0 # init left derivative term
    self.err_last_r = 0 # init right derivative term
    self.err_sum_l = 0 # init left integral term
    self.err_sum_r = 0 # init right integral term

  def main_loop(self):
    # initialize subscriber node for messages from a generic source 
    rospy.Subscriber('/subscribe', pid_input, controller, self)

    # get initial rosparam values
    self.P = rospy.get_param("rCtrl/P")
    self.I = rospy.get_param("rCtrl/I")
    self.D = rospy.get_param("rCtrl/D")
    self.src = rospy.get_param("rCtrl/src")

    rospy.spin() # wait for messages

if __name__ == '__main__':
    try:
        a = TheNode()
        a.main_loop()
    except rospy.ROSInterruptException:
        pass
