#!/usr/bin/env python
import rospy
# from lab1.msg import balboaMotorSpeeds # import motor speed message
from std_msgs.msg import Header # import header message
from lab1.msg import pid_output # import pid_output message
from lab1.msg import pid_input # import pid_output message

def parse_ang_control_msg(data, self):
  pass


def parse_ang_msg(data, self):
  pass




class TheNode(object):
  # This class holds the rospy logic for summing the PID outputs and publishing 
  # a motor speed message

  def __init__(self):

    rospy.init_node('ang_to_ang_vel') # intialize node
    
    # initialize publisher node for motor speeds
    self.pub = rospy.Publisher('/ang_vel', pid_input, queue_size=10)

  def main_loop(self):
    # initialize subscriber node for messages from a pid controller
    rospy.Subscriber('/ang', pid_input, parse_ang_msg, self)
    rospy.Subscriber('/ang_control', pid_output, parse_ang_control_msg, self)

    rospy.spin() # wait for messages

if __name__ == '__main__':
    try:
        a = TheNode()
        a.main_loop()
    except rospy.ROSInterruptException:
        pass
