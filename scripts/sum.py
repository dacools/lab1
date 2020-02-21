#!/usr/bin/env python
import rospy
from lab1.msg import balboaMotorSpeeds # import motor speed message
from std_msgs.msg import Header # import header message
from lab1.msg import pid_output # import pid_output message

def parse_dist_msg(data, self):
  self.sender = data.source # unpack sender
  self.mtrspeed.left = data.control_left # unpack left control effort
  self.mtrspeed.right = data.control_right # unpack right control effort

  # Publish the motor speeds
  self.pub.publish(self.mtrspeed)

def parse_ang_vel_msg(data, self):
  rospy.loginfo(data)

class TheNode(object):
  # This class holds the rospy logic for summing the PID outputs and publishing 
  # a motor speed message

  def __init__(self):

    rospy.init_node('summation') # intialize node
    
    # initialize publisher node for motor speeds
    self.pub = rospy.Publisher('/motorSpeeds', balboaMotorSpeeds, queue_size=10)

    self.mtrspeed = balboaMotorSpeeds() # default motor speed msg type
    self.mtrspeed.header = Header() # default header type
    self.mtrspeed.left = 0 # init left speed
    self.mtrspeed.right = 0 # init right speed
    self.sender = '' # init sender name

  def main_loop(self):
    # initialize subscriber node for messages from a pid controller
    rospy.Subscriber('/ang_vel_control', pid_output, parse_ang_vel_msg, self)
    rospy.Subscriber('/dist_control', pid_output, parse_dist_msg, self)

    rospy.spin() # wait for messages

if __name__ == '__main__':
    try:
        a = TheNode()
        a.main_loop()
    except rospy.ROSInterruptException:
        pass
