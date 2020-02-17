#!/usr/bin/env python
import rospy
from lab1.msg import balboaMotorSpeeds # import motor speed message
from std_msgs.msg import Header # import header message
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist # import Twist message
from lab1.msg import balboaLL
from lab1.msg import pid_output
from time import sleep

def parse_msg(data, self):
  sender = data.source
  print type(data.control_effort)
  self.mtrspeed.left = data.control_effort
  self.mtrspeed.right = data.control_effort

  rospy.loginfo(self.mtrspeed)

  # Publish the motor speeds
  # self.pub.publish(self.mtrspeed)
    

class TheNode(object):
  # This class holds the rospy logic for summing the PID outputs and publishing 
  # a motor speed

  def __init__(self):

    rospy.init_node('summation') # intialize node

    
    # initialize publisher node for turtle 1
    self.pub = rospy.Publisher('/motorSpeeds', balboaMotorSpeeds, queue_size=10)
    self.mtrspeed = balboaMotorSpeeds()
    self.mtrspeed.header = Header()
    self.mtrspeed.left = 0
    self.mtrspeed.right = 0


  def main_loop(self):
    # initialize subscriber node for messages from teleop_turtle_key
    rospy.Subscriber('/control', pid_output, parse_msg, self)

    rospy.spin()

if __name__ == '__main__':
    try:
        a = TheNode()
        a.main_loop()
    except rospy.ROSInterruptException:
        pass
