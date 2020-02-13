#!/usr/bin/env python
import rospy
from lab1.msg import balboaMotorSpeeds # import motor speed message
from std_msgs.msg import Header # import header message
from geometry_msgs.msg import Twist # import Twist message
from lab1.msg import balboaLL
from lab1.msg import pid_input
from time import sleep

def parse_msg(data, self):
    target_distance = 100
    self.dist_left = data.distanceLeft
    self.dist_Right = data.distanceRight
    avg_dist = (self.dist_left+self.dist_Right)/2

    # Publish the current and target values
    self.goal.current = avg_dist
    self.goal.target = target_distance
    self.dist.publish(self.goal)

class TheNode(object):
  # This class holds the rospy logic for sending a motor speed message
  # from a published teleop_turtle_key message

  def __init__(self):

    rospy.init_node('remote_drive') # intialize node

    
    # initialize publisher node for turtle 1
    self.dist = rospy.Publisher('/distance', pid_input, queue_size=10)
    self.goal = pid_input()


  def main_loop(self):
    # initialize subscriber node for messages from teleop_turtle_key
    rospy.Subscriber('balboaLL', balboaLL, parse_msg, self)

    rospy.spin()

if __name__ == '__main__':
    try:
        a = TheNode()
        a.main_loop()
    except rospy.ROSInterruptException:
        pass
