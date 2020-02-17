#!/usr/bin/env python
import rospy
from lab1.msg import balboaLL # import balboa message
from lab1.msg import pid_input # import pid_input message

PI = 3.14159265358979 # global variable for PI

def parse_balboa_msg(data, self):
    self.dist_left = data.distanceLeft # unpack left encoder
    self.dist_Right = data.distanceRight # unpack right encoder
    avg_dist = (self.dist_left+self.dist_Right)/2 # cal avg dist

    # Encoder count per revolution is gear motor ratio (3344/65)
    # times gearbox ratio (2.14/1) times encoder revolution (12/1)
    CPR = (3344 / 65) * 2.14 * 12

    # Distance per revolution is 2 PI times wheel radius (40 mm)
    distPR = 2*PI*40

    # Distance per encoder count is distPR / CPR
    DPC = distPR / CPR

    # convert encoder distance to mm
    avg_dist = avg_dist * DPC

    # Publish the current and target dist values
    self.dist_goal.current = avg_dist
    self.dist_goal.target = self.dist_target
    self.dist.publish(self.dist_goal)

class TheNode(object):
  # This class holds the rospy logic for sending pid_input messages
  # from a published balboa message and TODO: user input 

  def __init__(self):

    rospy.init_node('remote_drive') # intialize node
    
    # initialize publisher node for distance PID controller
    self.dist = rospy.Publisher('/distance', pid_input, queue_size=10)

    self.dist_goal = pid_input() # default pid_input type
    self.dist_target = 50 # travel 50 mm TODO: get from user

  def main_loop(self):
    # initialize subscriber node for messages from balboa robot
    rospy.Subscriber('balboaLL', balboaLL, parse_balboa_msg, self)

    rospy.spin() # wait for messages

if __name__ == '__main__':
    try:
        a = TheNode()
        a.main_loop()
    except rospy.ROSInterruptException:
        pass
