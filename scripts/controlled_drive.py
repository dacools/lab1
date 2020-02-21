#!/usr/bin/env python
import rospy
from lab1.msg import balboaLL # import balboa message
from lab1.msg import pid_input # import pid_input message

PI = 3.14159265358979 # global variable for PI

def parse_balboa_msg(data, self):
    # self.dist_left = data.distanceLeft # unpack left encoder
    # self.dist_right = data.distanceRight # unpack right encoder
    self.dist_left = data.encoderCountLeft # unpack left encoder
    self.dist_right = data.encoderCountRight # unpack right encoder
    self.dist_target = rospy.get_param("rCtrl/dist_target") # unpack target dist

    # Encoder count per revolution is gear motor ratio (3344/65)
    # times gearbox ratio (2.14/1) times encoder revolution (12/1)
    CPR = (3344 / 65) * 2.14 * 12

    # Distance per revolution is 2 PI times wheel radius (40 mm)
    distPR = 2*PI*40

    # Distance per encoder count is distPR / CPR
    DPC = distPR / CPR

    # convert encoder distance to mm
    self.dist_left = self.dist_left * DPC
    self.dist_right = self.dist_right * DPC

    # Publish the current and target dist values
    self.dist_pid_input.current_left = self.dist_left
    self.dist_pid_input.current_right = self.dist_right
    self.dist_pid_input.target_left = self.dist_target
    self.dist_pid_input.target_right = self.dist_target
    self.dist.publish(self.dist_pid_input)

class TheNode(object):
  # This class holds the rospy logic for sending pid_input messages
  # from a published balboa message and user input 

  def __init__(self):

    rospy.init_node('remote_drive') # intialize node
    
    # initialize publisher node for distance PID controller
    self.dist = rospy.Publisher('/distance', pid_input, queue_size=10)
    self.ang = rospy.Publisher('/ang', pid_input, queue_size=10)

    self.dist_left = 0 # init left distance
    self.dist_right = 0 # init right distance
    self.dist_pid_input = pid_input() # default pid_input type
    self.dist_target = rospy.get_param("rCtrl/dist_target") # get dist target from user

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
