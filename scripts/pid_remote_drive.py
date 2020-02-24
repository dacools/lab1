#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist # import Twist message

def multiplier_msg(data,self):
    self.distance_multiplier = data.linear.y
    self.angle_multiplier = data.angular.y

def turtle_msg(data, self):
    self.x = data.linear.x # set linear component
    self.z = data.angular.z # set angular component

    dist = 100*self.distance_multiplier # set a distance offset
    ang = 15*self.angle_multiplier # set the angle offest

    # retrieve the target parameters
    self.left = rospy.get_param("distance/tar/left")
    self.right = rospy.get_param("distance/tar/right")
    self.angle = rospy.get_param("angle/target")

    if self.x > 0:
        # move forward
        self.left = self.left + dist
        self.right = self.right + dist
    elif self.x < 0:
        # move backward
        self.left = self.left - dist
        self.right = self.right - dist
    elif self.z > 0:
        # turn left
        self.angle = self.angle - ang
    elif self.z < 0:
        # turn right
        self.angle = self.angle + ang
    else:
        rospy.loginfo("Parse error")

    rospy.set_param("distance/tar/left",self.left)
    rospy.set_param("distance/tar/right",self.right)
    rospy.set_param("angle/target",self.angle)

class TheNode(object):
  # This class holds the rospy logic for updating the PID targets
  # from a published teleop_turtle_key message or keyboard input

  def __init__(self):

    rospy.init_node('pid_remote_drive') # intialize node

    self.x = 0 # linear component variable
    self.z = 0 # angular component variable

    self.left = 0 # left distance variable
    self.right = 0 # right distance variable
    self.angle = 0 # angle variable

    self.angle_multiplier = 1 # angle multiplier variable
    self.distance_multiplier = 1 # distance multiplier variable

  def main_loop(self):
    # initialize subscriber node for messages from teleop_turtle_key
    rospy.Subscriber('/turtle1/cmd_vel', Twist, turtle_msg, self)
    rospy.Subscriber('/key_input', Twist, multiplier_msg, self)

    rospy.spin() # wait for messages

if __name__ == '__main__':
    try:
        a = TheNode()
        a.main_loop()
    except rospy.ROSInterruptException:
        pass
