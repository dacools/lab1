#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist # import Twist message

def multiplier_msg(data,self):
    self.distance_multiplier = data.linear.y
    self.angle_multiplier = data.angular.y

def turtle_msg(data, self):
    self.x = data.linear.x # set linear component
    self.z = data.angular.z # set angular component

    distance = 100.0*self.distance_multiplier #set a distance offset
    ang = 15.0*self.angle_multiplier #set the angle offest

    # retrieve the target parameters
    dist_target_left = rospy.get_param("distance/tar/left")
    dist_target_right = rospy.get_param("distance/tar/right")
    angle_target = rospy.get_param("angle/target")

    if self.x > 0:
        # move forward
        self.left = dist_target_left + distance
        self.right = dist_target_right + distance
    elif self.x < 0:
        # move backward
        self.left = dist_target_left - distance
        self.right = dist_target_right - distance
    elif self.z > 0:
        # turn left
        self.angle = angle_target - ang
    elif self.z < 0:
        # turn right
        self.angle = angle_target + ang
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

    # self.msg_data = Twist() # Twist variable for message received
    self.x = 0 # variable for linear component
    self.z = 0 # variable for angular component

    self.left = 0 # left distance variable
    self.right = 0 # right distance variable
    self.angle = 0 # angle variable

    self.angle_multiplier = 1 # variable for the angle multiplier
    self.distance_multiplier = 1 # variable for the distance multiplier

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
