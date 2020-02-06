#!/usr/bin/env python
import rospy
from lab1.msg import balboaMotorSpeeds # import motor speed message
from std_msgs.msg import Header # import header message
from geometry_msgs.msg import Twist # import Twist message

def parse_msg(data, self):
    self.msg_data = data
    self.x = data.linear.x
    self.z = data.angular.z

    if self.x > 0:
        # move forward
        self.spdMsg.left = 1
        self.spdMsg.right = 1
    elif self.x < 0:
        # move backward
        self.spdMsg.left = -1
        self.spdMsg.right = -1
    elif self.z > 0:
        # turn left
        self.spdMsg.left = -1
        self.spdMsg.right = 1
    elif self.z < 0:
        # turn right
        self.spdMsg.left = 1
        self.spdMsg.right = -1
    else:
        rospy.loginfo("Parse error")

    self.mtrSpd.publish(self.spdMsg)

class TheNode(object):
  # This class holds the rospy logic for sending a motor speed message
  # from a published teleop_turtle_key message

  def __init__(self):

    rospy.init_node('remote_drive') # intialize node

    # initialize publisher node for turtle 1
    self.mtrSpd = rospy.Publisher('/motorSpeeds', balboaMotorSpeeds, queue_size=10)

    self.msg_data = Twist() # Twist variable for message received
    self.x = 0
    self.z = 0
    self.spdMsg = balboaMotorSpeeds()
    self.spdMsg.header = Header()
    self.spdMsg.left = 0
    self.spdMsg.right = 0

  def main_loop(self):
    # initialize subscriber node for messages from teleop_turtle_key
    rospy.Subscriber('/turtle1/cmd_vel', Twist, parse_msg, self)

    rospy.spin()

if __name__ == '__main__':
    try:
        a = TheNode()
        a.main_loop()
    except rospy.ROSInterruptException:
        pass
