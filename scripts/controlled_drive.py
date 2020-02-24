#!/usr/bin/env python
import rospy
from lab1.msg import balboaLL # import balboa message
from lab1.msg import pid_input # import pid_input message

PI = 3.14159265358979 # global variable for PI

def parse_balboa_msg(data, self):
<<<<<<< HEAD
    # self.dist_left = data.distanceLeft # unpack left encoder
    # self.dist_right = data.distanceRight # unpack right encoder
    self.dist_left = data.encoderCountLeft # unpack left encoder
    self.dist_right = data.encoderCountRight # unpack right encoder
=======
    self.ang_left = data.angleX # unpack angle X
    self.ang_right = 0 # unused
    temp_ang = rospy.get_param('angle/target') # get angle target from user
    if temp_ang != self.ang_target:
        # angle target updated
        self.ang_target = temp_ang
        self.dist_updated = False
>>>>>>> angPID

    # adjust distance target based on target angle
    if not self.dist_updated:
        # get distances
        self.dist_tar_left = rospy.get_param('distance/tar/left')
        self.dist_tar_right = rospy.get_param('distance/tar/right')

        dist_adj = (self.ang_target / 360) * self.ECX # calculate adjustment

        self.dist_tar_left = self.dist_tar_left - dist_adj
        self.dist_tar_left = self.dist_tar_left + dist_adj
        rospy.set_param('distance/tar/left',self.dist_tar_left)
        rospy.set_param('distance/tar/right',self.dist_tar_left)
        self.dist_updated = True

    self.dist_left = data.encoderCountLeft # unpack left encoder
    self.dist_right = data.encoderCountRight # unpack right encoder
    #self.dist_target = rospy.get_param('distance/target') # get distance target from user
    self.dist_tar_left = rospy.get_param('distance/tar/left') # get distance target from user
    self.dist_tar_right = rospy.get_param('distance/tar/right') # get distance target from user

    # convert encoder distance to mm
    self.dist_left = self.dist_left * self.DPC
    self.dist_right = self.dist_right * self.DPC

    # convert angle reading from millidegrees to degrees
    self.ang_left = self.ang_left / 1000

    # Publish the current and target distance values
    self.dist_pid_input.source = 'distance'
    self.dist_pid_input.current_left = self.dist_left
    self.dist_pid_input.current_right = self.dist_right
    #self.dist_pid_input.target_left = self.dist_target
    #self.dist_pid_input.target_right = self.dist_target
    self.dist_pid_input.target_left = self.dist_tar_left
    self.dist_pid_input.target_right = self.dist_tar_right
    self.dist.publish(self.dist_pid_input)

    # Publish the current and target angle values
    self.ang_pid_input.source = 'angle'
    self.ang_pid_input.current_left = self.ang_left
    self.ang_pid_input.current_right = 0 # unused
    self.ang_pid_input.target_left = self.ang_target
    self.ang_pid_input.target_right = 0 # unused
    self.ang.publish(self.ang_pid_input)

class TheNode(object):
  # This class holds the rospy logic for sending pid_input messages
  # from a published balboa message and user input 

  def __init__(self):

    rospy.init_node('remote_drive') # intialize node
    
    # initialize publisher node for distance PID controller
    self.dist = rospy.Publisher('/dist', pid_input, queue_size=10)

    self.dist_left = 0 # init left distance
    self.dist_right = 0 # init right distance
    self.dist_pid_input = pid_input() # default pid_input type
    #self.dist_target = rospy.get_param('distance/target') # init distance target
    self.dist_tar_left = rospy.get_param('distance/tar/left') # init distance target
    self.dist_tar_right = rospy.get_param('distance/tar/right') # init distance target

    # Encoder count per revolution is gear motor ratio (3344/65)
    # times gearbox ratio (2.14/1) times encoder revolution (12/1)
    CPR = (3344 / 65) * 2.14 * 12

    # Distance per revolution is 2 PI times wheel radius (40 mm)
    distPR = 2*PI*40

    # Distance per encoder count is distPR / CPR
    self.DPC = distPR / CPR

    # Wheel distance per x-axis revolution is 2 PI times wheel spacing (102 mm)
    wDistPR = 2*PI*102

    # Encoder count per x-axis angle is half of wDistPR / DPC
    self.ECX = 0.5 * (wDistPR / self.DPC)

    # initialize publisher node for angle PID controller
    self.ang = rospy.Publisher('/ang', pid_input, queue_size=10)
    self.ang_left = 0 # init left angle
    self.ang_right = 0 # init right angle
    self.ang_pid_input = pid_input() # default pid_input type
    self.ang_target = rospy.get_param('angle/target') # init angle target
    self.ang_tar_old = 0
    self.dist_updated = True

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
