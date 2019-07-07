#!/usr/bin/env python

import hebi
import sys
import os
import rospy
import make_kin_utils
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import make_kin_utils

# numpy is aliased to np throughout this documentation.
import numpy as np

from time import sleep

lookup = hebi.Lookup()
# Give the Lookup process 2 seconds to discover modules
sleep(2)
print('Modules found on network:')
print(lookup.entrylist)


class RightArm(object):
  
  def __init__(self):

    #subscribers for moving to goal states and hold at obstacle position
    rospy.Subscriber("/joint_states", JointState, self.feedback_callback, queue_size=1)
    rospy.Subscriber("/hebiros/right_arm/feedback/joint_state", JointState, self.fb, queue_size=1)
    self.pub = rospy.Publisher('joint_states', JointState, queue_size=1)


    kin, joint_mask = make_kin_utils.make_backpack_kin()
    self.group = hebi.Lookup().get_group_from_names(['*'], ['base_temp', 'X-00476', 'elbow_temp'])
    self.kin = kin
    self.joint_mask = joint_mask
    self.stop = False
    self.at_new = False
    self.rate = rospy.Rate(20)
    self.gravity = np.matrix([0,0,9.81]).T
    self.js = JointState()
    self.counter = 0

  # Joint limits based on position
  # Temporary method right now once gravity compensation works
  def threshold(self, joints, efforts):

    if joints[0] > -1.57 and joints[0] < 1.57:
      if efforts[0] > 7 or efforts[0] < -2:
        return True
    else:
      if efforts[0] < -6.5 or efforts[0] > 2:
        return True

    if joints[1] > 0 and joints[1] < 3.14:
      if efforts[1] < -5 or efforts[1] > 3:
        return True
    else:                               
      if efforts[1] > 5 or efforts[1] < -3:
        return True

    if joints[2] > 0 and joints[2] < 3.14:
      if efforts[2] > 2 or efforts[2] < -2:
        return True
    else:
      if efforts[2] < -2.5 or efforts[2] > 2:
        return True

    
    return False
    

  def feedback_callback(self, data):
    # Robot has not collidesd
    if not self.stop:
      group = self.group
      kin = self.kin
      joint_mask = self.joint_mask
      positions = [0]*3

      # Move robot to joint state positions
      positions[0] = data.position[0]
      positions[1] = data.position[1]
      positions[2] = data.position[2]

      # Gravity Compensation values
      gravity= np.matrix([0,0,9.81]).T
      tauG, fkout = make_kin_utils.getGravCompTorques(kin, joint_mask, positions, gravity)
      group_command = hebi.GroupCommand(3)
      group_command.position = positions
      #group_command.effort = tauG
      group.send_command(group_command)

  def fb(self, data):

    kin = self.kin
    joint_mask = self.joint_mask
    group = self.group
    gravit = self.gravity
    positions = [0]*3

    # Robot collides when exceeds joint threshold
    if (self.threshold(data.position, data.effort)) or self.stop:
    	self.stop = True

    if self.stop:
    	positions = data.position
    	tauG, fkout = make_kin_utils.getGravCompTorques(kin, joint_mask, positions, gravity)
      	group_command = hebi.GroupCommand(3)
      	group_command.position = positions
      	#group_command.effort = tauG
      	group.send_command(group_command)
      	self.js.position = positions
      	self.pub.publish(self.js)
      	self.counter += 1
      	# Ensure that joint state message has fully arrive to current point
      	if self.counter >= 10:
      		self.stop = False
      		self.counter = 0


if __name__ == '__main__':
  rospy.init_node('right_arm', anonymous=True)
  objecta = RightArm()
  rospy.spin()