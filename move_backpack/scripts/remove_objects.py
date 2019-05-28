#!/usr/bin/env python

# The code in this file is modified based on https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py

# This file removes obstacles already added to the world. The obstacles are named "box1", "box2", "box3", etc. in order which they were addeded. To remove the desired box, run the command "rosron move_backpack remove_objects.py 'box-name' "

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg


class RemoveObject(object):
  def __init__(self):

    # First initialize `moveit_commander`_ and a `rospy`_ node
    moveit_commander.roscpp_initialize(sys.argv[0])
    rospy.init_node('remove_object',
                    anonymous=True)
    self.scene = moveit_commander.PlanningSceneInterface()
    self.box_name = str(sys.argv[1])

    print "============ Press `Enter` to remove " + self.box_name + " from scene ..."
    raw_input()

  # Remove specified box obstacle
  def remove_object(self):
    self.scene.remove_attached_object("motor1/INPUT_INTERFACE", name=self.box_name)
    self.scene.remove_world_object(self.box_name)

    
def main():
  print "============ Press `Enter` to remove objects from scene ..."
  raw_input()
  remove_box = RemoveObject()
  rospy.sleep(4.0)
  remove_box.remove_object()


if __name__ == '__main__':
  main()
