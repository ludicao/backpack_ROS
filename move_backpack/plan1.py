#!/usr/bin/env python

import numpy as np
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import rospy
import moveit_commander
from trac_ik_python.trac_ik import IK
import hebi
import subprocess
import sys
import os
from pp import Poses
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

class PoseObject(object):
    def __init__(self):
        super(PoseObject, self).__init__()
        self.stop = False  # True when hits obstacles
        self.plan_path= True  # In plan path mode	
	self.last = None
	self.check = False
	self.ismoving = False
        rospy.Subscriber("/aruco_single/pose", PoseStamped, self.fb, queue_size=1)
        rospy.Subscriber("/move_group/fake_controller_joint_states", JointState, self.js_callback, queue_size=1)
        rospy.Subscriber("/hebiros/right_arm/feedback/joint_state", JointState, self.limit, queue_size=1)
        rospy.Subscriber("plan_status", String, self.plan_status, queue_size=1)
	rospy.Subscriber("joint_states", JointState, self.keep_running, queue_size = 1)

        self.hebi = hebi.Lookup().get_group_from_names(['*'], ['base_temp', 'X-00476', 'elbow_temp'])

        self.pub = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=1)
	self.pubBool = rospy.Publisher('pose', Float32MultiArray, queue_size = 1)

        self.js = JointState()
        urdf_str = rospy.get_param('/robot_description')
        self.ik_solver = IK("world", "end_link/INPUT_INTERFACE",  urdf_string=urdf_str)

        # Current pose angles when arm hits object
        self.joint_pose = [0, 0, 0]
	'''
        # Marker for visualization of apriltag in Rviz
        self.robotMarker = Marker()
        self.robotMarker.header.frame_id = "world"
        self.robotMarker.header.stamp = rospy.get_rostime()
        self.robotMarker.ns = "robot"
        self.robotMarker.id = 0
        self.robotMarker.type = 2 # sphere
        self.robotMarker.action = 0
        self.robotMarker.pose.position.x = 0
        self.robotMarker.pose.position.y = 0
        self.robotMarker.pose.position.z = 0
        self.robotMarker.pose.orientation.x = 0
        self.robotMarker.pose.orientation.y = 0
        self.robotMarker.pose.orientation.z = 0
        self.robotMarker.pose.orientation.w = 1.0
        self.robotMarker.scale.x = .05
        self.robotMarker.scale.y = .05
        self.robotMarker.scale.z = .05
        self.robotMarker.color.r = 0.0
        self.robotMarker.color.g = 1.0
        self.robotMarker.color.b = 0.0
        self.robotMarker.color.a = 1.0
        self.robotMarker.lifetime = rospy.Duration(0)
	'''
        print("start")

    # Record current joint angles of arm when it hits objects
    def limit(self, data):
        self.js = data
        if self.threshold(data.position, data.effort) or self.stop:
	    if self.threshold(data.position, data.effort):
	    	self.joint_pose = data.position
                self.first = True	

	    print("in limit loop", self.joint_pose)
            group_command = hebi.GroupCommand(3)
            group_command.position = self.joint_pose
            self.hebi.send_command(group_command)
	    
            if self.first:
		self.js.position = self.joint_pose
	        self.pub.publish(self.js)
		while self.pub.get_num_connections() == 0:
                    rospy.sleep(.1)
		self.plan_path = True
                self.stop = True
		self.first = False


    # Execute planned path through joint state messages
    def js_callback(self, data):
        if self.plan_path and not self.stop and self.ismoving:  
	    print("in js_callback loop", self.ismoving)
            group_command = hebi.GroupCommand(3)
            positions = np.array([data.position[0], data.position[1], data.position[2]])
            #self.filter_pose = self.alpha*self.filter_pose+(1-self.alpha)*positions
            #group_command.position = self.filter_pose
            group_command.position = positions
            self.hebi.send_command(group_command)

    def plan_status(self, data):
	print("in listener2", self.plan_path, data)
	if self.plan_path and not self.stop:
            if data.data == "yes":
		self.stop = False 
		self.ismoving = True
	    if data.data == "yes2":
		self.plan_path = False
		self.ismoving = False
    


    # Subscriber callback to april tag position
    def fb(self, data):
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.position.z

        xyz_hebi = [0, 0, 0]
        xyz_hebi[0] = z-.3
        xyz_hebi[1] = -x + .15
        xyz_hebi[2] = -y + .6

	self.xyz_hebi = xyz_hebi
    
        #self.robotMarker.pose.position.x = xyz_hebi[0]
        #self.robotMarker.pose.position.y = xyz_hebi[1]
        #self.robotMarker.pose.position.z = xyz_hebi[2]
	if not self.check:
	    self.check = True 



    def keep_running(self, data):
        jspose = self.js.position
        if self.check:
            xyz_hebi = self.xyz_hebi
            # Path plan with collision checking to initial pose
            if self.plan_path and not self.ismoving :
	        print("in if self.plan_path in fb")

	        f = Float32MultiArray()
	        f.data = xyz_hebi
	        self.pubBool.publish(f)
	        while self.pubBool.get_num_connections() == 0:
		    rospy.sleep(.1)

            # Handtrack without collision checking afterwards
            if not self.plan_path:
	        print("in handtracking")
                position_approx = [jspose[0], jspose[1], jspose[2]]
                positions = self.ik_solver.get_ik(position_approx, xyz_hebi[0], xyz_hebi[1], xyz_hebi[2], 0.0, 0.0, 0.0, 1.0, .01, .01, .01)
	        if self.last == None:
		    self.last = position_approx
                if positions == None:
                    print("no joint states found")
                    positions = self.last
                group_command = hebi.GroupCommand(3)
                group_command.position = positions
                self.last = positions
                self.hebi.send_command(group_command)


    # Joint limits based on position
    # Temporary method right now once gravity compensation works
    def threshold(self, joints, efforts):
        if joints[0] > -1.57 and joints[0] < 1.57:
            if efforts[0] > 10 or efforts[0] < -7:
	        print("1111111111111111111111111111111111111")
                return True
        else:
            if efforts[0] < -8 or efforts[0] > 5:
		print("22222222222222222222222222222222222222222")
            	return True

        if joints[1] > 0 and joints[1] < 3.14:
            if efforts[1] < -8 or efforts[1] > 6:
		print("33333333333333333333333333333333333333333")
            	return True
        else:                               
            if efforts[1] > 8 or efforts[1] < -9:
		print("44444444444444444444444444444444444444444")
                return True

        if joints[2] > 0 and joints[2] < 3.14:
            if efforts[2] > 6 or efforts[2] < -4:
		print("55555555555555555555555555555555555555555")
                return True
        else:
            if efforts[2] < -6 or efforts[2] > 5:
		print("666666666666666666666666666666666666666666")
            	return True
 
        return False

if __name__ == '__main__':
    rospy.init_node('handtracking', anonymous=True)
    PoseObject = PoseObject()
rospy.spin()
