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

class PoseObject(object):
    def __init__(self):
        super(PoseObject, self).__init__()
        self.stop = False  # True when hits obstacles
        self.initial_pose = True  # In plan path mode
        self.counter = 0

        rospy.Subscriber("/aruco_single/pose", PoseStamped, self.fb)
        rospy.Subscriber("/joint_states", JointState, self.js_callback)
        rospy.Subscriber("/hebiros/right_arm/feedback/joint_state", JointState, self.limit, queue_size=1)

        self.hebi = hebi.Lookup().get_group_from_names(['*'], ['base_temp', 'X-00476', 'elbow_temp'])

        self.pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.pubMark = rospy.Publisher("visualization_marker", Marker, queue_size=1) 

        self.js = JointState()

        urdf_str = rospy.get_param('/robot_description')
        self.ik_solver = IK("world", "end_link/INPUT_INTERFACE",  urdf_string=urdf_str)

        self.group = moveit_commander.MoveGroupCommander("right_arm")
    
        # Feedback data of joint angles from hebi for recalculation of IK
        self.x, self.y, self.z = 0, 0, 0
        '''
        self.alpha = .25    # For filtering
        self.filter_pose = np.array([0.0, 0.0, 1.0])
        '''
        # Current pose angles when arm hits object
        self.joint_pose = [0, 0, 0]

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

        print("start")

    # Record current joint angles of arm when it hits objects
    def limit(self, data):
        if self.threshold(data.position, data.effort):
            self.stop = True
        if self.stop:
            group_command = hebi.GroupCommand(3)
            group_command.position = data.position
            self.hebi.send_command(group_command)
            self.js.position = data.position 
            self.counter += 1
            self.pub.publish(self.js)
            # Ensures that joint state messages have fully arrived 
            if self.counter >= 5:
                self.initial_pose = True
                self.counter = 0


    # Execute planned path through joint state messages
    def js_callback(self, data):
        self.js = data
        if self.initial_pose and not self.stop:  
            group_command = hebi.GroupCommand(3)
            positions = np.array([data.position[0], data.position[1], data.position[2]])
            #self.filter_pose = self.alpha*self.filter_pose+(1-self.alpha)*positions
            #group_command.position = self.filter_pose
            group_command.position = positions
            self.hebi.send_command(group_command)

    # Subscriber callback to april tag position
    def fb(self, data):
        jspose = self.js.position
        self.x = data.pose.position.x
        self.y = data.pose.position.y
        self.z = data.pose.position.z

        xyz_hebi = [0, 0, 0]
        xyz_hebi[0] = self.z-.3
        xyz_hebi[1] = -self.x + .15
        xyz_hebi[2] = -self.y + .6

        #self.old_pose = (1-self.alpha)*xyz_hebi + alpha*self.old_pose
    
        self.robotMarker.pose.position.x = xyz_hebi[0]
        self.robotMarker.pose.position.y = xyz_hebi[1]
        self.robotMarker.pose.position.z = xyz_hebi[2]  

        self.pubMark.publish(self.robotMarker)

        #xyz_hebi = self.old_pose

        # Path plan with collision checking to initial pose
        if self.initial_pose:
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.orientation.w = 1.0
            pose_goal.position.x = xyz_hebi[0]
            pose_goal.position.y = xyz_hebi[1]
            pose_goal.position.z = xyz_hebi[2]

            self.group.set_pose_target(pose_goal)
            self.group.set_goal_orientation_tolerance(3)
            self.group.set_goal_joint_tolerance(.05)
            
            plan = self.group.plan()

            if plan[0]:
                self.stop = False
                self.group.execute(plan[1], wait=False)
                self.group.stop()
                self.initial_pose = False
        

        # Handtrack without collision checking afterwards
        elif not self.initial_pose and not self.stop:
            position_approx = [jspose[0], jspose[1], jspose[2]]
            positions = self.ik_solver.get_ik(position_approx, xyz_hebi[0], xyz_hebi[1], xyz_hebi[2], 0.0, 0.0, 0.0, 1.0)
            if positions == None:
                #print("no joint states found")
                positions = position_approx

            group_command = hebi.GroupCommand(3)
            group_command.position = positions
            self.hebi.send_command(group_command)
            joints = (positions[0], positions[1], positions[2])
            self.js.position = joints
            self.pub.publish(self.js)


    # Joint limits based on position
    # Temporary method right now once gravity compensation works
    def threshold(self, joints, efforts):
        if joints[0] > -1.57 and joints[0] < 1.57:
            if efforts[0] > 4 or efforts[0] < -3:
                return True
        else:
            if efforts[0] < -5 or efforts[0] > 3:
            return True

        if joints[1] > 0 and joints[1] < 3.14:
            if efforts[1] < -7 or efforts[1] > 5:
            return True
        else:                               
            if efforts[1] > 7 or efforts[1] < -8:
                return True

        if joints[2] > 0 and joints[2] < 3.14:
            if efforts[2] > 5 or efforts[2] < -4:
                return True
        else:
            if efforts[2] < -4 or efforts[2] > 5:
            return True
 
        return False

if __name__ == '__main__':
    rospy.init_node('handtracking', anonymous=True)
    PoseObject = PoseObject()
    rospy.spin()
