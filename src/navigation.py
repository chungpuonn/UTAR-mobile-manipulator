#!/usr/bin/env python

import rospy, os, sys
import enum
import time
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import atexit

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from move import move_base

from tf.transformations import quaternion_from_euler
from std_msgs.msg import String

import tf

global original
original = 0
global move
move = 0
global rect_flag
rect_flag = 0
global cylind_flag
cylind_flag = 0
global sphere_flag
sphere_flag = 0
global start
start = 1
global waiting
waiting = 0
global start_flag
start_flag = 0



class NavToPoint:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        listener = tf.TransformListener()
	# Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(120))
        rospy.loginfo("Connected to move base server")
        
        # A variable to hold the initial pose of the robot to be set by the user in RViz
        initial_pose = PoseWithCovarianceStamped()
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)



	# Get the initial pose from the user
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        
        # Make sure we have the initial pose
        while initial_pose.header.stamp == "" or start_flag == 0:
        	rospy.sleep(1)
        	rospy.loginfo("start_flag state %s",start_flag)
			
            
        rospy.loginfo("Ready to go")
        rospy.sleep(1)

        locations = dict()

        # Location A
        A_x = 2.620
        A_y = -0.224
        A_theta = 0.010

        quaternion = quaternion_from_euler(A_x, A_y, A_theta)
        # quaternion[2] = -0.773
        # quaternion[3] = 0.634
        locations['A'] = Pose(Point(A_x, A_y, 0.000), Quaternion(0, 0, quaternion[2], quaternion[3]))

        # Location B
        B_x = -2.375
        B_y = 3.451
        B_theta = 0.12

        quaternion = quaternion_from_euler(B_x, B_y, B_theta)
        locations['B'] = Pose(Point(B_x, B_y, 0.000), Quaternion(0, 0, quaternion[2], quaternion[3]))

        # Location C
        C_x = 0.541
        C_y = -4.431
        C_theta = 0.12

        quaternion = quaternion_from_euler(0.0, 0.0, C_theta)
        quaternion[2] = 0.994
        quaternion[3] = -0.106
        locations['C'] = Pose(Point(C_x, C_y, 0.000), Quaternion(0, 0, quaternion[2], quaternion[3]))

        
        self.goal = MoveBaseGoal()
        rospy.loginfo("Starting navigation test")


        # while not rospy.is_shutdown():
        #     self.goal.target_pose.header.frame_id = 'map'
        #     self.goal.target_pose.header.stamp = rospy.Time.now()

        #     # Robot will go to point A
        #     if start == 1:
        #         rospy.loginfo("Going to point A")
        #         rospy.sleep(2)
        #         self.goal.target_pose.pose = locations['A']
        #         self.move_base.send_goal(self.goal)
        #         waiting = self.move_base.wait_for_result(rospy.Duration(300))
        #         global waiting
        #         waiting = 1
        #         if waiting == 1:
        #             rospy.loginfo("Reached point A")
        #             rospy.sleep(1)
        #             name1_flag = 0

        #             global female_1_flag
        #             female_1 = listener.frameExists("object_25")
        #             rospy.loginfo("found female 1? %s", female_1)
        #             if (female_1 == 1 and female_1_flag == 0):
        #                 global female_1_flag
        #                 female_1_flag = 1
        #                 rospy.loginfo("detected female1")

        #             global female_2_flag
        #             female_2 = listener.frameExists("object_26")
        #             rospy.loginfo("found female 2? %s", female_2)
        #             if (female_2 == 1 and female_2_flag == 0):
        #                 global female_2_flag
        #                 female_2_flag = 1
        #                 rospy.loginfo("detected female2")

        #             if female_1_flag == 1 or female_2_flag == 1:
        #                 while start == 1:
        #                     global name1_flag
        #                     if name1_flag == 0:
        #                         soundhandle.say('please tell me your name')
        #                         rospy.loginfo("please tell me your name")
        #                     rospy.sleep(10)


        #     if start == 2:
        #         rospy.loginfo("Going to point C")
        #         rospy.sleep(2)
        #         self.goal.target_pose.pose = locations['C']
        #         self.move_base.send_goal(self.goal)
        #         waiting = self.move_base.wait_for_result(rospy.Duration(300))
        #         if waiting == 1:
        #             rospy.loginfo("Reached point C")
        #             rospy.sleep(2)
        #             global male_1_flag
        #             male_1 = listener.frameExists("object_24")
        #             # rospy.loginfo("found john 1 status %s", male_1)
        #             if (male_1 == 1 and male_1_flag == 0):
        #                 rospy.loginfo("found someone name is john and his fravourite drink is milk")
        #                 soundhandle.say('found someone name is john and his fravourite drink is milk')
        #                 rospy.sleep(2)
        #                 global male_1_flag
        #                 male_1_flag = 1
        #                 # rospy.loginfo("assign male flag 1")
        #                 self.goal.target_pose.pose = locations['D']
        #                 self.move_base.send_goal(self.goal)
        #                 waiting = self.move_base.wait_for_result(rospy.Duration(300))
        #                 if waiting == 1:
        #                     rospy.loginfo("Reached point D")
        #                     global move
        #                     move = 1
        #             soundhandle.say('here is an empty seat')
        #             rospy.loginfo("here is an empty seat")
        #             rospy.sleep(2)
        #             soundhandle.say('The name of the guest is ')
        #             rospy.loginfo("The name of the guest is ")
        #             rospy.sleep(2)
        #             global name1
        #             soundhandle.say(name1)
        #             rospy.loginfo(name1)
        #             rospy.sleep(1)
        #             global drink1
        #             soundhandle.say('The fravourite drink is ')
        #             rospy.loginfo("The fravourite drink is ")
        #             rospy.sleep(2)
        #             soundhandle.say(drink1)
        #             rospy.loginfo(drink1)
        #             rospy.sleep(1)
        #             global start
        #             start = 3

        #     if start == 3:
        #         rospy.loginfo("Going to point A")
        #         rospy.sleep(2)
        #         self.goal.target_pose.pose = locations['A']
        #         self.move_base.send_goal(self.goal)
        #         waiting = self.move_base.wait_for_result(rospy.Duration(300))
        #         if waiting == 1:
        #             rospy.loginfo("Reached point A")
        #             rospy.sleep(1)
                    
        #             global female_1_flag
        #             female_1 = listener.frameExists("object_25")
        #             rospy.loginfo("found female 1? %s", female_1)
        #             if (female_1 == 1 and female_1_flag == 0):
        #                 global female_1_flag
        #                 female_1_flag = 1
        #                 rospy.loginfo("detected female1")

        #             global female_2_flag
        #             female_2 = listener.frameExists("object_26")
        #             rospy.loginfo("found female 2? %s", female_2)
        #             if (female_2 == 1 and female_2_flag == 0):
        #                 global female_2_flag
        #                 female_2_flag = 1
        #                 rospy.loginfo("detected female2")

        #             if female_1_flag == 1 and female_2_flag == 1:
        #                 while start == 3:
        #                     global name2_flag
        #                     if name2_flag == 0:
        #                         soundhandle.say('please tell me your name')
        #                         rospy.loginfo("please tell me your name")
        #                     rospy.sleep(10)

        #     if start == 4:
        #         rospy.loginfo("Going to next point")
        #         rospy.sleep(2)
        #         global move
        #         if move == 1:
        #             rospy.loginfo("point E")
        #             self.goal.target_pose.pose = locations['E']
        #             self.move_base.send_goal(self.goal)
        #             waiting = self.move_base.wait_for_result(rospy.Duration(300))
        #         else:
        #             rospy.loginfo("point D")
        #             self.goal.target_pose.pose = locations['D']
        #             self.move_base.send_goal(self.goal)
        #             waiting = self.move_base.wait_for_result(rospy.Duration(300))
        #         if waiting == 1:
        #             rospy.loginfo("Reached point")
        #             rospy.sleep(1)
        #             global male_1
        #             global male_1_flag
        #             male_1 = listener.frameExists("object_24")
        #             if (male_1 == 1 and male_1_flag == 0):
        #                 rospy.loginfo("found someone name is john and his fravourite drink is milk")
        #                 soundhandle.say('found someone name is john and his fravourite drink is milk')
        #                 rospy.sleep(2)
        #                 male_1_flag = 1
        #                 self.goal.target_pose.pose = locations['E']
        #                 self.move_base.send_goal(self.goal)
        #                 waiting = self.move_base.wait_for_result(rospy.Duration(300))
        #                 if waiting == 1:
        #                     rospy.loginfo("Reached point E")
        #                     global start
        #                     start = 6
        #             soundhandle.say('here is an empty seat')
        #             rospy.loginfo("here is an empty seat")
        #             rospy.sleep(2)
        #             soundhandle.say('The name of the guest is ')
        #             rospy.loginfo("The name of the guest is ")
        #             rospy.sleep(1)
        #             global name2
        #             soundhandle.say(name2)
        #             rospy.loginfo(name2)
        #             rospy.sleep(1)
        #             global drink2
        #             soundhandle.say('The fravourite drink is ')
        #             rospy.loginfo("The fravourite drink is ")
        #             rospy.sleep(2)
        #             soundhandle.say(drink2)
        #             rospy.loginfo(drink2)
        #             global start
        #             if start == 4:
        #                 start = 5

        #     if start == 5:
        #         rospy.loginfo("Going to point E")
        #         rospy.sleep(2)
        #         self.goal.target_pose.pose = locations['E']
        #         self.move_base.send_goal(self.goal)
        #         waiting = self.move_base.wait_for_result(rospy.Duration(300))
        #         if waiting == 1:
        #             rospy.loginfo("Reached point E")
        #             rospy.sleep(2)
        #             soundhandle.say('here is an empty seat')
        #             rospy.loginfo("here is an empty seat")
        #             rospy.sleep(2)
        #             soundhandle.say('The name of the guest is ')
        #             rospy.loginfo("The name of the guest is ")
        #             rospy.sleep(1)
        #             global name2
        #             soundhandle.say(name2)
        #             rospy.loginfo(name2)
        #             rospy.sleep(1)
        #             global drink2
        #             soundhandle.say('The fravourite drink is ')
        #             rospy.loginfo("The fravourite drink is ")
        #             rospy.sleep(2)
        #             soundhandle.say(drink2)
        #             rospy.loginfo(drink2)
        #             global start
        #             start = 0

        #     # After reached point A, robot will go back to initial position
        #     elif start == 0:
        #         rospy.loginfo("Going back home")
        #         rospy.sleep(2)
        #         self.goal.target_pose.pose = self.origin
        #         self.move_base.send_goal(self.goal)
        #         waiting = self.move_base.wait_for_result(rospy.Duration(300))
        #         if waiting == 1:
        #             rospy.loginfo("Reached home")
        #             rospy.sleep(2)

        #     rospy.Rate(5).sleep()


    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose
        if original == 0:
            self.origin = self.initial_pose.pose.pose
            # global original
            original = 1

    

    def cleanup(self):
        rospy.loginfo("Shutting down navigation	....")
        self.move_base.cancel_goal()

if __name__=="__main__":
	rospy.init_node('navi_point')
	listener = tf.TransformListener()
	try:
		NavToPoint()
		rospy.spin()
	except:
		pass
	
