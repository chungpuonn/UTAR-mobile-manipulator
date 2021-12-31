#! /usr/bin/env python

#Add collision object into planning scene in react to the find_object pkg

import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm")
group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
eef_link = move_group.get_end_effector_link()

class attach_dettach:

    def attach_box(self):
        print "============ Press `Enter` to attach wooden block to the gripper ..."
        raw_input()

        #Attach grasping object
        grasping_group = 'arm'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, "box", touch_links=touch_links)
        rospy.sleep(5)

    def dettach_box(self):

        #Detach grasping object
        print "============ Press `Enter` to dettach wooden block from the gripper ..."
        raw_input()
        scene.remove_attached_object(eef_link, "box")
        rospy.sleep(5)

    # def attach_box_pocket(self):

    #     print "============ Press `Enter` to attach wooden block to the robot pocket ..."
    #     raw_input()

    #     #Attach grasping object
    #     grasping_group = 'arm'
    #     touch_links = robot.get_link_names(group=grasping_group)
    #     scene.attach_box("base_link", "box", touch_links=touch_links)
    #     rospy.sleep(5)

    # def dettach_box_pocket(self):

    #     #Detach grasping object
    #     print "============ Press `Enter` to dettach wooden block from the robot pocket ..."
    #     raw_input()
    #     scene.remove_attached_object("base_link", "box")
    #     rospy.sleep(5)

    def attach_cylin(self):

        print "============ Press `Enter` to attach cylindrical shaft to the gripper ..."
        raw_input()

        #Attach grasping object
        grasping_group = 'arm'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, "cylinder", touch_links=touch_links)
        rospy.sleep(5)

    def dettach_cylin(self):

        #Detach grasping object
        print "============ Press `Enter` to dettach cylindrical shaft from the gripper ..."
        raw_input()
        scene.remove_attached_object(eef_link, "cylinder")
        scene.remove_world_object("cylinder")
        rospy.sleep(5)

    def remove_box(self):

        #Detach grasping object
        print "============ Press `Enter` to remove wooden block from the scene ..."
        raw_input()
        scene.remove_world_object("box")
        rospy.sleep(5)

    





