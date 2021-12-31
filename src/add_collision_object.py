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

# Wooden Block
global rect_frame
rect_frame = 'object_28'

# Cylindrical Shaft
global cylind_frame
cylind_frame = 'object_29'

# Ball
global sphere_frame
sphere_frame = 'object_30'

class collision_object:

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()    
        self.group = moveit_commander.MoveGroupCommander("arm")

        self.robot_frame = 'base_footprint'        


    #Get the tf of a given frame            
    def pub_object(self, frame_id):

        recog_flag = True

        self.robot_frame = 'base_footprint'

        try:
            self.listener.waitForTransform("/base_footprint", "/object_28", rospy.Time(), rospy.Duration(4.0))
            (trans,rot) = self.listener.lookupTransform(self.robot_frame, frame_id, rospy.Time(0))

            if frame_id == rect_frame:
                self.box(trans[0], trans[1], trans[2])
            elif frame_id == cylind_frame:
                self.cylinder(trans[0], trans[1], trans[2])
            elif frame_id == sphere_frame:
                self.sphere(trans[0], trans[1], trans[2])

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            recog_flag = False
            print('cant get frame')

        return (trans,rot)


    def box(self, x, y, z):

        # Add grasping object - BOX
        rospy.sleep(2)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot_frame
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y+0.011 
        box_pose.pose.position.z = z+0.05 
        box_name = "box"
        self.scene.add_box(box_name, box_pose, size=(0.022, 0.018, 0.10))

    def cylinder(self, x, y, z):

        # # Add grasping object - CYLINDER
        rospy.sleep(2)
        cylinder_pose = geometry_msgs.msg.PoseStamped()
        cylinder_pose.header.frame_id = self.robot_frame
        cylinder_pose.pose.orientation.w = 1.0
        cylinder_pose.pose.position.x = x
        cylinder_pose.pose.position.y = y 
        cylinder_pose.pose.position.z = z 
        cylinder_name = "cylinder"
        self.scene.add_cylinder(cylinder_name, cylinder_pose, height=0.07, radius=0.011)

    def sphere(self, x, y, z):

        # Add grasping object - SPHERE
        rospy.sleep(2)
        sphere_pose = geometry_msgs.msg.PoseStamped()
        sphere_pose.header.frame_id = self.robot_frame
        sphere_pose.pose.orientation.w = 1.0
        sphere_pose.pose.position.x = x
        sphere_pose.pose.position.y = y 
        sphere_pose.pose.position.z = z 
        sphere_name = "sphere"
        self.scene.add_sphere(sphere_name, sphere_pose, radius=0.1)
        #scene.set_object_color(sphere_name, 200, 0, 0, a=0.9)

    def attach(self, object_name):

        print "============ Press `Enter` to attach grasping object to the eff ..."
        raw_input()

        #Attach grasping object
        grasping_group = 'arm'
        eef_link = self.move_group.get_end_effector_link()

        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(eef_link, object_name, touch_links=touch_links)
        rospy.sleep(5)

    def dettach(self, object_name):

        #Detach grasping object
        eef_link = self.move_group.get_end_effector_link()
        self.scene.remove_attached_object(eef_link, object_name)
        rospy.sleep(5)






