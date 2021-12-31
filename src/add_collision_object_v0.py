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

class add_coll_obj:

    def add_coll_obj(self):

        scene = moveit_commander.PlanningSceneInterface()    
  

        # print "============ Press `Enter` to update table to the planning scene ..."
        # raw_input()

        # # Add grasping object
        # rospy.sleep(2)
        # box_pose = geometry_msgs.msg.PoseStamped()
        # box_pose.header.frame_id = "Table"
        # box_pose.pose.orientation.w = 1.0
        # box_pose.pose.position.x = 0.2873/2 
        # box_pose.pose.position.y = 1.0/2 
        # box_pose.pose.position.z = 0.3269/2 
        # box_name = "box"
        # scene.add_box(box_name, box_pose, size=(0.2873, 1.0, 0.3269))

        # Subscribe to tf frame pose
        listener = tf.TransformListener()
        rospy.sleep(2)

        rect_flag = 0
        cyl_flag = 0
        sphe_flag = 0

        rect_flag = listener.frameExists("object_28")
        cyl_flag = listener.frameExists("object_29")
        sphe_flag = listener.frameExists("object_30")

        if rect_flag == 1:

            listener.waitForTransform("/base_footprint", "/object_28", rospy.Time(), rospy.Duration(4.0))
            (trans, rot) = listener.lookupTransform('base_footprint', 'object_28', rospy.Time())

            goal_coord = [trans[0], trans[1], trans[2]] # x, y, z

            print "============ Press `Enter` to add a wooden block to the planning scene ..."
            raw_input()

            # Add grasping object - BOX
            rospy.sleep(2)
            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.header.frame_id = "base_footprint"
            box_pose.pose.orientation.w = 1.0
            box_pose.pose.position.x = trans[0] -0.0274-0.01
            box_pose.pose.position.y = trans[1] 
            box_pose.pose.position.z = trans[2] +0.014

            print (box_pose)

            box_name = "box"
            scene.add_box(box_name, box_pose, size=(0.0274 *0.85, 0.0187*0.80, 0.065))

            rect_flag = 0
            cyl_flag = 0
            sphe_flag = 0

        elif cyl_flag == 1:

            listener.waitForTransform("/base_footprint", "/object_29", rospy.Time(), rospy.Duration(4.0))
            (trans, rot) = listener.lookupTransform('base_footprint', 'object_29', rospy.Time())

            goal_coord = [trans[0], trans[1], trans[2]] # x, y, z

            print "============ Press `Enter` to add a cylindrical shaft to the planning scene ..."
            raw_input()

            # # Add grasping object - CYLINDER
            rospy.sleep(2)
            cylinder_pose = geometry_msgs.msg.PoseStamped()
            cylinder_pose.header.frame_id = "base_footprint"
            cylinder_pose.pose.orientation.w = 1.0
            cylinder_pose.pose.position.x = trans[0] -0.02-0.015
            cylinder_pose.pose.position.y = trans[1] +0.009
            cylinder_pose.pose.position.z = trans[2] +0.014
            cylinder_name = "cylinder"
            scene.add_cylinder(cylinder_name, cylinder_pose, height=0.065, radius=0.009)

            rect_flag = 0
            cyl_flag = 0
            sphe_flag = 0

        elif sphe_flag == 1:

            listener.waitForTransform("/base_footprint", "/object_30", rospy.Time(), rospy.Duration(4.0))
            (trans, rot) = listener.lookupTransform('base_footprint', 'object_30', rospy.Time())

            goal_coord = [trans[0], trans[1], trans[2]] # x, y, z

            print "============ Press `Enter` to add a ball to the planning scene ..."
            raw_input()

            # Add grasping object - SPHERE
            rospy.sleep(2)
            sphere_pose = geometry_msgs.msg.PoseStamped()
            sphere_pose.header.frame_id = "base_link"
            sphere_pose.pose.orientation.w = 1.0
            sphere_pose.pose.position.x = trans[0] -0.03-0.01
            sphere_pose.pose.position.y = trans[1]
            sphere_pose.pose.position.z = trans[2] +0.012
            sphere_name = "sphere"
            scene.add_sphere(sphere_name, sphere_pose, radius=0.015)

            rect_flag = 0
            cyl_flag = 0
            sphe_flag = 0

            #scene.set_object_color(sphere_name, 200, 0, 0, a=0.9)






