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

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm")
group.set_planner_id("PRMkConfigDefault")
#"PRMkConfigDefault"
group.set_planning_time(10)
group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
eef_link = move_group.get_end_effector_link()
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)


# Subscribe to tf frame pose


print "============ Press `Enter` to add a mesh to the planning scene ..."
raw_input()

# Add grasping object - MESH
rospy.sleep(2)
mesh_pose = geometry_msgs.msg.PoseStamped()
mesh_pose.header.frame_id = "base_link"
mesh_pose.pose.orientation.w = 1.0
mesh_pose.pose.position.x = 1 
mesh_pose.pose.position.y = 1 
mesh_pose.pose.position.z = 1 
mesh_name = "mesh"
scene.add_mesh(mesh_name, mesh_pose, '/home/puonn/catkin_ws/src/RosForSwiftAndSwiftPro/wheel_with_robot_v6/meshes/laser_link_2.stl')






