#! /usr/bin/env python
# -*- encoding: UTF-8 -*
import rospy, os, sys
import enum
import time
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import atexit


from navigation import NavToPoint
from move import move_base
from add_collision_object_v0 import add_coll_obj

# Wooden Block
global rect_frame
rect_frame = '/object_28'

# Cylindrical Shaft
global cylind_frame
cylind_frame = '/object_29'

# Ball
global sphere_frame
sphere_frame = '/object_30'

global rect_flag
rect_flag = 0
global cylind_flag
cylind_flag = 0
global sphere_flag
sphere_flag = 0

class program():

    def __init__(self):
        rospy.init_node("program_navigation")

        self.mb = move_base()

        self.r = rospy.Rate(10)
        self.step = 0 
        self.reached = False
        self.previous_command = ''

        print "Starting..."
        self.carry_task()

    
    def go_s1(self):

        try:
            #Station 1 Pose
            print "Move to Station 1..."
            self.mb.move_to_point(4.16, 0.24, -0.15) 
            self.mb.no_planner(0.04, 0.1, 0, 0) 
            self.step = self.step+1  #step == 1

        except EOFError:
            print "Error!!!"


    def find_rect_object(self):

        print "Recognising objects ..."
        raw_input()
        self.step = self.step+1  #step == 2


    #preparation of leaving pose for later smoother navigation planners computation
    def leaving(self):

        try:
            rospy.sleep(2)
            #Reverse Driving for a distance
            print "Leaving from Station 1..."
            raw_input()
            self.mb.no_planner(3, -2, 0, 0) 

            #Rotate clockwise 90 degrees 
            self.mb.no_planner(0, 0, 1.57, -0.3)

            self.step = self.step+1  #step == 3

        except EOFError:
            print "Error!!!"


    def go_s2(self):

        try:
            #Station 2 Pose
            print "Move to Station 2..."
            raw_input()
            self.mb.move_to_point(0.265, -3.02, 1.5708) 
            self.mb.no_planner(0.12, 0.05, 0, 0) 
            self.step = self.step+1  #step == 4

        except EOFError:
            print "Error!!!"
            

    def leaving_2(self):

        try:
            rospy.sleep(2)
            #Reverse Driving for a distance
            print "Leaving from Station 2..."
            raw_input()
            self.mb.no_planner(3, -2, 0, 0) 

            #Rotate counter-clockwise 90 degrees 
            self.mb.no_planner(0, 0, 1.57, 0.3)

            self.step = self.step+1  #step == 5

        except EOFError:
            print "Error!!!"


    def go_s3(self):

        try:
            #Station 3 Pose
            print "Move to Station 3..."
            raw_input()
            self.mb.move_to_point(2.73, 3.2, -1.5708) 
            self.mb.no_planner(0.12, 0.05, 0, 0) 
            self.step = self.step+1  #step == 6

        except EOFError:
            print "Error!!!"


    def leaving_3(self):

        try:
            rospy.sleep(2)
            #Reverse Driving for a distance
            print "Leaving from Station 3..."
            raw_input()
            self.mb.no_planner(3, -2, 0, 0)
            #Rotate clockwise 90 degrees 
            self.mb.no_planner(0, 0, 1.57, -0.3)

            self.step = self.step+1  #step == 7

        except EOFError:
            print "Error!!!"


    def go_home(self):

        try:

            print "Going back home..."
            raw_input()
            #Home Position
            self.mb.move_to_point(0.12, 0, 0) 
            self.step = self.step+1  #step == 8

        except EOFError:
            print "Error!!!"

                
    def end_task(self):
        try:
            if self.mb.state :
                print "Reached HOME, program cycle ended. Thank you"
                self.step = self.step+1  #step == 9
        except EOFError:
            print "Error!!!"


    def control(self): 
        if self.step == 0:
            self.go_s1()
        elif self.step == 1:
            self.find_rect_object()
        elif self.step == 2:
            self.leaving()
        elif self.step == 3:
            self.go_s2()
        elif self.step == 4:
            self.leaving_2()
        elif self.step == 5:
            self.go_s3()
        elif self.step == 6:
            self.leaving_3()
        elif self.step == 7:
            self.go_home()
        elif self.step == 8:
            self.end_task()

    def carry_task(self):
        while not rospy.is_shutdown():
            try: 
                self.control()     
            except:
                pass
            self.r.sleep()

if __name__ == "__main__":
    program()