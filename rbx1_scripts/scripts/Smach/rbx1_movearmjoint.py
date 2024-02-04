#!/usr/bin/env python3

# This script provides an action server by which to control the arm using MoveIt

import sys
import rospy
import signal
import time
import smach


from moveit_msgs.msg import DisplayTrajectory
from math import pi
import math
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from actionlib import SimpleActionClient

from rbx1_scripts.msg import executeJointGoalAction, executeJointGoalGoal

class JOINT_GOAL(smach.State):

    def __init__(self):
        smach.State.__init__(self,outcomes=['success','failure'],input_keys=['target_joint'])

    def execute(self, userdata):
        #Create an action client 
        joint_client = SimpleActionClient('executeJointGoal_as',executeJointGoalAction)
        joint_client.wait_for_server()
        #Create an action goal object 
        goal = executeJointGoalGoal()
        goal.target = userdata.target_joint
        #Maka an action call and wait for result 
        joint_client.send_goal(goal)
        joint_client.wait_for_result()

        joint_result = joint_client.result()

        if joint_result:
            return 'success'
        
        else:
            return "failure"




        
        