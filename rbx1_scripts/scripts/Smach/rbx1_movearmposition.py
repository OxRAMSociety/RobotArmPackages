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

from rbx1_scripts.msg import executePositionGoalAction, executePositionGoalGoal

class POS_GOAL(smach.State):
    
    def __init__(self):
        smach.State.__init__(self,outcomes=['success','failure'],input_keys=['target_pos'])
    
    def execute(self, userdata):
        #Create an action client 
        pos_client = SimpleActionClient('executeJointGoal_as',executePositionGoalAction)
        pos_client.wait_for_server()
        #Create an action goal object 
        goal = executePositionGoalGoal()
        goal.target = userdata.target_pos
        #Maka an action call and wait for result 
        pos_client.send_goal(goal)
        pos_client.wait_for_result()

        pos_result = pos_client.result()

        if pos_result:
            return 'success'
        
        else:
            return "failure"

        

    