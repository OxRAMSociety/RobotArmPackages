#!/usr/bin/env python3

# This script provides an action server by which to control the arm using MoveIt

import sys
import rospy
import signal
import time
import smach


import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from math import pi
import math
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from actionlib import SimpleActionClient

from rbx1_scripts.msg import executePoseGoalAction, executePoseGoalGoal, executePoseGoalResult




class MOVE_ARM(smach.State):
    
    def __init__(self):
        smach.State.__init__(self,outcomes = ['success','failure'], input_keys = ['target_pose'])
    
    def execute(self,userdata):
        client = SimpleActionClient('executePoseGoal_as',executePoseGoalAction)
        client.wait_for_server()
        goal = executePoseGoalGoal()
        goal.target = userdata.target_pose
        client.send_goal(goal)

        client.wait_for_result()

        result = client.result()

        if result:
            return 'success'
        
        else:
            return 'failure'

