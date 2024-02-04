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

from rbx1_scripts.msg import executePoseGoalAction, executePoseGoalGoal, executePoseGoalResult
from rbx1_scripts.srv import PoseService




class MOVE_ARM(smach.State):
    
    def __init__(self):
        smach.State.__init__(self,outcomes = ['success','failure'], input_keys = ['target_location'])
        self.request = rospy.ServiceProxy('PoseService', PoseService)
        self.client =  SimpleActionClient('executePoseGoal_as',executePoseGoalAction)
        self.goal = executePoseGoalGoal()

    def execute(self,userdata):
        rospy.loginfo('Getting target pose information')
        #Makes a service call to get the pose information 
        self.goal.target = self.request(userdata.target_location)
        self.client.wait_for_server()
        #Makes an action call
        self.client.send_goal(self.goal)

        self.client.wait_for_result()

        result = self.client.result()

        if result:
            return 'success'
        
        else:
            return 'failure'

