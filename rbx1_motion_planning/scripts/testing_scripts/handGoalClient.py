#! /usr/bin/env python3

# An action client to test the executeHandGoal_as action server
# Currently doesnt work, suspected to be because the hand is to restricted in the model

import rospy
import math
import numpy as np
pi = math.pi

from actionlib import SimpleActionClient
from rbx1_motion_planning.msg import executeHandGoalAction, executeHandGoalGoal

def call_server():
    client = SimpleActionClient('executeHandGoal_as', executeHandGoalAction)

    client.wait_for_server()

    goal = executeHandGoalGoal()
    rospy.loginfo("Empty Goal: %s" % goal)
    goal.target = 1.5
    rospy.loginfo("Goal: %s" % goal)
    
    client.send_goal(goal)

    client.wait_for_result()

    result = client.get_result()

    return result

if __name__ == '__main__':

    try:
        rospy.init_node('executeHandGoalClientNode')
        result =  call_server()
        rospy.loginfo("Result Received: %s", result)
    except rospy.ROSInterruptException as e:
        rospy.logerr('Something went wrong: %s', e)