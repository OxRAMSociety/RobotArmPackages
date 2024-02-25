#! /usr/bin/env python3

# An action client to test the executeJointGoal_as action server

import rospy
import math
import numpy as np
pi = math.pi

from actionlib import SimpleActionClient
from rbx1_motion_planning.msg import executeJointGoalAction, executeJointGoalGoal

def call_server():
    client = SimpleActionClient('executeJointGoal_as', executeJointGoalAction)

    client.wait_for_server()

    goal = executeJointGoalGoal()
    rospy.loginfo("Empty Goal: %s" % goal)
    goal.target.data = [1, 1, 1, 1, 1, 1]
    rospy.loginfo("Goal: %s" % goal)
    
    client.send_goal(goal)

    client.wait_for_result()

    result = client.get_result()

    return result

if __name__ == '__main__':

    try:
        rospy.init_node('example_action_client_node')
        result =  call_server()
        rospy.loginfo("Result Received: %s", result)
    except rospy.ROSInterruptException as e:
        rospy.logerr('Something went wrong: %s', e)