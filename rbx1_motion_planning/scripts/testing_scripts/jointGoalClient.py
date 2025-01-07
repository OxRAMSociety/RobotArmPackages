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
    goal.target.data = [-1, 0, 0, 0, pi/2, 0]
    rospy.loginfo("Goal: %s" % goal)
    #1.0337636332601423, 1.2769740515619334, -0.855007766900004, 1.4386449271569612, 1.6142400117962017, 0.42305389743080846, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    
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