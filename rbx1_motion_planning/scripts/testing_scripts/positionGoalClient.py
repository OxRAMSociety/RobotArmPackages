#! /usr/bin/env python3

# An action client to test the executePositionGoal_as action server

import rospy
import math
pi = math.pi

from actionlib import SimpleActionClient
from rbx1_motion_planning.msg import executePositionGoalAction, executePositionGoalGoal

def call_server():
    client = SimpleActionClient('executePositionGoal_as', executePositionGoalAction)

    client.wait_for_server()

    goal = executePositionGoalGoal()
    #rospy.loginfo("Empty Goal: %s" % goal)
    goal.target.data = [0.1, 0.1, 0.1]
    #rospy.loginfo("Goal: %s" % goal)
    
    client.send_goal(goal)

    client.wait_for_result()

    result = client.get_result()

    return result

if __name__ == '__main__':

    try:
        rospy.init_node('executePositionGoalClientNode')
        result =  call_server()
        rospy.loginfo("Result Received: %s", result)
    except rospy.ROSInterruptException as e:
        rospy.logerr('Something went wrong: %s', e)