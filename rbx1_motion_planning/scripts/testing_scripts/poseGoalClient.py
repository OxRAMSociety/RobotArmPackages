#! /usr/bin/env python3

# An action client to test the executePoseGoal_as action server

import rospy
import math
pi = math.pi

from actionlib import SimpleActionClient
from rbx1_motion_planning.msg import executePoseGoalAction, executePoseGoalGoal

def call_server():
    client = SimpleActionClient('executePoseGoal_as', executePoseGoalAction)

    client.wait_for_server()

    goal = executePoseGoalGoal()
    #rospy.loginfo("Empty Goal: %s" % goal)
    goal.target.position.x = -0.5
    goal.target.position.y = -0.5
    goal.target.position.z = 0.2
    goal.target.orientation.x = 0
    goal.target.orientation.y = 1
    goal.target.orientation.z = 0
    goal.target.orientation.w = 1
    #rospy.loginfo("Goal: %s" % goal)
    
    client.send_goal(goal)

    client.wait_for_result()

    result = client.get_result()

    return result

if __name__ == '__main__':

    try:
        rospy.init_node('executePoseGoalClientNode')
        result =  call_server()
        rospy.loginfo("Result Received: %s", result)
    except rospy.ROSInterruptException as e:
        rospy.logerr('Something went wrong: %s', e)