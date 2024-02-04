#!/usr/bin/env python3

import rospy
from rbx1_scripts.msg import executePositionGoalAction, executePositionGoalFeedback, executePositionGoalResult,executePositionGoalGoal
import actionlib
from std_msgs.msg import Float64MultiArray

if __name__ == '__main__':
    rospy.init_node('position_goal_client')

    # Create an action client for the "executePositionGoal_as" action server
    client = actionlib.SimpleActionClient("executePositionGoal_as", executePositionGoalAction)
    client.wait_for_server()

    # Prompt the user for the goal coordinates
    # position = Float64MultiArray()
    # position[0]
    position_goal = executePositionGoalGoal()
    position_goal.target.data.append( float(input('Enter the x-coordinate of the goal position: ')))
    position_goal.target.data.append( float(input('Enter the y-coordinate of the goal position: ')))
    position_goal.target.data.append( float(input('Enter the z-coordinate of the goal position: ')))

    # Create a goal message
    #goal = executePoseGoalGoal()
    #goal.goal_pose = pose_goal

    # Send the goal to the action server and wait for a result
    client.send_goal(position_goal)
    client.wait_for_result()

    # Print the result
    result = client.get_result()
    rospy.loginfo('Result: {}'.format(result))
