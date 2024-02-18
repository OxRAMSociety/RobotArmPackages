#!/usr/bin/env python3

import rospy
from rbx1_scripts.msg import executePositionGoalAction, executePositionGoalFeedback, executePositionGoalResult,executePositionGoalGoal, executePoseGoalAction, executePoseGoalGoal
import actionlib
from std_msgs.msg import Float64MultiArray

if __name__ == '__main__':
    rospy.init_node('position_goal_client')

    # Create an action client for the "executePositionGoal_as" action server
    # position_ac = actionlib.SimpleActionClient("executePositionGoal_as", executePositionGoalAction)
    pose_ac = actionlib.SimpleActionClient("executePoseGoal_as", executePoseGoalAction)

    # position_ac.wait_for_server()
    pose_ac.wait_for_server()

    # Prompt the user for the goal coordinates
    # position = Float64MultiArray()
    # position_goal = executePositionGoalGoal()
    # position_goal.target.data.append( float(input('Enter the x-coordinate of the goal position: ')))
    # position_goal.target.data.append( float(input('Enter the y-coordinate of the goal position: ')))
    # position_goal.target.data.append( float(input('Enter the z-coordinate of the goal position: ')))

    pose_goal = executePoseGoalGoal()
    print(str(pose_goal))
    pose_goal.target.position.x = float(input('Enter the x-coordinate of the goal position: '))
    pose_goal.target.position.y = float(input('Enter the y-coordinate of the goal position: '))
    pose_goal.target.position.z = float(input('Enter the z-coordinate of the goal position: '))
    pose_goal.target.orientation.x = 0.0
    pose_goal.target.orientation.y = 0.0
    pose_goal.target.orientation.z = 0.0
    pose_goal.target.orientation.w = 1.0



    # Create a goal message
    #goal = executePoseGoalGoal()
    #goal.goal_pose = pose_goal

    # Send the goal to the action server and wait for a result
    # position_ac.send_goal(position_goal)
    # position_ac.wait_for_result()
    pose_ac.send_goal(pose_goal)
    pose_ac.wait_for_result()


    # Print the result
    # result = position_ac.get_result()
    # rospy.loginfo('Result: {}'.format(result))

    result = pose_ac.get_result()
    rospy.loginfo('Result: {}'.format(result))
