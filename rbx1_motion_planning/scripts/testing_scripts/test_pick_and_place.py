#! /usr/bin/env python3

# Demo script for moving a pawn two spaces forward, must be run after add_chess_collision_object.py so that the piece can be picked up

import rospy
import numpy as np
from actionlib import SimpleActionClient
from rbx1_motion_planning.msg import executePoseGoalAction, executePoseGoalGoal
from rbx1_motion_planning.msg import attachObjectAction, attachObjectGoal
from rbx1_motion_planning.msg import detachObjectAction, detachObjectGoal

def ExecutePoseGoal(pose_goal):
    client = SimpleActionClient('executePoseGoal_as', executePoseGoalAction)

    client.wait_for_server()

    goal = executePoseGoalGoal()
    rospy.loginfo("Empty Goal: %s" % goal)

    # set position goal
    goal.target.position.x = pose_goal[0]
    goal.target.position.y = pose_goal[1]
    goal.target.position.z = pose_goal[2]

    # make unit quaternion
    npquaternion = np.array(pose_goal[3:7])
    npquaternion = npquaternion/np.linalg.norm(npquaternion)
    quaternion = npquaternion.tolist()

    # set orientation goal
    goal.target.orientation.x = quaternion[0]
    goal.target.orientation.y = quaternion[1]
    goal.target.orientation.z = quaternion[2]
    goal.target.orientation.w = quaternion[3]

    # Log the goal
    rospy.loginfo("Goal: %s" % goal)
    
    client.send_goal(goal)

    client.wait_for_result()

    result = client.get_result()

    return result

def attach_object(object_name):
    client = SimpleActionClient('attachObject_as', attachObjectAction)

    client.wait_for_server()

    goal = attachObjectGoal()
    rospy.loginfo("Empty Goal: %s" % goal)

    goal.object_name = object_name

    # Log the goal
    rospy.loginfo("Goal: %s" % goal)

    client.send_goal(goal)

    client.wait_for_result()

    result = client.get_result()

    return result

def detach_object():
    client = SimpleActionClient('detachObject_as', detachObjectAction)

    client.wait_for_server()

    goal = detachObjectGoal()

    client.send_goal(goal)

    client.wait_for_result()

    result = client.get_result()

    return result


if __name__ == '__main__':
    rospy.init_node('TestPickAndPlaceNode')

    result = detach_object()
    rospy.loginfo("Detach Result Received: %s", result)

    rospy.sleep(0.5)

    result = ExecutePoseGoal([0.22 + 0.0875, 
                              -0.0625,
                              0.15,
                              0.271, -0.653, 0.271, 0.653])
    rospy.loginfo("Move Above Result Received: %s", result)

    rospy.sleep(0.5)

    result = ExecutePoseGoal([0.22 + 0.0875, 
                              -0.0625,
                              0.10,
                              0.271, -0.653, 0.271, 0.653])
    rospy.loginfo("Move Down Result Received: %s", result)

    rospy.sleep(0.5)

    result = attach_object("black_pawn_1")
    rospy.loginfo("Attach Result Received: %s", result)

    rospy.sleep(0.5)

    result = ExecutePoseGoal([0.22 + 0.0875, 
                              -0.0625,
                              0.15,
                              0.271, -0.653, 0.271, 0.653])
    rospy.loginfo("Move Up Result Received: %s", result)

    rospy.sleep(0.5)
    
    result = ExecutePoseGoal([0.22 + 0.0875, 
                              -0.0125,
                              0.15,
                              0.5, -0.5, 0.5, 0.5])
    rospy.loginfo("Move Across Result Received: %s", result)

    rospy.sleep(0.5)
    
    result = ExecutePoseGoal([0.22 + 0.0875, 
                              -0.0125,
                              0.10,
                              0.5, -0.5, 0.5, 0.5])
    rospy.loginfo("Move Down Result Received: %s", result)

    rospy.sleep(0.5)

    result = detach_object()
    rospy.loginfo("Detach Result Received: %s", result)

    rospy.sleep(0.5)
    
    result = ExecutePoseGoal([0.22 + 0.0875, 
                              -0.0125,
                              0.15,
                              0.5, -0.5, 0.5, 0.5])
    rospy.loginfo("Move Up Result Received: %s", result)