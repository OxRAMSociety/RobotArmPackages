#! /usr/bin/env python3

# Demo script for moving a pawn two spaces forward, must be run after add_chess_collision_object.py so that the piece can be picked up

import rospy
import numpy as np
from actionlib import SimpleActionClient
from rbx1_motion_planning.msg import executePoseGoalAction, executePoseGoalGoal
from rbx1_motion_planning.msg import executeHandGoalAction, executeHandGoalGoal
from rbx1_motion_planning.msg import attachObjectAction, attachObjectGoal
from rbx1_motion_planning.msg import detachObjectAction, detachObjectGoal
from tf.transformations import quaternion_about_axis, quaternion_multiply
import math
pi = math.pi

def OrientationFromVectorAndTwist(twist_angle, pointing_vector):
    # Calculate quaternion orientation from pointing vector and twist angle around this vector
    # Inputs - twist_angle - angle of twist around the pointing vector in raduans 
    #          pointing_vector - the 3D vector [x,y,z] that you want the robots hand to face in

    # define the quaternion_1 for rotation to the axis wanted
    pointing_vector = np.array(pointing_vector) * -1 # for some reason the robot always points in the opposite direction so this -1 fixes that
    unit_pointing_vector = pointing_vector/np.linalg.norm(pointing_vector)
    x = unit_pointing_vector[0]
    y = unit_pointing_vector[1]
    z = unit_pointing_vector[2]
    # Check if [x,y,z] == [-1,0,0] because if it does then the equation gives [0,0,0,0] as the quaternion which casues an error
    if [x,y,z] == [-1,0,0]:
        # This quaternion gives the facing direction of -x with the grippers horizontal, this is already normalised so can skip that
        non_unit_quaternion_1 = np.array([1,0,0,0]) 
    else:
        # equation comes from https://raw.org/proof/quaternion-from-two-vectors/ with i (unit vector in x direction) as the first vector
        non_unit_quaternion_1 = np.array([0, -z, y, 1+x])
        # Normalise the quaternion
        quaternion_1 = non_unit_quaternion_1/np.linalg.norm(non_unit_quaternion_1)
    # define the quaternion_2 for rotation around the axis wanted
    quaternion_2 = quaternion_about_axis(twist_angle, pointing_vector)
    # multiply the two to get the overall rotation
    quaternion = quaternion_multiply(quaternion_2, quaternion_1)
    return quaternion

def ExecutePoseGoal(pose_goal):
    client = SimpleActionClient('executePoseGoal_as', executePoseGoalAction)

    client.wait_for_server()

    goal = executePoseGoalGoal()
    rospy.loginfo("Empty Goal: %s" % goal)

    # set position goal
    goal.target.position.x = pose_goal[0]
    goal.target.position.y = pose_goal[1]
    goal.target.position.z = pose_goal[2]

    # make sure its a unit quaternion
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

def ExecuteHandGoal(hand_goal_1, hand_goal_2 = None):
    client = SimpleActionClient('executeHandGoal_as', executeHandGoalAction)

    client.wait_for_server()

    goal = executeHandGoalGoal()

    rospy.loginfo("Empty Goal: %s" % goal)

    # Check if 1 or 2 values have been given
    if hand_goal_2 is not None:
        # If both values have been given send them both
        goal.data = [hand_goal_1, hand_goal_2]
    else:
        # If only one value is given send it alone
        goal.data = [hand_goal_1]
    
    rospy.loginfo("Goal: %s" % goal)
    
    client.send_goal(goal)

    client.wait_for_result()

    result = client.get_result()

    return result

def name2coords(name):
    # A function to convert form a square name such as 'a1' to a coordinate relative to the centre of the board

    # Define constants
    board_width = 0.2 # in meters
    square_width = board_width/8 # in meters

    name = name.lower()
    letter = name[0]
    letter_coord = ord(letter) - 96
    number_coord = int(name[1])
    square_coords = np.array([letter_coord-4, number_coord-4]) # -4 is to account for the chess board being defined by its centre coordinates
    relative_coords = (square_coords-0.5) * square_width # -0.5 to get the middle of the square from the top right corner
    return relative_coords

if __name__ == '__main__':
    rospy.init_node('TestUserInputNode')

    result = detach_object()
    rospy.loginfo("Detach Result Received: %s", result)

    print("Type the square name. To end the program type end")

    while True:
        user_input_1 = input("Enter the square the piece is currently on: ") 
        user_input_2 = input("Enter the square to move to: ")
        if (user_input_1 == "End") | (user_input_1 == "end") | (user_input_1 == "END"):
            break
        elif len(user_input_1) == 2:
            relative_coords_1 = name2coords(user_input_1)
            




