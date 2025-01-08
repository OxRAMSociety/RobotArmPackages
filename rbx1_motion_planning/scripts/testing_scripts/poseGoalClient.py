#! /usr/bin/env python3

# An action client to test the executePoseGoal_as action server

# Notes:
# THE FOLLOWING DATA IS UNTESTED AND OUT OF DATE BUT KEPT TO BE CHECKED
# Vertically downwards quaternions:
# goal.target.orientation.x = -0.5
# goal.target.orientation.y = -0.5
# goal.target.orientation.z = -0.5
# goal.target.orientation.w = 0.5
#
#x = 0.43, y = -0.56, z = 0.43, w = 0.56
#
# These two dont work?
# goal.target.orientation.x = 0.707
# goal.target.orientation.y = 0
# goal.target.orientation.z = 0.707
# goal.target.orientation.w = 0
#
# goal.target.orientation.x = 0
# goal.target.orientation.y = -0.707
# goal.target.orientation.z = 0
# goal.target.orientation.w = 0.707
#

# For position on +ve y axis, x=0, and grippers aligned with the x axis
# Orientation (quaternion): x = 0, y = -0.707, z = 0, w = 0.707

# THE FOLLOWING DATA IS TESTED UNDER THE NEW CONFIGURATION
# Quaternion: [x,y,z,w] Facing direction: (+-)(xyz) Gripper orientation: xyz (explination of orientation)
# [1, 0, 0, 0] -x y (horizontal)
# [0, 1, 0, 0] +x y (horizontal)
# [0, 0, 1, 0] +x y (horizontal)
# [0, 0, 0, 1] -x y (horizontal)
# [0.707, 0, 0, 0.707] -x z (vertical)
# [0, 0.707, 0.707, 0] +x z (vertical)
# [0, 0.707, 0, 0.707] +z y 
# [-0.707, 0.707, 0, 0] +y x (horizontal)
# [0, -0.707, 0, 0.707] -z y
# [0, -0.707, 0.707, 0] +x z (vertical)
# [0, 0, -0.707, 0.707] +y x (horizontal)
# [0.5, 0.5, 0.5, 0.5] -y z (vertical)
# [0.5, -0.5, 0.5, 0.5] -z x
# [0.5, 0.5, 0.5, -0.5] -z x
# [0.5, 0.5, -0.5, 0.5] +z x
# [0.707, 0, 0.707, 0] -z y
# [0.271, -0.653, 0.271, 0.653] -z y=-x
# [-0.271, -0.653, -0.271, 0.653] -z y=x

# Most useful orientations:
# [0.5, -0.5, 0.5, 0.5] -z x
# [0.5, 0.5, 0.5, -0.5] -z x
# [0, -0.707, 0, 0.707] -z y
# [0.707, 0, 0.707, 0] -z y
# [0.271, -0.653, 0.271, 0.653] -z y=-x
# [-0.271, -0.653, -0.271, 0.653] -z y=x


import rospy
import numpy as np
import math
pi = math.pi

from tf.transformations import quaternion_from_euler
from actionlib import SimpleActionClient
from rbx1_motion_planning.msg import executePoseGoalAction, executePoseGoalGoal

def call_server():
    client = SimpleActionClient('executePoseGoal_as', executePoseGoalAction)

    client.wait_for_server()

    goal = executePoseGoalGoal()
    #rospy.loginfo("Empty Goal: %s" % goal)
    goal.target.position.x = 0.22 + 0.0875
    goal.target.position.y = 0 - 0.0875
    goal.target.position.z = 0.15

    # quaternion from euler
    # roll_angle = 0
    # pitch_angle = 3*pi/4
    # yaw_angle = 3*pi/4
    # quaternion = quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)
    
    # quaternion directly
    npquaternion = np.array([-0.271, -0.653, -0.271, 0.653])
    npquaternion = npquaternion/np.linalg.norm(npquaternion)
    quaternion = npquaternion.tolist()

    # rospy.loginfo("Quaternion: %s" % quaternion)

    goal.target.orientation.x = quaternion[0]
    goal.target.orientation.y = quaternion[1]
    goal.target.orientation.z = quaternion[2]
    goal.target.orientation.w = quaternion[3]
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