#!/usr/bin/env python3

"""A function that takes the board grid names and the board origin and converts it to poses in the transform
reference frame"""

import string
import rospy
# import smach
from geometry_msgs.msg import Pose
import tf2_ros

def board_to_coord(board_pos): #board_origin is the transform for the tag on the chess board from tf
    # coord = Pose()
    gridsize = 27.1 # SPecifies the board grid size in mm
    if type(board_pos) != str:
        raise TypeError("input should be a string, e.g. A8")
    else:
        x = (ord((board_pos[0].upper())) - ord("A"))*gridsize + gridsize/2 + 60
        y = (int(board_pos[1]) - 1)*gridsize + gridsize/2 + 66.5
        # coord.position.z = board_origin.position.z
        # coord.orientation.x = board_origin.orientation.x
        # coord.orientation.y = board_origin.orientation.y
        # coord.orientation.z = board_origin.orientation.z
        # coord.orientation.w = board_origin.orientation.w
        # x = (ord((board_pos[0].upper())) - ord("A"))*gridsize + gridsize/2
        # y = (int(board_pos[1]) - 1)*gridsize + gridsize/2
    return [x,y]

rospy.init_node('static_grid_tf')

letters = "abcdefgh"
numbers = "12345678"
tfs = []
br = tf2_ros.StaticTransformBroadcaster()
for i in range(8):
    for j in range(8):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "april_tag"
        t.child_frame_id = letters[i] + numbers[j]
        t.transform.translation.x = board_to_coord(letters[i] + numbers[j])[0]
        t.transform.translation.y = board_to_coord(letters[i] + numbers[j])[1]
        tfs.append(t)
br.sendTransform(tfs)


