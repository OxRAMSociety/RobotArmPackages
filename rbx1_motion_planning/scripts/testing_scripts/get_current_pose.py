#! /usr/bin/env python3

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander

# Initialize the ROS node
rospy.init_node('get_current_pose_example')

# Initialize MoveIt Commander
robot = RobotCommander()
scene = PlanningSceneInterface()
group = MoveGroupCommander("arm") 

# Get the current pose of the end effector
current_pose = group.get_current_pose()

# Print the current pose (position and orientation)
print("Current Pose of the End Effector:")
print("Position: x = {:.2f}, y = {:.2f}, z = {:.2f}".format(
    current_pose.pose.position.x,
    current_pose.pose.position.y,
    current_pose.pose.position.z
))
print("Orientation (quaternion): x = {:.2f}, y = {:.2f}, z = {:.2f}, w = {:.2f}".format(
    current_pose.pose.orientation.x,
    current_pose.pose.orientation.y,
    current_pose.pose.orientation.z,
    current_pose.pose.orientation.w
))