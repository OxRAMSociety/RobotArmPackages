#!/usr/bin/env python3

# This script provides an action server by which to control the arm using MoveIt

import sys
import rospy
import signal
import time


import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from math import pi
import math
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from actionlib import SimpleActionServer
from rbx1_scripts.msg import executeJointGoalAction, executeJointGoalFeedback, executeJointGoalResult
from rbx1_scripts.msg import executePoseGoalAction, executePoseGoalFeedback, executePoseGoalResult
from rbx1_scripts.msg import executePositionGoalAction, executePositionGoalFeedback, executePositionGoalResult


class rbx1MoveItCommander:

	def __init__(self):
		# Setup MoveIt commander essentials
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.arm = moveit_commander.MoveGroupCommander("arm")
		self.arm_frame = self.arm.get_planning_frame()
		rospy.loginfo("============ Reference frame: %s", self.arm_frame)
		self.gripper = moveit_commander.MoveGroupCommander("gripper")
		self.gripper_frame = self.gripper.get_planning_frame()
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size = 20)

		# Get basic info
		self.eef_link = self.arm.get_end_effector_link()
		rospy.loginfo("============ End Effector: %s", self.eef_link)
		self.planning_frame = self.arm.get_planning_frame()
		self.group_names = self.robot.get_group_names()
		rospy.loginfo("============ Robot Groups: %s", self.group_names)

		# Start Action Servers
		self.executeJointGoal_as = SimpleActionServer("executeJointGoal_as", executeJointGoalAction, execute_cb = self.executeJointGoal_cb, auto_start = False)
		self.executeJointGoal_as.start()

		self.executePoseGoal_as = SimpleActionServer("executePoseGoal_as", executePoseGoalAction, execute_cb = self.executePoseGoal_cb, auto_start = False)
		self.executePoseGoal_as.start()

		self.executePositionGoal_as = SimpleActionServer("executePositionGoal_as", executePositionGoalAction, execute_cb = self.executePositionGoal_cb, auto_start = False)
		self.executePositionGoal_as.start()


	# Functions

	def query_acc_vel(self):
		# Prompts the user for the max. acceleration and velocity scaling factors
		a_scale = float(input("Max. Acceleration Scaling Factor? (0.1<=a<=1): "))
		if a_scale < 0.1:
			a_scale = 0.1
		elif a_scale > 1:
			a_scale = 1
		self.arm.set_max_acceleration_scaling_factor(a_scale)

		v_scale = float(input("Max. Velocity Scaling Factor? (0.1<=v<=1): "))
		if v_scale < 0.1:
			v_scale = 0.1
		elif v_scale > 1:
			v_scale = 1
		self.arm.set_max_velocity_scaling_factor(v_scale)


	def check(self, box_is_known=False, box_is_attached=False, timeout=4):
		# Checks that box correctly attaches or detaches
		start = rospy.get_time()
		seconds = rospy.get_time()
		while (seconds - start < timeout) and not rospy.is_shutdown():
			# Test if the box is in attached objects
			attached_objects = self.scene.get_attached_objects([self.box_name])
			is_attached = len(attached_objects.keys()) > 0

			# Test if the box is in the scene.
			# Note that attaching the box will remove it from known_objects
			is_known = self.box_name in self.scene.get_known_object_names()

			# Test if we are in the expected state
			if (box_is_attached == is_attached) and (box_is_known == is_known):
				return True
				

			# Sleep so that we give other threads time on the processor
			rospy.sleep(0.1)
			seconds = rospy.get_time()

		# If we exited the while loop without returning then we timed out
		return False


	def execute_joint_goal(self, joint_target):
		# Sets Joint Goals for a given set of inputs (in degrees) and moves accordingly
		joint_goal = self.arm.get_current_joint_values()
		joint_goal[0] = math.radians(joint_target[0])
		joint_goal[1] = math.radians(joint_target[1])
		joint_goal[2] = math.radians(joint_target[2])
		joint_goal[3] = math.radians(joint_target[3])
		joint_goal[4] = math.radians(joint_target[4])
		joint_goal[5] = math.radians(joint_target[5])
		rospy.loginfo(joint_goal)

		# `go()` returns a boolean indicating whether the planning and execution was successful.
		success = self.arm.go(joint_goal, wait=True)
		# Calling `stop()` ensures that there is no residual movement
		self.arm.stop()
		return success


	def executeJointGoal_cb(self, goal):
		rospy.loginfo("============ Executing Joint Goal")
		result = self.execute_joint_goal(goal.target)

		if result:
			rospy.loginfo("SUCCESS - Executed Joint Goal!")
			self.executeJointGoal_as.set_succeeded(executeJointGoalResult(result))
		else:
			rospy.logerr("FAILURE - Joint Goal Failed!")
			self.executeJointGoal_as.set_aborted(executeJointGoalResult(result))


	def execute_pose_goal(self, pose_goal):
		# Sets a Pose Goal for a given set of inputs (in Cartesian coords.) and moves accordingly
		# I have found a lower bound for the locus of points to which the arm can always move, described by the following inequality:
		#				~~~ x^2 + (y + 0.2425)^2 + (z - 0.1864)^2 <= 0.19717 ~~~
		# If it's not working, check that your destination is achievable!
		self.arm.set_pose_target(pose_goal, end_effector_link = self.eef_link)
		rospy.loginfo(pose_goal)
		
		# `go()` returns a boolean indicating whether the planning and execution was successful.
		success = self.arm.go(wait=True)
		# Calling `stop()` ensures that there is no residual movement
		self.arm.stop()
		# It is always good to clear your targets after planning with poses.
		self.arm.clear_pose_targets()
		return success


	def executePoseGoal_cb(self, goal):
		rospy.loginfo("============ Executing Pose Goal")
		result = self.execute_pose_goal(goal.target)

		if result:
			rospy.loginfo("SUCCESS - Executed Pose Goal!")
			self.executePoseGoal_as.set_succeeded(executePoseGoalResult(result))
		else:
			rospy.logerr("FAILURE - Pose Goal Failed!")
			self.executePoseGoal_as.set_aborted(executePoseGoalResult(result))


	def execute_position_goal(self, position_goal):
		# Sets a Position Goal for a given set of inputs (in Cartesian coords.) and moves accordingly
		# I have found a lower bound for the locus of points to which the arm can always move, described by the following inequality:
		#				~~~ x^2 + (y + 0.2425)^2 + (z - 0.1864)^2 <= 0.19717 ~~~
		# If it's not working, check that your destination is achievable!
		self.arm.set_position_target(position_goal.data, end_effector_link = self.eef_link)
		rospy.loginfo(position_goal)

		# `go()` returns a boolean indicating whether the planning and execution was successful.
		success = self.arm.go(wait=True)
		# Calling `stop()` ensures that there is no residual movement
		self.arm.stop()
		# It is always good to clear your targets after planning with poses.
		self.arm.clear_pose_targets()
		return success


	def executePositionGoal_cb(self, goal):
		rospy.loginfo("============ Executing Position Goal")
		result = self.execute_position_goal(goal.target)

		if result:
			rospy.loginfo("SUCCESS - Executed Position Goal!")
			self.executePositionGoal_as.set_succeeded(executePositionGoalResult(result))
		else:
			rospy.logerr("FAILURE - Position Goal Failed!")
			self.executePositionGoal_as.set_aborted(executePositionGoalResult(result))


	def print_state(self):
		# Prints the current robot state
		rospy.loginfo("============ Printing Robot State:")
		rospy.loginfo(self.robot.get_current_state())
		rospy.loginfo("")


	def spawn_box(self, name, box_size, position, orientation):
		# Spawns the box into the scene
		rospy.sleep(0.5)
		box_pose = geometry_msgs.msg.PoseStamped()
		box_pose.header.frame_id = self.gripper_frame
		box_pose.pose.position.x = position[0]
		box_pose.pose.position.y = position[1]
		box_pose.pose.position.z = position[2]
		box_pose.pose.orientation.x = orientation[0]
		box_pose.pose.orientation.y = orientation[1]
		box_pose.pose.orientation.z = orientation[2]
		box_pose.pose.orientation.w = orientation[3]
		self.scene.add_box(name, box_pose, size=(box_size, box_size, box_size))
		self.box_name = name
		self.check(box_is_known=True)


	def attach_box(self, name):
		# Attaches the box to the gripper
		touch_links = self.robot.get_link_names(group = "gripper")
		self.scene.attach_box(self.eef_link, name, touch_links=touch_links)
		self.check(box_is_attached=True,box_is_known=False)


	def detach_box(self, name):
		# Detaches the box from the gripper
		self.scene.remove_attached_object(self.eef_link, name)
		self.check(box_is_known=True,box_is_attached=False)


	def remove_box(self, name):
		# Removes the box from the scene
		self.scene.remove_world_object(name)
		self.check(box_is_attached=False,box_is_known=False)


if __name__ == '__main__':

    # Setup ROS node and MoveIt commander
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('rbx1MoveItCommander', anonymous=True)

	s = rbx1MoveItCommander()
	s.print_state()
	s.query_acc_vel()

	rospy.spin()
