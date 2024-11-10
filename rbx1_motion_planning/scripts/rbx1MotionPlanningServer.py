#! /usr/bin/env python3

# Questions:
    # Why do we need acceleration and velocity scaling factors? (I havent done the query_acc_vel function as I dont know why its needed)
        # Look up what the set_max_acceleration_scaling_factor() function does and velocity

    # Are the boxes only used to attach chess pieces to the robot hand when they are being moved 
    #   or are boxes also used to represent obstructions in the scene for object avoidance?
    # With the current code I think only 1 box can be created as its data is stored in single vairbales 
    #   e.g. self.box_name so if a new box is added this is overwritten

    # Error when using pose and position goal:
    #   Got a callback on a goalHandle that we're not tracking.
    # This doesnt seem to cause a problem, robot moves as expected
    # This may be due to the previous pose target not beign cleared properly? Althought the code should do this
    # I looked it up and a website said it may be becuase multiple clients are trying to use the server at the same 
    #   time however this shouldnt be the case with how I was using the code

    # What more needs to be added, this is up to date with the previous version (except quer_acc_vel)

# Notes:
    # Currently the script assumes all data is proveded as moveit wants it (e.g. assumes joint goals are given in radians)
    #   this can be changed once I know what format the goals will be given in

# This script sets up an action server for motion planing using moveit

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from actionlib import SimpleActionServer
from rbx1_motion_planning.msg import executeJointGoalAction, executeJointGoalFeedback, executeJointGoalResult, executeJointGoalGoal #executeJointGoalGoal only needed for testing
from rbx1_motion_planning.msg import executePoseGoalAction, executePoseGoalFeedback, executePoseGoalResult, executePoseGoalGoal #executePoseGoalGoal only needed for testing
from rbx1_motion_planning.msg import executePositionGoalAction, executePositionGoalFeedback, executePositionGoalResult, executePositionGoalGoal #executePositionGoalGoal only needed for testing

class robotMoveitCommander:
    # class containting the different ros nodes used to plan the motion given different inputs

    def __init__(self):
        # Store the data for the robot, scene, and joints
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_name = "arm"
        self.arm = moveit_commander.MoveGroupCommander(self.arm_name)
        self.hand_name = "gripper"
        self.hand = moveit_commander.MoveGroupCommander(self.hand_name)

        # Store the number of joints to know how many values need to be updated in the joint goal cb
        self.num_joints = len(self.arm.get_current_joint_values())

        # Create the ros publisher
        self.display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
        )

        # Get basic info
        # Get the name of the reference frame for the robot
        self.planning_frame = self.arm.get_planning_frame()
        print("============ Planning frame: %s" % self.planning_frame)

        # Get the name of the end effector link
        self.eef_link = self.arm.get_end_effector_link()
        print("============ End effector link: %s" % self.eef_link)

        # Get a list of the groups in the robot
        self.group_names = self.robot.get_group_names()
        print("============ Available Planning Groups: %s" % self.group_names)

        # Print the current state of the robot for debugging
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")

        # Start Action Servers
        # Start the joint goal action server
        self.executeJointGoal_as = SimpleActionServer("executeJointGoal_as", executeJointGoalAction, execute_cb = self.executeJointGoal_cb, auto_start=False)
        self.executeJointGoal_as.start()
        # Start the pose goal action server
        self.executePoseGoal_as = SimpleActionServer("executePoseGoal_as", executePoseGoalAction, execute_cb = self.executePoseGoal_cb, auto_start=False)
        self.executePoseGoal_as.start()
        # Start the position goal action server
        self.executePositionGoal_as = SimpleActionServer("executePositionGoal_as", executePositionGoalAction, execute_cb = self.executePositionGoal_cb, auto_start=False)
        self.executePositionGoal_as.start()

    def execute_joint_goal(self, joint_target):
        # Function to execute a movement when provided with a joint goal
        # Input: joint_target - an array of joint angles [joint1, joint2, ...], type: std_msgs/Float64MultiArray

        # Store the current joint values incase some of the joint targets are left blank, then the joint wont move
        joint_goal = self.arm.get_current_joint_values()
        # Update the joint goals from the current positions to the joint targets given
        for n in range(self.num_joints):
            joint_goal[n] = joint_target.data[n] # .data is needed as the ros Float64MultiArray also has other message info
        # Log the joint goal
        rospy.loginfo("Joint Goal: %s" % joint_goal)
        # Execute the movement and store the result (bool)
        outcome = self.arm.go(joint_goal, wait=True)
        # Call stop to ensure no residual movement
        self.arm.stop()
        # Return the outcome of the movement
        return outcome
    
    def executeJointGoal_cb(self, goal):
        # Callback function for the executeJointGoal action server

        # Log that a joint goal is trying to be executed
        rospy.loginfo("============ Executing Joint Goal")
        # Store whether the movement was successful or not (bool)
        result = self.execute_joint_goal(goal.target)

        # Check if the movement was successful
        if result:
            # Log that it was successful
            rospy.loginfo("SUCCESS - Executed Joint Goal!")
            # Set the result of the action server to be successful (use executeJointGoalResult() so that the result is structured as ros wants it)
            self.executeJointGoal_as.set_succeeded(executeJointGoalResult(result))
        else:
            # Log that it failed
            rospy.loginfo("FALIURE - Joint Goal Aborted!")
            # Set the result of the action server to be aborted (use executeJointGoalResult() so that the result is structured as ros wants it)
            self.executeJointGoal_as.set_aborted(executeJointGoalResult(result))

    def execute_pose_goal(self, pose_goal):
        # Function to execute a movement when provided with a pose goal
        # Input: pose_goal - a pose with cartesian coordinates (x,y,z) and a quaternion orientation (x,y,z vector and w angle), type: geometry_msgs/Pose 

        # Set the pose target to the pose goal provided
        self.arm.set_pose_target(pose_goal) # Old version set end effector link within this function call, tutorials dont do this
        # Log the target pose
        rospy.loginfo("Pose Goal: %s" % pose_goal)
        # Execute the movement and store the result (bool)
        outcome = self.arm.go(wait=True) # Goal not provided as it has alredy been set above
        # Call stop to ensure no residual movement
        self.arm.stop()
        # Clear the pose target
        self.arm.clear_pose_targets()
        # Return the outcome of the movement
        return outcome
    
    def executePoseGoal_cb(self, goal):
        # Callback function for the executePoseGoal action server

        # Log that a pose goal is trying to be executed
        rospy.loginfo("============ Executing Pose Goal")
        # Store whether the movement was successful or not (bool)
        result = self.execute_pose_goal(goal.target)

        # Check if the movement was successful
        if result:
            # Log that it was successful
            rospy.loginfo("SUCCESS - Executed Pose Goal!")
            # Set the result of the action server to be successful (use executePoseGoalResult() so that the result is structured as ros wants it)
            self.executePoseGoal_as.set_succeeded(executePoseGoalResult(result))
        else:
            # Log that it failed
            rospy.loginfo("FALIURE - Pose Goal Aborted!")
            # Set the result of the action server to be aborted (use executePoseGoalResult() so that the result is structured as ros wants it)
            self.executePoseGoal_as.set_aborted(executePoseGoalResult(result))

    def execute_position_goal(self, position_goal):
        # Function to execute a movement when provided with a position goal 
        # Input: position goal - array of cartesian coordinates [x,y,z], type: std_msgs/Float64MultiArray

        # ==================================================================
        # Comment from previous version:
        # Sets a Position Goal for a given set of inputs (in Cartesian coords.) and moves accordingly
		# I have found a lower bound for the locus of points to which the arm can always move, described by the following inequality:
		#				~~~ x^2 + (y + 0.2425)^2 + (z - 0.1864)^2 <= 0.19717 ~~~
		# If it's not working, check that your destination is achievable!
        # ==================================================================


        # Set the position target to the position goal provided
        # rospy.loginfo("position goal: %s" % position_goal)
        self .arm.set_position_target(position_goal.data) # .data is needed as the ros Float64MultiArray also has other message info
        # Old version also set the end effector link here, tutorials dont do this so I will leave it out and then put it in if it causes errors
        # Execute the movement and store the result (bool)
        outcome = self.arm.go(wait=True) # Goal not provided as it has alredy been set above
        # Call stop to ensure no residual movement
        self.arm.stop()
        # Clear the pose target, this also clears position targets (and orientation targets) as they are grouped in the same category in moveit
        self.arm.clear_pose_targets()
        # Return the outcome of the movement
        return outcome
    
    def executePositionGoal_cb(self, goal):
        # Callback function for the executePositionGoal action server

        # Log that a position goal is trying to be executed
        rospy.loginfo("============ Executing Position Goal")
        # Log the target position
        rospy.loginfo("Position Goal: %s" % goal)
        # Store whether the movement was successful or not (bool)
        result = self.execute_position_goal(goal.target)

        # Check if the movement was successful
        if result:
            # Log that it was successful
            rospy.loginfo("SUCCESS - Executed Position Goal!")
            # Set the result of the action server to be successful (use executePositionGoalResult() so that the result is structured as ros wants it)
            self.executePositionGoal_as.set_succeeded(executePositionGoalResult(result))
        else:
            # Log that it failed
            rospy.loginfo("FALIURE - Position Goal Aborted!")
            # Set the result of the action server to be aborted (use executePositionGoalResult() so that the result is structured as ros wants it)
            self.executePositionGoal_as.set_aborted(executePositionGoalResult(result))


    def print_state(self):
        # Function to print the current robot state

        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")

    def check(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Function to check if a box is in the scene and if its attached
        rospy.loginfo("============ Checking if scene updated correctly") # Only here for testing

        # Get starting time and create a varibale to store the elapsed time for timeout
        start = rospy.get_time()
        seconds = rospy.get_time()
        # While the time is less than the timeout and ros is still active
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is attached
            # Store all the attached objects with the current box name
            attached_objects = self.scene.get_attached_objects([self.box_name])
            # Check if there are any attached objects with the current box name
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene
            # (Attaching the box will remove it from known_objects)
            # Check if there are any boxes in the scenes known objects with the current box name
            is_known = self.box_name in self.scene.get_known_object_names()

            # Test if the current state is what we want it to be
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                # Log that the scene has updated
                rospy.loginfo("Scene updated as expected")
                # Success so return true
                return True
            
            # Sleep to give other threads time on the processor
            rospy.sleep(0.1)
            # Update the elapsed time
            seconds = rospy.get_time()
        
        # If the while loop is exited without returning then we timed out or ros was closed
        # This means that the state never got to what we wanted so return false
        # Failed so return false
        return False

    def add_box(self, name, box_size, position, orientation, timeout=4):
        # Function to add a box to the scene
        # Inputs:
            # name - the name of the box, type: string
            # box_size -  the box dimesions as an array [x,y,z], type: array length 3
            # position -  the box position as an array of cartesian coordinates [x,y,z], type: array length 3
            # orientation - the box orientation as a quaternion [x,y,z,w] (vector, angle), type: array length 4
            # timeout - the timeout of the check function, type: int

        rospy.loginfo("============ Adding Box")

        # Previous version had rospy.sleep(0.5) at the start, if problems with this occur try adding that

        # Set box_pose as a PoseStamped message type
        box_pose = geometry_msgs.msg.PoseStamped()
        # Set the frame ID to the name of the robot hand
        box_pose.header.frame_id = self.hand_name
        # Set the position and orientation of the box
        box_pose.pose.position.x = position[0]
        box_pose.pose.position.y = position[1]
        box_pose.pose.position.z = position[2]
        box_pose.pose.orientation.x = orientation[0]
        box_pose.pose.orientation.y = orientation[1]
        box_pose.pose.orientation.z = orientation[2]
        box_pose.pose.orientation.w = orientation[3]
        # Add the box to the scene
        self.scene.add_box(name, box_pose, box_size) # box_size should be given as an array [width, length, height]
        # Store the box name
        self.box_name = name
        # Wait for the box to be added and return if it was successful 
        return self.check(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Function that attaches a box to the robot

        rospy.loginfo("============ Attaching Box")

        # Collisions between the box and all links in touch_links are ignored so that they can be in contact, add the links used to pick up the box to this
        # Add the hand links to touch_links
        touch_links = self.robot.get_link_names(group=self.hand_name)
        # Attach the box to the robot 
        self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)
        # Wait for the box to attach and return if it was successful 
        return self.check(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        # Function that detaches a box from the robot

        rospy.loginfo("============ Detaching Box")
        # Detach the box
        self.scene.remove_attached_object(self.eef_link, name=self.box_name)
        # Wait for the box to detach and return if it was successful 
        return self.check(box_is_known=True, box_is_attached=False, timeout=timeout)
    
    def remove_box(self, timeout=4):
        # Function that removes a box from the scene

        rospy.loginfo("============ Removing Box")

        # Remove the object (must be detached before it can be removed)
        self.scene.remove_world_object(self.box_name)
        # Wait for the box to be removed and return if it was successful 
        return self.check(box_is_attached=False, box_is_known=False, timeout=timeout)

if __name__ == '__main__':
    # Setup moveit commander
    moveit_commander.roscpp_initialize(sys.argv)
    # Setup ROS node
    rospy.init_node("rbx1MotionPlanningServer", anonymous=True)
    # Create an instance of the class
    s = robotMoveitCommander()

    # Testing
    # s.add_box("box", [0.075,0.075,0.075], [0,0,0.11], [0,0,0,1])
    # s.attach_box()
    # goal1 = executeJointGoalGoal()
    # goal1.target = [1, 1, 1, -1, 1, 1, 1]
    # s.executeJointGoal_cb(goal1)
    # s.detach_box()
    # goal2 = executePositionGoalGoal()
    # goal2.target = [0.5, 0.2, 0.6]
    # s.executePositionGoal_cb(goal2)
    # s.remove_box()
    # s.executeJointGoal_cb(goal1)
    # END Testing

    rospy.spin()