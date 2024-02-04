#!/usr/bin/env python3


import rospy
import smach
from geometry_msgs.msg import Pose
import smach_ros
import tf2_ros

from GetPose import GetPose




#Define the idle state
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['Start',"Stay Idle"])
    
    def execute(self,userdata):
        user = input("Do you want to start the machine:(y/n) ")
        if user == "y":
            return "Start"
        else:
            return "Stay Idle"
    


#Define state check_target_location
class CheckTarget(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'],input_keys= ['pose','robot_pose'],output_keys= ['robot_pose'])
        
    
    def execute(self,userdata):
        rospy.loginfo("Getting Pose Data")
        userdata.robot_pose = userdata.pose
        rospy.loginfo('Robot orietation w: %d', userdata.robot_pose.orientation.w)
        return 'success'
        


if __name__ == '__main__':
    rospy.init_node("State_Machine_1.0")

    sm = smach.StateMachine(outcomes=['end'])

    # Open the container
    with sm:
        # Add states to the container
        # Provide the name of the state, the class from which it is derived,
        # and the next state for each possible outcome
        smach.StateMachine.add('Idle', Idle(), 
                               transitions={'Start':'GetPose', 'Stay Idle':'Idle'})
        smach.StateMachine.add('GetPose', GetPose(), 
                               transitions={'success':'CheckTarget'},
                               remapping = {'pose' : 'pose'})
        smach.StateMachine.add('CheckTarget',CheckTarget(),
                                transitions= {'success' : 'end'},
                                remapping = { 'pose' : 'pose',
                                              'robot_pose' : 'robot_pose'})

    # Execute SMACH plan
    outcome = sm.execute()

