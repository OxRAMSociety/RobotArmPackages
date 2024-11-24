#!/usr/bin/env python3
'''
Here we define the FSM for the robot. Currently it only has 3 states: Idle --> GetPose --> MoveArm. 
Idle state receives the start robot signal, GetPose gets the target for the next move and find the target's pose, MoveArm moves the arm to the Pose that is identified by GetPose. 
'''

import rospy
import smach

from rbx1_GetPose import GetPose
from rbx1_movearmpose import MOVE_ARM

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

if __name__ == '__main__':
    rospy.init_node("State_Machine_1.0")

    sm = smach.StateMachine(outcomes=['success','fail'])

    # Open the container
    with sm:
        # Add states to the container
        # Provide the name of the state, the class from which it is derived,
        # and the next state for each possible outcome
        smach.StateMachine.add('Idle', Idle(), 
                               transitions={'Start':'GetPose', 'Stay Idle':'Idle'})
        smach.StateMachine.add('GetPose', GetPose(), 
                               transitions={'success':'MoveArm'},
                               remapping = {'pose' : 'pose'})
        smach.StateMachine.add('MoveArm',MOVE_ARM(),
                               transitions= {'success' : 'success',
                                             'failure' : 'fail'})

      

    # Execute SMACH plan
    outcome = sm.execute()

