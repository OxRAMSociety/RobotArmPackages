#!/usr/bin/env python3


import rospy
import smach



from rbx1_GetPose import GetPose
from rbx1_movearmpose import MOVE_ARM
from geometry_msgs.msg import Pose



#Define the idle state
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['Start',"Stay Idle"])
        self.client =  SimpleActionClient('executePoseGoal_as',executePoseGoalAction)
        self.goal = executePoseGoalGoal()
    
    def execute(self,userdata):
        user = input("Do you want to start the machine:(y/n) ")
        if user == "y":
           
            init_pose = Pose()
            init_pose.position.x = p[0]
            init_pose.position.y = p[1]
            init_pose.position.z = p[2]
            init_pose.orentation.x = p[3]
            init_pose.orentation.y = p[4]
            init_pose.orentation.z = p[5]
            init_pose.orentation.w = p[6]

            rospy.loginfo('Getting target pose information')
        #Makes a service call to get the pose information 
            self.goal.target = self.request(init_pose)
            self.client.wait_for_server()
        #Makes an action call
            self.client.send_goal(self.goal)
  
            self.client.wait_for_result()

            result = self.client.result()
    
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

    sm = smach.StateMachine(outcomes=['success','fail'])

    # Open the container
    with sm:
        # Add states to the container
        # Provide the name of the state, the class from which it is derived,
        # and the next state for each possible outcome
        smach.StateMachine.add('Idle', Idle(), 
                               transitions={'Start':'GetPose', 'Stay Idle':'Idle'})
        smach.StateMachine.add('GetPose', GetPose(), 
                               transitions={'success':'GetPose'},
                               remapping = {'pose' : 'pose'})
        smach.StateMachine.add('MoveArm',MOVE_ARM(),
                               transitions= {'success' : 'success',
                                             'failure' : 'fail'})

      

    # Execute SMACH plan
    outcome = sm.execute()

