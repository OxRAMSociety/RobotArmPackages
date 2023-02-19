#!/usr/bin/env python3


import rospy
import smach
from geometry_msgs.msg import Pose
import smach_ros
import tf2_ros

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
    

#Define the state for getting pose
class GetPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['success'],output_keys = ["pose"])
    
    def execute(self,userdata):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        #To do add timer
        trans = tfBuffer.lookup_transform(map, "tag1", rospy.Time())
        pose = Pose()
        pose.position.x = trans.transform.translation.x
        pose.position.y = trans.transform.translation.y
        pose.position.z = trans.transform.translation.z
        pose.orientation.x = trans.transform.translation.x
        pose.orientation.y = trans.transform.translation.y
        pose.orientation.z = trans.transform.translation.z
        pose.orientation.w = trans.transform.translation.w
        rospy.loginfo("Robot y position: %d", pose.position.y)
        



#Define state check_target_location
class CheckTarget(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure'])
        self.counter = 0
    
    def execute(self,userdata):
        rospy.loginfo("Checking Target")
        if self.counter < 3:
            self.counter +=1;
            return "success"
        else:
            return "failure"
        


if __name__ == '__main__':
    rospy.init_node("State_Machine_1.0")

    sm = smach.StateMachine(outcomes=['end'])

    # Open the container
    with sm:
        # Add states to the container
        # Provide the name of the state, the class from which it is derived,
        # and the next state for each possible outcome
        smach.StateMachine.add('Idle', Idle(), 
                               transitions={'Start':'GetPose', 'StayIdle':'Idle'})
        smach.StateMachine.add('GetPose', GetPose(), 
                               transitions={'success':'end'})

    # Execute SMACH plan
    outcome = sm.execute()

