#!/usr/bin/env python3


import rospy
import smach
from geometry_msgs.msg import Pose
import tf2_ros


#Define the state for getting pose
class GetPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['success'],input_keys=['grid_index'],output_keys = ["pose"])
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
    
    def execute(self,userdata):
        trans = self.tfBuffer.lookup_transform('map', "tag1", rospy.Time())
        userdata.pose = Pose()
        userdata.pose.position.x = trans.transform.translation.x
        userdata.pose.position.y = trans.transform.translation.y
        userdata.pose.position.z = trans.transform.translation.z
        userdata.pose.orientation.x = trans.transform.rotation.x
        userdata.pose.orientation.y = trans.transform.rotation.y
        userdata.pose.orientation.z = trans.transform.rotation.z
        userdata.pose.orientation.w = trans.transform.rotation.w
        rospy.loginfo("Robot orientation w: %d", userdata.pose.orientation.w)
        return 'success'
        