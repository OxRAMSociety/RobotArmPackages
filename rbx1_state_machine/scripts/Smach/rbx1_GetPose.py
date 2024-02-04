#!/usr/bin/env python3


import rospy
import smach
from geometry_msgs.msg import Pose
import tf2_ros

from rbx1_scripts.msg import executePoseGoalAction, executePoseGoalGoal, executePoseGoalResult
from rbx1_scripts.srv import PoseService

#Define the state for getting pose
class GetPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['success'],input_keys=['pose'],output_keys = ["pose"])
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.request = rospy.ServiceProxy('PoseService', PoseService)
    def execute(self,userdata):
        trans = self.tfBuffer.lookup_transform('map', "tag1", rospy.Time())
        userdata.tag1_to_map = Pose()
        userdata.tag1_to_map.position.x = trans.transform.translation.x
        userdata.tag1_to_map.position.y = trans.transform.translation.y
        userdata.tag1_to_map.position.z = trans.transform.translation.z
        userdata.tag1_to_map.orientation.x = trans.transform.rotation.x
        userdata.tag1_to_map.orientation.y = trans.transform.rotation.y
        userdata.tag1_to_map.orientation.z = trans.transform.rotation.z
        userdata.tag1_to_map.orientation.w = trans.transform.rotation.w
        userdata.target_to_map = Pose()
        userdata.target_location = "A1"
        target_to_tag1 = self.request(userdata.target_location)
        userdata.target_to_map.position.x = trans.transform.translation.x + target_to_tag1.transform.translation.x
        userdata.target_to_map.position.y = trans.transform.translation.y + target_to_tag1.transform.translation.y
        userdata.target_to_map.position.z = trans.transform.translation.z + target_to_tag1.transform.translation.z
        userdata.target_to_map.orientation.x = trans.transform.rotation.x + target_to_tag1.transform.rotation.x
        userdata.target_to_map.orientation.y = trans.transform.rotation.y + target_to_tag1.transform.rotation.y
        userdata.target_to_map.orientation.z = trans.transform.rotation.z + target_to_tag1.transform.rotation.z
        userdata.target_to_map.orientation.w = trans.transform.rotation.w + target_to_tag1.transform.rotation.w
        
        rospy.loginfo("Robot orientation w: %d", userdata.target_to_map.orientation.w)

        return 'success'
        