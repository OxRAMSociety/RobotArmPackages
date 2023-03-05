#! /usr/bin/env python

import rospy
from rbx1_scripts.srv import PoseService, PoseServiceResponse, PoseServiceRequest
from math import sqrt
from geometry_msgs.msg import Pose

target_posedict = {"A1" : [0, 0 ,0 ,0 ,0, 0, 1]}
               

def pose_request(request):
    r = request.target_location
    
    rospy.loginfo("Request received, checking target_pose for location %s", r)
    p = target_posedict[r]
    
    target_pose = PoseServiceResponse()
    target_pose.position.x = p[0]
    target_pose.position.y = p[1]
    target_pose.position.z = p[2]
    target_pose.orentation.x = p[3]
    target_pose.orentation.y = p[4]
    target_pose.orentation.z = p[5]
    target_pose.orentation.w = p[6]
    return PoseServiceResponse(target_pose)


if __name__ == '__main__':

    rospy.init_node("PoseServer")
    srv = rospy.Service("PoseService",PoseService,pose_request)
    rospy.spin()

