#! /usr/bin/env python3
'''This is a ros service that will map chess board coordinate requested by any other nodes to a transform relative to tag1 (an april tag on the chess board)'''
import rospy
from rbx1_scripts.srv import PoseService, PoseServiceResponse, PoseServiceRequest
from math import sqrt
from geometry_msgs.msg import Pose

## map chess board grid coordinate to transform relative to tag1
dist = 27.5 #unit mm
matrix = [0, 0, 0, 0, 0, 0, 1]
target_posedict = {}
letters = ["A", "B", "C", "D", "E", "F", "G", "H"]
for j in range(8):
    for k in range(8):
        matrix = matrix.copy() #note that if we don't copy, there are dependencies in between dictionary elements as they can also take variables input
        matrix[0] = j * dist
        matrix[1] = k * dist
        my_key = "%s%s" % (letters[j], k + 1)
        target_posedict[my_key] = matrix

               
# Function that handles the request for the service, request will be in form of chess board grid coordinate (e.g. B2, F5)
def pose_request(request):
    r = request.target_location
    
    rospy.loginfo("Request received, checking target_pose for location %s", r)
    p = target_posedict[r]
    
    target_pose = Pose()
    target_pose.position.x = p[0]
    target_pose.position.y = p[1]
    target_pose.position.z = p[2]
    target_pose.orientation.x = p[3]
    target_pose.orientation.y = p[4]
    target_pose.orientation.z = p[5]
    target_pose.orientation.w = p[6]
    return target_pose


if __name__ == '__main__':

    rospy.init_node("PoseServer")
    srv = rospy.Service("PoseService",PoseService,pose_request)
    rospy.spin()

