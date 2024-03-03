#! /usr/bin/env python3

import rospy
from rbx1_scripts.srv import PoseService, PoseServiceResponse, PoseServiceRequest
from math import sqrt
from geometry_msgs.msg import Pose


# matrix = [0, 0, 0, 0, 0, 0, 1]
# target_posedict = {}
# letters = ["A", "B", "C", "D", "E", "F", "G", "H"]
# for j in range(0, 8):
#     for k in range(0, 8):
#         my_key = "%s%s" % (letters[j], k + 1)
#         target_posedict["%s" % (my_key)] = matrix

#dist = int(input('sidelength'))
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

